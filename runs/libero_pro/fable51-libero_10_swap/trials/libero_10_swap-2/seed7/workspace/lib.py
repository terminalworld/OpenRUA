"""Reusable helpers: joint state, FK (via /compute_fk), IK, trajectory, gripper, servo."""
import time
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE = np.zeros(3)  # FK/IK model frame verified == world (TF chain matches /compute_fk)

def quat_R(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

class Robot:
    def __init__(self, name="agent"):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None; self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped, M["sensors"][1]["port"], self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.spin(0.5)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m
    def spin(self, sec=0.2):
        end = time.time() + sec
        while time.time() < end: rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self._js = None
        while self._js is None: rclpy.spin_once(self.node, timeout_sec=0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d
    def arm_q(self):
        d = self.joints(); return [d[j] for j in ARM]
    def finger_gap(self):
        d = self.joints(); return abs(d["panda_finger_joint1"]) + abs(d["panda_finger_joint2"])
    def wrench(self):
        self._wr = None
        while self._wr is None: rclpy.spin_once(self.node, timeout_sec=0.1)
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk_world(self, q=None):
        """hand pose in WORLD frame: (pos[3], quat[4])"""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = list(q if q is not None else self.arm_q())
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1: raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]), r.pose_stamped[0].header.frame_id

    def ik_world(self, pos, quat, seed=None):
        """IK for HAND pose given in WORLD frame -> list of arm joints or None"""
        pos = np.asarray(pos, float) - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        s = JointState(); s.name = list(ARM); s.position = list(seed if seed is not None else self.arm_q())
        req.ik_request.robot_state.joint_state = s
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None: print("IK: no answer"); return None
        if r.error_code.val != 1: print(f"IK failed code={r.error_code.val}"); return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_q(self, q, sec=3.0, verbose=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        if verbose: print(f"move_q: code={code} max_err={err:.4f}")
        return code, err

    def move_qs(self, qs, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, s in zip(qs, secs):
            pt = JointTrajectoryPoint(positions=[float(x) for x in q])
            pt.time_from_start = Duration(sec=int(s), nanosec=int((s % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(qs[-1])).max()
        print(f"move_qs: code={code} max_err={err:.4f}")
        return code, err

    def move_pose(self, pos, quat, sec=3.0, seed=None):
        q = self.ik_world(pos, quat, seed)
        if q is None: return None
        self.move_q(q, sec)
        p, _, _ = self.fk_world()
        print(f"  reached world pos {np.round(p,4)} (target {np.round(pos,4)})")
        return q

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}")
        return r

    def servo(self, lin=(0,0,0), ang=(0,0,0), ticks=20):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            self.twist.publish(msg); rclpy.spin_once(self.node, timeout_sec=0.05)
