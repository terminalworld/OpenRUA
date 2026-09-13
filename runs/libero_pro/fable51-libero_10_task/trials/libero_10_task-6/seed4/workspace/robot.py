"""Helpers: FK/IK, trajectory, gripper, joint state, all on one node."""
import time, math, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = ARM["joints"]
BASE_T = np.zeros(3)   # FK/IK on this machine already report in the world frame (verified)
TCP = M["hand"]["tcp_offset_m"]

def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def quat_mul(q1, q2):
    x1,y1,z1,w1 = q1; x2,y2,z2,w2 = q2
    return np.array([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2,
                     w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2])

def quat_z(yaw):
    return np.array([0, 0, math.sin(yaw/2), math.cos(yaw/2)])

class Robot:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, ARM["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)
        self.joints()

    def _on_js(self, m): self._js = m

    def spin(self, t=0.05): rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None: self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return np.array([d[j] for j in JOINTS]), d

    def fingers(self):
        _, d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def _seed(self, q):
        js = JointState(); js.name = list(JOINTS); js.position = [float(v) for v in q]
        return js

    def fk_hand(self, q=None):
        """Hand pose in WORLD frame: (pos[3], quat[4])."""
        if q is None: q, _ = self.joints()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result(); assert r is not None and r.error_code.val == 1, r
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_T
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik_hand(self, pos_w, quat, seed=None, timeout=30):
        """IK for hand at world pos/quat. Returns joint array or None."""
        if seed is None: seed, _ = self.joints()
        p = np.asarray(pos_w, float) - BASE_T
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        po = req.ik_request.pose_stamped.pose
        po.position.x, po.position.y, po.position.z = map(float, p)
        po.orientation.x, po.orientation.y, po.orientation.z, po.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return np.array([d[j] for j in JOINTS])

    def ik_tcp(self, tcp_w, quat, **kw):
        R = quat_R(*quat)
        return self.ik_hand(np.asarray(tcp_w) - TCP * R[:, 2], quat, **kw)

    def move(self, waypoints, times):
        """waypoints: list of joint arrays; times: cumulative seconds."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for q, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        q, _ = self.joints()
        err = np.abs(q - waypoints[-1]).max()
        print(f"  move done code={code} max_joint_err={err:.4f}")
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def go_tcp(self, tcp_w, quat, seconds, seed=None):
        q = self.ik_tcp(tcp_w, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED for tcp {np.round(tcp_w,3)}"); return None
        self.move([q], [seconds])
        pos, _ = self.fk_hand()
        R = quat_R(*quat); tcp = pos + TCP * R[:, 2]
        print(f"  tcp now {np.round(tcp,4)} target {np.round(tcp_w,4)}")
        return q

    def go_tcp_line(self, tcp_from, tcp_to, quat, seconds, n=4, seed=None):
        """Straight-line TCP motion via n IK waypoints in one trajectory."""
        qs, ts = [], []
        s = seed if seed is not None else self.joints()[0]
        for i in range(1, n+1):
            a = i / n
            p = np.asarray(tcp_from) * (1-a) + np.asarray(tcp_to) * a
            q = self.ik_tcp(p, quat, seed=s)
            if q is None:
                print(f"  IK FAILED at line waypoint {np.round(p,3)}"); return None
            qs.append(q); ts.append(seconds * a); s = q
        self.move(qs, ts)
        pos, _ = self.fk_hand()
        R = quat_R(*quat); tcp = pos + TCP * R[:, 2]
        print(f"  tcp now {np.round(tcp,4)} target {np.round(tcp_to,4)}")
        return qs[-1]
