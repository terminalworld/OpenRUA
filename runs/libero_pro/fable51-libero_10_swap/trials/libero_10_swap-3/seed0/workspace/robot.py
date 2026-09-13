"""Helper lib: joint state, FK/IK via MoveIt, multi-point FJT, gripper."""
import math, time
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import WrenchStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE = np.zeros(3)   # MoveIt model frame here IS world (FK header says world; verified by IK round-trip)
TCP = M["hand"]["tcp_offset_m"]

def quat_to_R(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

class Robot:
    def __init__(self, name="robot_helper"):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self._wr = None
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.05): rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None: self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints(); return [d[j] for j in JOINTS]

    def fingers(self):
        d = self.joints(); return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None: self.spin(0.2)
        f = self._wr.wrench.force; t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk_hand(self, q=None):
        """hand pose in WORLD: (pos[3], quat[4] xyzw)"""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def tcp(self, q=None):
        pos, quat = self.fk_hand(q)
        R = quat_to_R(*quat)
        return pos + TCP * R[:, 2], quat

    def ik_hand(self, pos_world, quat, seed=None, timeout=30, avoid=False):
        """IK for hand pose in WORLD -> list of joint positions or None"""
        seed = seed if seed is not None else self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"  # group tip is link8 (45 deg yaw off); target the hand explicitly
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos_world) - BASE
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = JOINTS
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = bool(avoid)
        req.ik_request.timeout = Duration(sec=0, nanosec=200_000_000)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def ik_tcp(self, tcp_world, quat, seed=None, avoid=False):
        R = quat_to_R(*quat)
        return self.ik_hand(np.asarray(tcp_world) - TCP * R[:, 2], quat, seed, avoid=avoid)

    def move_joints(self, waypoints, seconds, timeout=600):
        """waypoints: list of joint vectors; times spread evenly to `seconds`"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        n = len(waypoints)
        for i, wp in enumerate(waypoints):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result()
        code = r.result.error_code if r else None
        err = np.abs(np.array(self.arm_q()) - np.array(waypoints[-1])).max()
        return code, err

    def move_tcp_line(self, tcp_target, quat, seconds, step=0.02, seed=None, avoid=False):
        """straight-ish Cartesian line for the TCP via IK waypoints"""
        start, _ = self.tcp()
        tcp_target = np.asarray(tcp_target, float)
        dist = np.linalg.norm(tcp_target - start)
        n = max(1, int(math.ceil(dist / step)))
        seed = seed if seed is not None else self.arm_q()
        wps = []
        for i in range(1, n + 1):
            p = start + (tcp_target - start) * i / n
            q = self.ik_tcp(p, quat, seed, avoid=avoid)
            if q is None:
                raise RuntimeError(f"IK failed at waypoint {i}/{n}: {p}")
            # reject solutions that jump far from the seed (branch flips)
            if np.abs(np.array(q) - np.array(seed)).max() > 1.0:
                raise RuntimeError(f"IK branch jump at waypoint {i}/{n}: {np.round(q,3)} vs {np.round(seed,3)}")
            wps.append(q); seed = q
        return self.move_joints(wps, seconds)

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()
