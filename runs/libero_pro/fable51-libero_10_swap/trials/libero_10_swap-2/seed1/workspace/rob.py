#!/usr/bin/env python3
"""Small helper library: joint state, FK, IK, trajectory, gripper, servo."""
import math, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_down(yaw):
    """Hand pointing straight down, fingers closing along world direction at angle `yaw`
    from world y (yaw=0 -> fingers along y, yaw=pi/2 -> fingers along x)."""
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    # qz(yaw) * (1,0,0,0)
    return (c, s, 0.0, 0.0)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin(0.1)
        f = self._wr.wrench.force; t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk_pose(self, q=None):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in (q or self.arm_q())]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z]), \
            (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp(self, q=None):
        p, quat = self.fk_pose(q)
        R = quat_R(*quat)
        return p + TCP * R[:, 2], quat

    def ik_hand(self, xyz, quat, seed=None, timeout=20.0):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.ik_link_name = "panda_hand"
        seed = seed or self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=90)
        r = fut.result()
        if r is None:
            raise RuntimeError("IK no answer")
        if r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp(self, xyz, quat, seed=None, tries=5):
        R = quat_R(*quat)
        hand = np.array(xyz, float) - TCP * R[:, 2]
        seed = seed or self.arm_q()
        best = None
        for i in range(tries):
            q = self.ik_hand(hand, quat, seed=seed)
            if q is not None:
                # verify
                t, _ = self.tcp(q)
                err = np.linalg.norm(t - np.array(xyz))
                if err < 0.005:
                    if best is None or self._dist(q, seed) < self._dist(best, seed):
                        best = q
                    if i >= 1:
                        break
            seed = [s + np.random.uniform(-0.3, 0.3) for s in seed]
        return best

    @staticmethod
    def _dist(a, b):
        return float(np.linalg.norm(np.array(a) - np.array(b)))

    def move_q(self, q, seconds=None, verbose=True, retry=2):
        cur = self.arm_q()
        d = max(abs(a - b) for a, b in zip(q, cur))
        if seconds is None:
            seconds = max(1.5, min(14.0, d / 0.2))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        after = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q, after))
        if verbose:
            print(f"move_q: code={code} maxerr={err:.4f} ({seconds:.1f}s)", flush=True)
        if code != 0 and err > 0.02 and retry > 0:
            return self.move_q(q, seconds=max(seconds, 2.0) * 1.5, verbose=verbose, retry=retry - 1)
        return code, err

    def move_tcp(self, xyz, quat, seconds=None, seed=None):
        q = self.ik_tcp(xyz, quat, seed=seed)
        if q is None:
            print("IK FAILED for", xyz, quat, flush=True)
            return None
        code, err = self.move_q(q, seconds)
        t, _ = self.tcp()
        print(f"  tcp now {t.round(4)} target {np.round(xyz, 4)}", flush=True)
        return q

    def gripper(self, width, wait=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=wait)
        r = res.result().result
        f = self.fingers()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def servo(self, v, n=20, w=(0, 0, 0)):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, w)
        for _ in range(n):
            self.twist.publish(msg); self.spin(0.05)
