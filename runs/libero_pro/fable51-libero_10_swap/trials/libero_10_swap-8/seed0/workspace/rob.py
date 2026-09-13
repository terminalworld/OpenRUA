#!/usr/bin/env python3
"""Robot helper: joint state, FK, IK, trajectory, gripper (one node, reused clients)."""
import math, sys, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE = np.zeros(3)  # verified: /compute_fk & /compute_ik on this machine work in WORLD coords (link0 at (-0.66,0,0.912))
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time() * 1000) % 100000))
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm(self):
        d = self.joints()
        return np.array([d[j] for j in JOINTS])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    # ---- kinematics (poses in WORLD frame; converted to base frame for MoveIt)
    def fk(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(map(float, self.arm() if q is None else q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def ik(self, pos_world, quat, seed=None, timeout=5.0):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos_world, float) - BASE
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = list(map(float, self.arm() if seed is None else seed))
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    # ---- motion
    def move_joints(self, targets, seconds, wait=True):
        """targets: list of joint arrays (waypoints); seconds: list of times or total."""
        if isinstance(targets, np.ndarray) and targets.ndim == 1:
            targets = [targets]
        if not isinstance(seconds, (list, tuple)):
            n = len(targets)
            seconds = [seconds * (i + 1) / n for i in range(n)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for q, t in zip(targets, seconds):
            pt = JointTrajectoryPoint(positions=list(map(float, q)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        if not wait:
            return gh
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        q = self.arm()
        err = np.abs(q - np.asarray(targets[-1])).max()
        print(f"  traj done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, seed=None, via=None):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {pos}")
        cur = self.arm()
        # keep joint7 / wrist continuity: unwrap solution near current config
        for i in (0, 2, 4, 6):
            while q[i] - cur[i] > math.pi: q[i] -= 2 * math.pi
            while q[i] - cur[i] < -math.pi: q[i] += 2 * math.pi
        lim = FJT["limits_rad"]
        for i, (lo, hi) in enumerate(lim):
            if not (lo - 1e-3 <= q[i] <= hi + 1e-3):
                raise RuntimeError(f"IK solution violates limit joint{i+1}: {q[i]:.3f} not in [{lo},{hi}]")
        r = self.move_joints([q], seconds)
        pos2, quat2 = self.fk()
        print(f"  hand now at {np.round(pos2, 4)} (target {np.round(pos, 4)}) dpos={np.linalg.norm(pos2-pos):.4f}", flush=True)
        return q, pos2

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        res = rf.result().result
        f = self.fingers()
        print(f"  gripper -> {width}: reached={res.reached_goal} stalled={res.stalled} fingers={f[0]:.4f},{f[1]:.4f}", flush=True)
        return f

    def servo(self, lin, n=20, frame=None):
        """Stream n twist messages (m/s in panda_link0 frame)."""
        msg = TwistStamped()
        msg.header.frame_id = frame or "panda_link0"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)


# Standard orientations (hand frame axes expressed in world)
def quat_from_axes(x, y, z):
    R = np.column_stack([np.asarray(x, float), np.asarray(y, float), np.asarray(z, float)])
    return R_to_quat(R)

# hand z = approach direction, hand y = finger axis
Q_DOWN = quat_from_axes([1, 0, 0], [0, -1, 0], [0, 0, -1])          # pointing down, fingers along y
Q_FWD_FY_NEG = quat_from_axes([0, 0, 1], [0, -1, 0], [1, 0, 0])     # pointing +x, fingers along y
Q_FWD_FY_POS = quat_from_axes([0, 0, -1], [0, 1, 0], [1, 0, 0])     # pointing +x, fingers along y (flipped)
