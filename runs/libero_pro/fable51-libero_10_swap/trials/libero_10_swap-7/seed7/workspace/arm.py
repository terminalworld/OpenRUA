#!/usr/bin/env python3
"""Reusable arm helper: FK / IK / trajectory / gripper / servo, one node.

World <-> base conversion uses the fixed world->panda_link0 transform
read from /tf at import time. Poses given to ik/move are WORLD-frame
TCP (fingertip) poses unless at="hand".
"""
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
MAX_JOINT_RATE = 0.15  # rad/s the controller actually achieves (measured)
# top-down grasp: hand +Z -> world -Z, fingers open along world Y
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def q_down_yaw(yaw):
    """Top-down grasp rotated by yaw (rad) about world Z."""
    qz = (0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2))
    return quat_mul(qz, Q_DOWN)


# MACHINE FACT (measured from camera clouds): IK/FK poses are for
# panda_link8, and panda_hand is rotated -45 deg about z relative to it,
# so under Q_DOWN the fingers open along world (0.71, -0.71), NOT y.
# q_grasp() hides that: `finger_axis` is the world-plane angle of the
# finger opening axis (0 = fingers along x, pi/2 = along y); `tilt` leans
# the hand about the world axis perpendicular to the fingers (positive =
# fingertips toward -x for finger_axis=pi/2), keeping the pads vertical.
def q_grasp(finger_axis=np.pi / 2, tilt=0.0):
    yaw = (finger_axis + np.pi / 4 + np.pi / 2) % np.pi - np.pi / 2  # fingers are symmetric: yaw mod pi, nearest 0
    base = q_down_yaw(yaw)
    if abs(tilt) < 1e-9:
        return base
    ax = np.array([np.cos(finger_axis), np.sin(finger_axis), 0.0])  # tilt axis = finger axis
    s, c = np.sin(tilt / 2), np.cos(tilt / 2)
    return quat_mul((ax[0] * s, ax[1] * s, ax[2] * s, c), base)


def R_to_quat(R):
    """3x3 rotation -> quaternion (x,y,z,w)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def q_from_axes(finger_axis, approach_axis):
    """link8 quaternion for a hand whose fingers open along `finger_axis`
    (world) and whose fingertips point along `approach_axis` (world).
    e.g. q_from_axes((1,0,0), (0,1,0)) = horizontal hand pointing +y,
    fingers closing along x. Accounts for the hand's -45deg offset."""
    z = np.array(approach_axis, float); z /= np.linalg.norm(z)
    y = np.array(finger_axis, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    R_hand = np.column_stack([x, y, z])
    Rz45 = quat_to_R((0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)))  # hand = link8*Rz(-45) -> link8 = hand*Rz(+45)
    return R_to_quat(R_hand @ Rz45)


def slerp(q0, q1, t):
    q0, q1 = np.array(q0, float), np.array(q1, float)
    d = q0.dot(q1)
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        r = q0 + t * (q1 - q0)
        return tuple(r / np.linalg.norm(r))
    th = np.arccos(d)
    return tuple((np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th))


def finger_axis_world(q_link8):
    """World direction along which the fingers open, for a link8 quat."""
    Rz = quat_to_R((0.0, 0.0, np.sin(-np.pi / 8), np.cos(-np.pi / 8)))  # hand = link8 * Rz(-45deg)
    return (quat_to_R(q_link8) @ Rz)[:, 1]


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self._base = None
        self.node.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        t0 = time.time()
        while (self._js is None or self._base is None) and time.time() - t0 < 15:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._base is None:
            self._base = np.array([-0.51, 0.0, 0.42])
            print("WARN: no world->panda_link0 on /tf, using manifest default")
        print("base in world:", self._base)

    def _on_js(self, msg):
        self._js = msg

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == "panda_link0" and t.header.frame_id == "world":
                tr = t.transform.translation
                self._base = np.array([tr.x, tr.y, tr.z])

    # ---- sensing ----
    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in JOINTS]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def hand_pose(self, at="tcp"):
        """World-frame position of hand or TCP + quaternion (x,y,z,w)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        seed = JointState()
        seed.name = list(JOINTS)
        seed.position = self.arm_q()
        req.robot_state.joint_state = seed
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])  # FK is already world-frame
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        if at == "tcp":
            pos = pos + TCP * quat_to_R(q)[:, 2]
        return pos, q

    # ---- planning ----
    def solve_ik(self, pos_world, q=Q_DOWN, at="tcp", seed=None):
        pos = np.array(pos_world, dtype=float)
        if at == "tcp":
            pos = pos - TCP * quat_to_R(q)[:, 2]
        # IK poses are world-frame on this machine (verified against FK)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        s = JointState()
        s.name = list(JOINTS)
        s.position = list(seed) if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state = s
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    # ---- acting ----
    def settle(self, max_reads=60):
        """Poll joint states until the arm stops moving; return final q."""
        prev = np.array(self.arm_q())
        still = 0
        for _ in range(max_reads):
            q = np.array(self.arm_q())
            if np.abs(q - prev).max() < 1e-4:
                still += 1
                if still >= 3:
                    break
            else:
                still = 0
            prev = q
        return prev

    def move_joints(self, targets, seconds=3.0, tol=0.01, retries=3):
        """targets: list of joint vectors (waypoints) or a single vector.
        Sends the goal, waits for the result, waits for the arm to stop,
        and re-sends the final target if it did not converge (-5 on a
        long goal is usually controller lag, see docs/30-action.md)."""
        if not isinstance(targets[0], (list, tuple, np.ndarray)):
            targets = [targets]
        final = np.array(targets[-1], dtype=float)
        cmd = final.copy()   # what we actually send; offset by the sag if needed
        # the controller tracks at roughly 0.2 rad/s; give it the time
        dist = np.abs(final - np.array(self.arm_q())).max()
        seconds = max(seconds, dist / MAX_JOINT_RATE)
        for attempt in range(retries):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINTS)
            n = len(targets)
            for i, tq in enumerate(targets):
                pt = JointTrajectoryPoint(positions=[float(v) for v in tq])
                t = seconds * (i + 1) / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                goal.trajectory.points.append(pt)
            fut = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            gh = fut.result()
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
            code = rf.result().result.error_code
            q = self.settle()
            err = np.abs(q - final).max()
            print(f"  traj error_code={code} max_joint_err={err:.4f} (attempt {attempt + 1})")
            if err <= tol:
                return code, err
            # MACHINE FACT: with the arm low and extended, the controller
            # settles short of the command on the gravity-loaded joints
            # (2 and 4) with no contact and error_code 0. Re-sending the
            # same point does nothing; add the residual to the command.
            cmd = cmd + (final - q)
            targets = [cmd]
            seconds = max(2.0, err / MAX_JOINT_RATE)
        return code, err

    def move_to(self, pos_world, q=Q_DOWN, seconds=3.0, at="tcp", seed=None,
                steps=1, tol_m=0.01):
        """Move the TCP (or hand) to a world pose. steps>1 interpolates a
        straight Cartesian line from the current pose, IK-seeding each
        waypoint with the previous one for joint-space continuity."""
        target = np.array(pos_world, dtype=float)
        sols = []
        if steps > 1:
            start, _ = self.hand_pose(at)
            prev = self.arm_q()
            for i in range(1, steps + 1):
                wp = start + (target - start) * i / steps
                sol = self.solve_ik(wp, q, at, seed=prev)
                if sol is None:
                    print(f"  IK FAILED for waypoint {np.round(wp, 3)}")
                    return False
                sols.append(sol)
                prev = sol
        else:
            sol = self.solve_ik(target, q, at, seed)
            if sol is None:
                print(f"  IK FAILED for {np.round(target, 3)}")
                return False
            sols = [sol]
        self.move_joints(sols, seconds)
        p, _ = self.hand_pose(at)
        d = np.linalg.norm(p - target)
        print(f"  now at {at} {np.round(p, 4)} (target {np.round(target, 4)}, off {d*1000:.1f} mm)")
        return d <= tol_m

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        g = self.finger_gap()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={g}")
        return g

    def link8_quat(self):
        """Current link8 orientation (hand = link8 * Rz(-45deg))."""
        _, qh = self.hand_pose()
        return quat_mul(qh, (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)))

    def move_pose(self, pos_world, q, steps=6, seconds=6.0, max_jump=0.6, tol_m=0.01):
        """Move TCP to (pos, link8 quat) interpolating position linearly and
        orientation by slerp from the current pose; IK per waypoint seeded
        by the previous one. MACHINE FACT: KDL IK flips branches without
        warning, which produced a wild swing once - so refuse if two
        consecutive waypoints differ by more than max_jump rad."""
        p0, _ = self.hand_pose()
        q0 = self.link8_quat()
        target = np.array(pos_world, float)
        seed = self.arm_q()
        wps = []
        for t in np.linspace(0, 1, steps + 1)[1:]:
            sol = self.solve_ik(p0 + t * (target - p0), q=slerp(q0, q, t), seed=seed)
            if sol is None:
                print(f"  move_pose: no IK at t={t:.2f}")
                return False
            jump = np.abs(np.array(sol) - np.array(seed)).max()
            if jump > max_jump:
                print(f"  move_pose: branch jump {jump:.2f} rad at t={t:.2f}; refusing")
                return False
            wps.append(sol)
            seed = sol
        self.move_joints(wps, seconds=seconds)
        p, _ = self.hand_pose()
        d = np.linalg.norm(p - target)
        print(f"  now at tcp {np.round(p, 4)} (target {np.round(target, 4)}, off {d*1000:.1f} mm)")
        return d <= tol_m

    def servo(self, v, n=20):
        """Stream n twist messages with linear velocity v (m/s, base frame)."""
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
