#!/usr/bin/env python3
"""Small control library: IK (hand frame, base-frame poses), FK, trajectories,
gripper, joint state. World<->base offset from TF (panda_link0 in world)."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetCartesianPath
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # MoveIt model frame == world (verified via FK vs TF)
TCP = float(M["hand"]["tcp_offset_m"])
DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z -> world -z, hand y -> world -y (finger axis = world y)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def qmul(a, b):
    """quaternion product, (x,y,z,w) convention"""
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def pitch_down(theta):
    """DOWN orientation tilted about world y by theta: hand approach axis
    becomes (-sin t, 0, -cos t), i.e. leaning in from +x for t>0."""
    ry = (0.0, np.sin(theta / 2), 0.0, np.cos(theta / 2))
    return qmul(ry, DOWN)


def slerp(qa, qb, t):
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    d = np.dot(qa, qb)
    if d < 0: qb, d = -qb, -d
    if d > 0.9995:
        r = qa + t * (qb - qa); return tuple(r / np.linalg.norm(r))
    th = np.arccos(d)
    return tuple((np.sin((1 - t) * th) * qa + np.sin(t * th) * qb) / np.sin(th))


def yaw_down(yaw):
    """Quaternion: DOWN orientation then rotated by yaw about world z."""
    # q = Rz(yaw) * Rx(pi)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    # Rz(yaw) = (0,0,sy,cy); Rx(pi) = (1,0,0,0); product (w1w2 - v1.v2, ...)
    w = -0.0 * 0 + cy * 0 - 0  # placeholder, compute properly below
    q1 = np.array([cy, 0, 0, sy])  # w,x,y,z
    q2 = np.array([0, 1, 0, 0])
    w = q1[0] * q2[0] - np.dot(q1[1:], q2[1:])
    v = q1[0] * q2[1:] + q2[0] * q1[1:] + np.cross(q1[1:], q2[1:])
    return (v[0], v[1], v[2], w)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, M["sensors"][1]["port"], self._on_wr, 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.cart = self.node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.spin(0.5)

    def _on_js(self, m):
        self._js["m"] = m

    def _on_wr(self, m):
        self._wr["m"] = m

    def spin(self, t):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self._js.clear()
        while "m" not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr.clear()
        end = time.time() + 3
        while "m" not in self._wr and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if "m" not in self._wr:
            return None
        f = self._wr["m"].wrench.force
        return np.array([f.x, f.y, f.z])

    # ---- kinematics ----
    def ik_hand_world(self, pos_w, quat, seed=None):
        """IK for the panda_hand frame at world position pos_w."""
        pos_b = np.asarray(pos_w) - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        s = JointState()
        s.name = list(JOINTS)
        s.position = [float(v) for v in (seed if seed is not None else self.arm_q())]
        req.ik_request.robot_state.joint_state = s
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"IK failed {None if res is None else res.error_code.val} for {pos_w}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def ik_tcp_world(self, tcp_w, quat, seed=None):
        R = quat_R(*quat)
        hand = np.asarray(tcp_w) - TCP * R[:, 2]
        return self.ik_hand_world(hand, quat, seed)

    def fk_world(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        s = JointState(); s.name = list(JOINTS)
        s.position = [float(v) for v in (q if q is not None else self.arm_q())]
        req.robot_state.joint_state = s
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError("FK failed")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, quat

    def tcp_world(self, q=None):
        pos, quat = self.fk_world(q)
        R = quat_R(*quat)
        return pos + TCP * R[:, 2], quat

    # ---- motion ----
    def traj(self, qs, times):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for q, t in zip(qs, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(qs[-1])).max()
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code

    def move_tcp_line(self, tcp_from, tcp_to, quat, n=4, speed=0.15, seed=None, min_t=1.0):
        """Straight TCP line via n IK waypoints, one multi-point trajectory."""
        tcp_from, tcp_to = np.asarray(tcp_from), np.asarray(tcp_to)
        seed = list(seed if seed is not None else self.arm_q())
        qs, ts = [], []
        dist = np.linalg.norm(tcp_to - tcp_from)
        total = max(min_t, dist / speed)
        for i in range(1, n + 1):
            a = i / n
            p = tcp_from + a * (tcp_to - tcp_from)
            q = self.ik_tcp_world(p, quat, seed)
            seed = q
            qs.append(q); ts.append(total * a)
        qs.append(qs[-1]); ts.append(total + 1.0)  # hold point: let the controller settle
        code = self.traj(qs, ts)
        pos, _ = self.tcp_world()
        print(f"  tcp now {np.round(pos, 4)} target {np.round(tcp_to, 4)} err {np.linalg.norm(pos - tcp_to):.4f}")
        return code

    def cart_path(self, tcp_pts, quats, start_q=None, max_step=0.01):
        """Joint-continuous Cartesian path through TCP waypoints via MoveIt.
        Returns (list of q, fraction)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.group_name = M["planning"]["group"]
        req.link_name = "panda_hand"
        s = JointState(); s.name = list(JOINTS)
        s.position = [float(v) for v in (start_q if start_q is not None else self.arm_q())]
        req.start_state.joint_state = s
        for p, quat in zip(tcp_pts, quats):
            R = quat_R(*quat)
            hand = np.asarray(p) - TCP * R[:, 2]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, hand)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quat)
            req.waypoints.append(pose)
        req.max_step = max_step
        req.jump_threshold = 3.0
        req.avoid_collisions = False
        fut = self.cart.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
        res = fut.result()
        if res is None:
            raise RuntimeError("cartesian path service timeout")
        jt = res.solution.joint_trajectory
        idx = [jt.joint_names.index(j) for j in JOINTS]
        qs = [[pt.positions[i] for i in idx] for pt in jt.points]
        return qs, res.fraction, res.error_code.val

    def move_cart(self, tcp_to, quat, speed=0.08, quat_from=None, n_rot=1, min_t=1.0):
        """Move TCP on a straight line (optionally interpolating orientation)
        using compute_cartesian_path; retimed at constant speed; hold point."""
        pos, q0 = self.tcp_world()
        tcp_to = np.asarray(tcp_to)
        pts, quats = [], []
        if quat_from is None:
            pts, quats = [tcp_to], [quat]
        else:
            for i in range(1, n_rot + 1):
                a = i / n_rot
                pts.append(pos + a * (tcp_to - pos))
                quats.append(slerp(quat_from, quat, a))
        qs, frac, code = self.cart_path(pts, quats)
        if frac < 0.999 or len(qs) < 2:
            raise RuntimeError(f"cartesian path incomplete: fraction={frac:.3f} code={code} n={len(qs)}")
        # joint-space continuity check
        dq = np.abs(np.diff(np.array(qs), axis=0)).max()
        dist = np.linalg.norm(tcp_to - pos)
        total = max(min_t, dist / speed)
        ts = list(np.linspace(0, total, len(qs))[1:])
        qs = qs[1:]
        qs.append(qs[-1]); ts.append(total + 1.0)
        print(f"  cart path: {len(qs)} pts, max dq step {dq:.3f}, T={total:.1f}s")
        code = self.traj(qs, ts)
        p2, _ = self.tcp_world()
        print(f"  tcp now {np.round(p2, 4)} target {np.round(tcp_to, 4)} err {np.linalg.norm(p2 - tcp_to):.4f}")
        return code

    def move_tcp(self, tcp_to, quat, **kw):
        pos, _ = self.tcp_world()
        return self.move_tcp_line(pos, tcp_to, quat, **kw)

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        self.spin(0.3)
        f = self.fingers()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
