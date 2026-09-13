#!/usr/bin/env python3
"""Reusable control helpers for this Panda (one node, clients built once)."""
import sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import WrenchStamped

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])
TCP_OFF = 0.1034


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    m = R
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s; x = (m[2, 1] - m[1, 2]) / s; y = (m[0, 2] - m[2, 0]) / s; z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s; x = 0.25 * s; y = (m[0, 1] + m[1, 0]) / s; z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s; x = (m[0, 1] + m[1, 0]) / s; y = 0.25 * s; z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s; x = (m[0, 2] + m[2, 0]) / s; y = (m[1, 2] + m[2, 1]) / s; z = 0.25 * s
    q = np.array([x, y, z, w]); return q / np.linalg.norm(q)


def rot_from_axes(approach, closing):
    """Hand rotation: z = approach, y = finger closing dir, x = y cross z."""
    z = np.array(approach, float); z /= np.linalg.norm(z)
    y = np.array(closing, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.stack([x, y, z], 1)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = None
        self._wrench = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_w, 10)
        self.traj = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_w(self, m): self._wrench = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            end = time.time() + 10
            while self._js is None and time.time() < end:
                self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wrench = None
        end = time.time() + 5
        while self._wrench is None and time.time() < end:
            self.spin(0.1)
        if self._wrench is None:
            return None
        f = self._wrench.wrench.force; t = self._wrench.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def fk(self, q=None):
        """hand pose in WORLD: (pos, R)."""
        if q is None: q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])  # already world
        R = quat_to_R([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk(q)
        return pos + TCP_OFF * R[:, 2], R

    def ik(self, hand_pos_world, R, seed=None, timeout=30, avoid_collisions=False):
        """returns joint array or None"""
        if seed is None: seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        bp = np.array(hand_pos_world)  # MoveIt model frame == world here (verified by FK)
        p.position.x, p.position.y, p.position.z = map(float, bp)
        q = R_to_quat(R)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = bool(avoid_collisions)
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            print("IK: no answer"); return None
        if res.error_code.val != 1:
            print(f"IK failed code={res.error_code.val}"); return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    def ik_tcp(self, tcp_pos_world, R, seed=None, avoid_collisions=False):
        hand = np.array(tcp_pos_world) - TCP_OFF * R[:, 2]
        return self.ik(hand, R, seed, avoid_collisions=avoid_collisions)

    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if waypoints:
            for (wq, wt) in waypoints:
                pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
                pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            print("traj goal rejected"); return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = r.result.error_code if r else None
        err = np.abs(self.arm_q() - np.array(q)).max()
        print(f"move_q: code={code} max_err={err:.4f}")
        return code

    def move_tcp(self, tcp_pos, R, seconds=3.0, seed=None):
        q = self.ik_tcp(tcp_pos, R, seed)
        if q is None:
            return None
        code = self.move_q(q, seconds)
        p, _ = self.tcp()
        print(f"  tcp now {np.round(p,4)} target {np.round(tcp_pos,4)}")
        return code

    def gripper(self, width, effort=30.0):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(effort)
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f


DOWN = rot_from_axes([0, 0, -1], [0, 1, 0])   # approach down, fingers close along y
DOWN_X = rot_from_axes([0, 0, -1], [1, 0, 0])  # approach down, fingers close along x


REF = np.array([0.0, 0.0, 0.0, -2.0, 0.0, 2.2, 0.785])
SEEDS = [
    np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),
    np.array([0.0, 0.0, 0.0, -2.0, 0.0, 2.2, 0.785]),
    np.array([0.0, 0.5, 0.0, -2.2, 0.0, 2.7, 0.785]),
    np.array([0.0, 1.0, 0.0, -1.8, 0.0, 2.8, 0.785]),
    np.array([0.0, 0.3, 0.0, -1.5, 0.0, 1.8, 0.785]),
    np.array([0.0, -0.3, 0.0, -2.6, 0.0, 2.3, 0.785]),
]


def best_ik(r, tcp_pos, R, extra_seeds=(), n_rand=6, ref=REF, w=(3, 1, 3, 1, 3, 1, 2), verbose=False):
    """Try many seeds; return the solution closest (weighted) to a natural reference config."""
    rng = np.random.default_rng(0)
    seeds = list(extra_seeds) + SEEDS
    for _ in range(n_rand):
        seeds.append(ref + rng.normal(0, 0.4, 7))
    best, bestc = None, 1e9
    for s in seeds:
        sol = r.ik_tcp(tcp_pos, R, seed=s)
        if sol is None:
            continue
        # wrap j7 preference: near 0.785 or -2.356 equivalent? keep simple: distance to ref
        c = float(np.sum(np.array(w) * (sol - ref) ** 2))
        if verbose:
            print("  cand", np.round(sol, 2), round(c, 2))
        if c < bestc:
            best, bestc = sol, c
    return best


LINKS = ["panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5",
         "panda_link6", "panda_link7", "panda_hand"]


def fk_links(r, q, links=LINKS):
    req = GetPositionFK.Request()
    req.fk_link_names = list(links)
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk_cli.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    out = {}
    for name, ps in zip(res.fk_link_names, res.pose_stamped):
        p = ps.pose
        out[name] = (np.array([p.position.x, p.position.y, p.position.z]),
                     quat_to_R([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))
    return out


def hand_points(pos, R, finger=0.04):
    """Sample points of the hand/finger volume in world (approximate box model)."""
    pts = []
    # palm box: x_h +-0.032, y_h +-0.1, z_h 0..0.066
    for xs in (-0.032, 0.032):
        for ys in (-0.1, -0.05, 0, 0.05, 0.1):
            for zs in (0.0, 0.033, 0.066):
                pts.append(pos + R @ np.array([xs, ys, zs]))
    # fingers: at y = +-finger, x +-0.01, z 0.066..0.112
    for ys in (-finger, finger):
        for zs in (0.08, 0.1, 0.112):
            for xs in (-0.01, 0.01):
                pts.append(pos + R @ np.array([xs, ys, zs]))
    return np.array(pts)


def link_clearance(r, q, finger=0.04, verbose=False):
    """Return min clearance summary vs table (z=0.90) and cabinet box, for arm links + hand."""
    fk = fk_links(r, q)
    worst = []
    for name, (p, R) in fk.items():
        if name == "panda_hand":
            pts = hand_points(p, R, finger)
            rad = 0.0
        else:
            pts = p[None, :]
            rad = 0.06 if name in ("panda_link4", "panda_link5", "panda_link6", "panda_link7") else 0.08
        # table
        dz = pts[:, 2].min() - rad - 0.90
        # cabinet box x[-0.12,0.13] y[0.213,0.42] z[0.90,1.13]
        inside = ((pts[:, 0] > -0.12 - rad) & (pts[:, 0] < 0.13 + rad) & (pts[:, 1] > 0.213 - rad)
                  & (pts[:, 1] < 0.42 + rad) & (pts[:, 2] < 1.13 + rad)).any()
        worst.append((name, round(float(dz), 3), bool(inside)))
    if verbose:
        for w in worst: print("   ", w)
    return worst


def ik_near(r, tcp_pos, R, prev, max_dq=0.5, tries=10, avoid_collisions=True, rng=np.random.default_rng(3)):
    """IK solution continuous with prev (max joint jump < max_dq), or None."""
    best = None
    for i in range(tries):
        seed = prev if i == 0 else prev + rng.normal(0, 0.05 * i, 7)
        sol = r.ik_tcp(tcp_pos, R, seed=seed, avoid_collisions=avoid_collisions)
        if sol is None:
            continue
        d = np.abs(sol - prev).max()
        if d < max_dq and (best is None or d < np.abs(best - prev).max()):
            best = sol
            if d < max_dq / 2:
                break
    return best


def tilt_R(deg, closing=(0, -1, 0)):
    """side-grasp orientation: approach +x tilted deg below horizontal."""
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return rot_from_axes([c, 0, -s], closing)


from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState as _RobotState

def state_valid(r, q, finger=0.04, verbose=False):
    """MoveIt /check_state_validity (self-collision + joint limits). Returns (valid, contact pairs)."""
    if not hasattr(r, 'sv_cli'):
        r.sv_cli = r.node.create_client(GetStateValidity, '/check_state_validity')
        r.sv_cli.wait_for_service(10)
    req = GetStateValidity.Request()
    js = JointState(); js.name = list(ARM) + ['panda_finger_joint1', 'panda_finger_joint2']
    js.position = [float(v) for v in q] + [finger, finger]
    req.robot_state = _RobotState(joint_state=js)
    req.group_name = 'panda_arm'
    fut = r.sv_cli.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    pairs = [(c.contact_body_1, c.contact_body_2) for c in res.contacts]
    if verbose: print('valid', res.valid, pairs)
    return res.valid, pairs


def path_valid(r, qs, step=0.05, finger=0.04):
    """check interpolated joint path for self-collision; returns list of (idx, t, pairs) failures."""
    bad = []
    for i, (a, b) in enumerate(zip(qs[:-1], qs[1:])):
        n = max(2, int(np.ceil(np.abs(b - a).max() / step)))
        for t in np.linspace(0, 1, n + 1):
            v, pairs = state_valid(r, a + (b - a) * t, finger)
            if not v: bad.append((i, round(float(t), 3), pairs))
    return bad
