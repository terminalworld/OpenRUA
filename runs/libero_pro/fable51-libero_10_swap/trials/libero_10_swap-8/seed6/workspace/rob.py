#!/usr/bin/env python3
"""Reusable robot client: joint state, IK, trajectory, gripper. Builds clients once."""
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geom import R_to_q

ARM = [f"panda_joint{i}" for i in range(1, 8)]
FJT = "/panda_arm_controller/follow_joint_trajectory"
GRIP = "/franka_gripper/gripper_action"


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT)
        self.grip = ActionClient(self.node, GripperCommand, GRIP)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(timeout_sec=20), "no FJT server"
        assert self.grip.wait_for_server(timeout_sec=20), "no gripper server"
        assert self.ik_cli.wait_for_service(timeout_sec=20), "no IK"
        assert self.fk_cli.wait_for_service(timeout_sec=20), "no FK"
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self._js = m

    def spin(self, n=3):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def joints(self):
        self.spin(5)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        d = self.joints()
        return np.array([d[a] for a in ARM])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk(self, q=None):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(q if q is not None else self.arm_q())
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    # /compute_ik solves for the panda_arm tip link (panda_link8); panda_hand is link8 rotated
    # -45deg about z (same origin). All poses in this class are panda_hand poses (as /compute_fk
    # reports), so convert before asking IK.
    HAND_TO_LINK8 = np.array([[np.cos(np.pi / 4), -np.sin(np.pi / 4), 0],
                              [np.sin(np.pi / 4), np.cos(np.pi / 4), 0], [0, 0, 1]])

    def ik(self, pos, R, seed=None):
        q = R_to_q(np.asarray(R) @ self.HAND_TO_LINK8)
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = "panda_arm"
        r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = map(float, pos)
        r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y, r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = map(float, q)
        r.robot_state.joint_state.name = ARM
        r.robot_state.joint_state.position = list(map(float, seed if seed is not None else self.arm_q()))
        r.avoid_collisions = False
        r.timeout.sec = 2
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            log("IK FAIL", None if res is None else res.error_code.val, "at", np.round(pos, 4))
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[a] for a in ARM])

    def ik_local(self, pos, R, seed, iters=30, tol=5e-4):
        """Damped-least-squares IK from `seed` using a numerical Jacobian (FK service), so the
        result stays on the seed's solution branch (the IK service sometimes jumps branches)."""
        q = np.array(seed, float)
        pos = np.asarray(pos, float)

        def pose_err(qq):
            p, quat = self.fk(qq)
            x, y, z, w = quat
            Rc = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                           [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                           [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            Re = R @ Rc.T                     # rotation taking current to target
            ang = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))
            axis = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
            na = np.linalg.norm(axis)
            rot = np.zeros(3) if na < 1e-9 else axis / na * ang
            return np.concatenate([pos - p, rot])

        e = pose_err(q)
        hist = []
        for _ in range(iters):
            hist.append(round(float(np.linalg.norm(e)), 5))
            if np.linalg.norm(e[:3]) < tol and np.linalg.norm(e[3:]) < 4 * tol:
                return q
            J = np.zeros((6, 7))
            h = 1e-4
            for j in range(7):
                dq = np.zeros(7); dq[j] = h
                J[:, j] = (pose_err(q + dq) - e) / -h   # = d fk/dq (e = target - fk)
            step = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
            alpha = 1.0
            while alpha > 0.05:                         # backtracking so the error never grows
                qn = q + alpha * step
                en = pose_err(qn)
                if np.linalg.norm(en) < np.linalg.norm(e):
                    q, e = qn, en
                    break
                alpha *= 0.5
            else:
                break
        log(f"ik_local: not converged, pos err {np.linalg.norm(e[:3]):.4f} rot err {np.linalg.norm(e[3:]):.4f}"
            f" seed {np.round(seed,3)} target {np.round(pos,4)} hist {hist}")
        return None

    Q_LO = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    Q_HI = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

    def within_limits(self, q, margin=0.05):
        return bool(np.all(q > self.Q_LO + margin) and np.all(q < self.Q_HI - margin))

    def plan_line(self, q0, p1, R, step=0.015):
        """Joint waypoints for a straight hand path from fk(q0) to p1 at fixed R (on-branch)."""
        p0, _ = self.fk(q0)
        n = max(1, int(np.ceil(np.linalg.norm(np.asarray(p1) - p0) / step)))
        path, q = [], np.array(q0)
        for i in range(1, n + 1):
            q = self.ik_local(p0 + (np.asarray(p1) - p0) * i / n, R, q)
            if q is None or not self.within_limits(q):
                log(f"plan_line: fail at waypoint {i}/{n}" + ("" if q is None else f" (limits) {np.round(q,2)}"))
                return None
            path.append(q)
        return path

    def plan_rot(self, q0, G, R1, step_deg=10.0):
        """Joint waypoints rotating the hand about the fixed world point G (rigidly attached to the
        hand) from its pose at q0 to orientation R1."""
        from scipy.spatial.transform import Rotation as Rot
        p0, qu = self.fk(q0)
        R0 = Rot.from_quat(qu).as_matrix()
        off = R0.T @ (p0 - np.asarray(G))          # G->hand offset in hand frame
        dR = Rot.from_matrix(R1 @ R0.T)
        ang = np.degrees(dR.magnitude())
        n = max(1, int(np.ceil(ang / step_deg)))
        path, q = [], np.array(q0)
        for i in range(1, n + 1):
            Ri = (Rot.from_rotvec(dR.as_rotvec() * i / n)).as_matrix() @ R0
            pi_ = np.asarray(G) + Ri @ off
            q = self.ik_local(pi_, Ri, q)
            if q is None or not self.within_limits(q):
                log(f"plan_rot: fail at step {i}/{n}" + ("" if q is None else f" (limits) {np.round(q,2)}"))
                return None
            path.append(q)
        return path

    def move_path(self, path, secs):
        """Execute a precomputed joint path (from the current configuration) as one trajectory;
        time is stretched so no joint exceeds ~0.4 rad/s."""
        qs = [self.arm_q()] + list(path)
        travel = sum(np.abs(np.array(b) - np.array(a)).max() for a, b in zip(qs[:-1], qs[1:]))
        secs = max(secs, travel / 0.15)
        ok = self.move_joints(path[-1], secs, via=path[:-1])
        p, _ = self.fk()
        log(f"move_path ({len(path)} pts, {secs:.1f}s) ok={ok}; hand now {np.round(p, 4)}")
        return ok

    def move_joints(self, q, secs, via=None):
        """One trajectory; optional list of intermediate joint vectors `via` spread evenly."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        # densify (linear in joint space, <=0.05 rad per segment) and time points by arc length so
        # the controller sees a uniform-speed path; single big jumps otherwise abort with -5.
        raw = [self.arm_q()] + list(via or []) + [q]
        pts = []
        for a, b in zip(raw[:-1], raw[1:]):
            a, b = np.asarray(a, float), np.asarray(b, float)
            k = max(1, int(np.ceil(np.abs(b - a).max() / 0.05)))
            pts += [a + (b - a) * j / k for j in range(1, k + 1)]
        seg = np.array([np.abs(np.asarray(b) - np.asarray(a)).max() for a, b in zip(raw[:1] + pts[:-1], pts)])
        keep = seg > 1e-6
        pts = [p for p, k in zip(pts, keep) if k] or [np.asarray(q, float)]
        seg = seg[keep] if keep.any() else np.array([1.0])
        cum = np.cumsum(seg) / seg.sum()
        for i, p in enumerate(pts):
            t = max(0.2, secs * cum[i])
            pt = JointTrajectoryPoint(positions=list(map(float, p)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=120)
        gh = send.result()
        if gh is None or not gh.accepted:
            log("FJT goal rejected")
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=1800)
        if res.result() is None:
            log("FJT no result (timeout)")
        else:
            code = res.result().result.error_code
            if code != 0:
                log("FJT error_code", code)
        err = np.abs(self.arm_q() - np.array(q)).max()
        log(f"move done; max joint err {err:.4f} rad")
        return err < 0.02

    def ik_best(self, pos, R, seed, flip_ok=True):
        """IK; with flip_ok also try R rotated 180deg about hand z (symmetric gripper) and
        return the solution closest to seed in joint space."""
        cands = [R]
        if flip_ok:
            cands.append(R @ np.diag([-1.0, -1.0, 1.0]))
        best = None
        for Rc in cands:
            q = self.ik(pos, Rc, seed)
            if q is None:
                continue
            d = np.abs(q - seed).max()
            if best is None or d < best[1]:
                best = (q, d)
        return None if best is None else best[0]

    def move_pose(self, pos, R, secs, seed=None, via_poses=None, flip_ok=True, retries=1):
        """IK for pos/R (world, hand frame), execute. via_poses: list of (pos,R) intermediate."""
        seed = self.arm_q() if seed is None else seed
        via = []
        for vp, vR in (via_poses or []):
            qv = self.ik_best(vp, vR, seed, flip_ok)
            if qv is None:
                return False
            via.append(qv)
            seed = qv
        q = self.ik_best(pos, R, seed, flip_ok)
        if q is None:
            return False
        travel = np.abs(q - self.arm_q()).max()
        secs = max(secs, travel / 0.15)      # controller aborts (-5) on fast large joint moves
        log(f"joint travel {np.round(q - self.arm_q(), 2)} in {secs:.1f}s")
        ok = self.move_joints(q, secs, via=via)
        for _ in range(retries):
            if ok:
                break
            log("retrying same goal (slower)")
            ok = self.move_joints(q, secs * 1.5)
        p, _ = self.fk()
        log(f"hand now at {np.round(p, 4)} (target {np.round(pos, 4)}) d={np.linalg.norm(p - pos):.4f}")
        return ok

    def move_line(self, pos, R, secs, step=0.015, retries=1):
        """Straight-line Cartesian move (fixed orientation R) from the current hand pose:
        waypoints every `step` m, each IK-solved with the previous as seed, sent as one
        trajectory so the hand does not swing sideways like a joint-space interpolation."""
        p0, _ = self.fk()
        pos = np.asarray(pos, float)
        n = max(1, int(np.ceil(np.linalg.norm(pos - p0) / step)))
        seed = self.arm_q()
        via = []
        for i in range(1, n + 1):
            p = p0 + (pos - p0) * i / n
            q = self.ik(p, R, seed)
            if q is None or np.abs(q - seed).max() > 0.3:
                q = self.ik_local(p, R, seed)      # stay on the current branch
            if q is None:
                log(f"move_line: IK failed at waypoint {i}/{n} {np.round(p,4)}")
                return False
            if np.abs(q - seed).max() > 0.5:
                log(f"move_line: joint jump {np.abs(q - seed).max():.2f} at waypoint {i}; abort")
                return False
            via.append(q)
            seed = q
        q = via.pop()
        ok = self.move_joints(q, secs, via=via)
        for _ in range(retries):
            if ok:
                break
            log("move_line: retrying remaining path (slower)")
            ok = self.move_line(pos, R, secs, step, retries=0)
        p, _ = self.fk()
        log(f"hand now at {np.round(p, 4)} (target {np.round(pos, 4)}) d={np.linalg.norm(p - pos):.4f}")
        return ok

    def gripper(self, width, timeout=300):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result()
        f = self.fingers()
        log(f"gripper -> {width}: reached={None if r is None else r.result.reached_goal} "
            f"stalled={None if r is None else r.result.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f
