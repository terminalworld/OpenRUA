#!/usr/bin/env python3
"""Small control library for this Panda: state, FK/IK, trajectories, gripper.

CLI:
  python3 ctl.py state                       # joints + hand/TCP pose (world)
  python3 ctl.py movej j1,...,j7 [sec]       # joint move
  python3 ctl.py move x y z rx ry rz [sec] [--tcp]  # world pose, rotvec (deg) about world axes
  python3 ctl.py grip open|close
"""
import sys, math, time
import numpy as np
import rclpy, yaml
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # measured: /compute_fk and /compute_ik work in WORLD coords here
TCP = float(M["hand"]["tcp_offset_m"])


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def hand_pose(self, q=None):
        """FK: hand pose in WORLD -> (pos[3], Rot)."""
        if q is None:
            q = self.arm_q()
        self.fk.wait_for_service(5)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if r is None else r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = Rot.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, R

    def tcp_pose(self, q=None):
        pos, R = self.hand_pose(q)
        return pos + TCP * R.as_matrix()[:, 2], R

    # ---------------- IK
    def solve_ik(self, pos_world, R, seed=None, tcp=False, tries=4):
        """pos_world: hand (or TCP if tcp=True) position in world; R: scipy Rotation."""
        pos_world = np.asarray(pos_world, float)
        if tcp:
            pos_world = pos_world - TCP * R.as_matrix()[:, 2]
        p_base = pos_world - BASE_IN_WORLD
        q = R.as_quat()
        self.ik.wait_for_service(5)
        seed0 = list(seed if seed is not None else self.arm_q())
        seed = list(seed0)
        last = None
        sols = []
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            pp = req.ik_request.pose_stamped.pose
            pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
            pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = [float(s) for s in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 1
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                sols.append([sol[j] for j in JOINTS])
                if len(sols) >= 2 and k >= 1:
                    break
            else:
                last = None if r is None else r.error_code.val
            seed = [s + np.random.uniform(-0.2, 0.2) for s in seed0]
        if sols:
            d = [np.abs(np.array(s_) - np.array(seed0)).sum() for s_ in sols]
            return sols[int(np.argmin(d))]
        raise RuntimeError(f"IK failed (code {last}) for world pos {pos_world.round(3)}")

    # ---------------- motion
    def movej(self, targets, secs):
        """targets: list of joint vectors (waypoints); secs: list of time_from_start or single total."""
        if not isinstance(targets[0], (list, tuple, np.ndarray)):
            targets = [targets]
        if not isinstance(secs, (list, tuple)):
            n = len(targets)
            secs = [secs * (i + 1) / n for i in range(n)]
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for tq, ts in zip(targets, secs):
            pt = JointTrajectoryPoint(positions=[float(v) for v in tq])
            pt.time_from_start = Duration(sec=int(ts), nanosec=int((ts % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("FJT goal not accepted")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=900)
        code = res.result().result.error_code if res.result() else None
        q = np.array(self.arm_q())
        err = np.abs(q - np.array(targets[-1])).max()
        print(f"  movej: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move(self, pos_world, R, secs=3.0, tcp=False, seed=None):
        q = self.solve_ik(pos_world, R, seed=seed, tcp=tcp)
        code, err = self.movej(q, secs)
        p, Rr = self.tcp_pose() if tcp else self.hand_pose()
        print(f"  reached {'tcp' if tcp else 'hand'} pos={p.round(4)} target={np.round(pos_world,4)} "
              f"rot_err_deg={np.degrees((Rr.inv()*R).magnitude()):.2f}")
        return q

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper: reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f


def R_from_axes(zaxis, yaxis):
    """Hand rotation with hand Z (approach) and hand Y (finger opening) given in world."""
    z = np.asarray(zaxis, float); z /= np.linalg.norm(z)
    y = np.asarray(yaxis, float); y = y - z * (y @ z); y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return Rot.from_matrix(np.column_stack([x, y, z]))


def fmt(pos, R):
    return f"pos={np.round(pos,4)} quat(xyzw)={np.round(R.as_quat(),4)} Z={np.round(R.as_matrix()[:,2],3)} Y={np.round(R.as_matrix()[:,1],3)}"


if __name__ == "__main__":
    a = sys.argv[1:]
    c = Ctl()
    if not a or a[0] == "state":
        j = c.joints()
        print("joints:", {k: round(v, 4) for k, v in j.items()})
        print("hand :", fmt(*c.hand_pose()))
        print("tcp  :", fmt(*c.tcp_pose()))
    elif a[0] == "movej":
        q = [float(x) for x in a[1].split(",")]
        c.movej(q, float(a[2]) if len(a) > 2 else 3.0)
    elif a[0] == "move":
        x, y, z, rx, ry, rz = map(float, a[1:7])
        secs = float(a[7]) if len(a) > 7 and not a[7].startswith("--") else 3.0
        R = Rot.from_rotvec(np.radians([rx, ry, rz]))
        c.move([x, y, z], R, secs, tcp="--tcp" in a)
    elif a[0] == "grip":
        c.gripper(GRIP["open_m"] if a[1] == "open" else GRIP["closed_m"])
    rclpy.shutdown()


def move_checked(c, pos, R, secs=3.0, tcp=True, tol_m=0.01, tol_deg=3.0, retries=1):
    """IK -> verify FK of solution -> execute -> verify reached. Raises on mismatch."""
    for attempt in range(retries + 1):
        q = c.solve_ik(pos, R, tcp=tcp)
        p_fk, R_fk = (c.tcp_pose(q) if tcp else c.hand_pose(q))
        perr = np.linalg.norm(p_fk - np.asarray(pos)); rerr = np.degrees((R_fk.inv() * R).magnitude())
        if perr > tol_m or rerr > tol_deg:
            raise RuntimeError(f"IK solution FK mismatch: pos err {perr:.4f} rot err {rerr:.2f}")
        code, jerr = c.movej(q, secs)
        p, Rr = (c.tcp_pose() if tcp else c.hand_pose())
        perr = np.linalg.norm(p - np.asarray(pos)); rerr = np.degrees((Rr.inv() * R).magnitude())
        print(f"  -> at {p.round(4)} (target {np.round(pos,4)}) pos_err={perr:.4f} rot_err={rerr:.2f}deg code={code}")
        if perr <= tol_m and rerr <= tol_deg:
            return q
        if attempt < retries:
            print("  re-sending same goal (tracking lag)")
            c.movej(q, secs)
            p, Rr = (c.tcp_pose() if tcp else c.hand_pose())
            perr = np.linalg.norm(p - np.asarray(pos)); rerr = np.degrees((Rr.inv() * R).magnitude())
            print(f"  -> at {p.round(4)} pos_err={perr:.4f} rot_err={rerr:.2f}deg")
            if perr <= tol_m and rerr <= tol_deg:
                return q
    raise RuntimeError("motion did not converge to target")


LINKS = ["panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7", "panda_hand", "panda_leftfinger", "panda_rightfinger"]


def fk_links(c, q, links=LINKS):
    """World positions of several links for joint vector q -> dict name->pos."""
    c.fk.wait_for_service(5)
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = list(links)
    req.robot_state.joint_state.name = list(JOINTS)
    req.robot_state.joint_state.position = [float(v) for v in q]
    fut = c.fk.call_async(req)
    rclpy.spin_until_future_complete(c.node, fut, timeout_sec=30)
    r = fut.result()
    out = {}
    for name, ps in zip(r.fk_link_names, r.pose_stamped):
        p = ps.pose.position
        out[name] = np.array([p.x, p.y, p.z])
    return out


def path_check(c, q_from, q_to, n=15, z_floor=0.93, verbose=True):
    """Interpolate in joint space and report min z of links + TCP; returns (ok, samples)."""
    q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
    worst = 9.0; samples = []
    for i in range(n + 1):
        q = q_from + (q_to - q_from) * i / n
        L = fk_links(c, q)
        tcp, _ = c.tcp_pose(q)
        L["tcp"] = tcp
        zmin_name = min(L, key=lambda k: L[k][2])
        samples.append((i, L))
        if L[zmin_name][2] < worst:
            worst = L[zmin_name][2]
        if verbose:
            print(f"   step {i:2d}: hand={L['panda_hand'].round(3)} tcp={tcp.round(3)} lowest={zmin_name}@{L[zmin_name][2]:.3f}")
    ok = worst >= z_floor
    print(f"  path lowest point z={worst:.3f} -> {'OK' if ok else 'TOO LOW'}")
    return ok, samples
