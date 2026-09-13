#!/usr/bin/env python3
"""Robot helper: joints, FK/IK (MoveIt), trajectories, gripper.

Import and use Rob(), or CLI:
  python3 rob.py js                         # arm joints + finger gap
  python3 rob.py fk                         # TCP pose in world
  python3 rob.py grip open|close
  python3 rob.py movej q1,...,q7 secs
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.zeros(3)  # MoveIt model frame here IS world (FK of panda_link0 -> (-0.66,0,0.912))


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def hand_R(approach, finger_axis):
    """Rotation matrix for the panda_hand frame: z = approach (palm->tips),
    y = finger opening axis. Both given in world; orthonormalised."""
    z = np.asarray(approach, float); z /= np.linalg.norm(z)
    y = np.asarray(finger_axis, float); y -= z * (y @ z); y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


class Rob:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        end = time.time() + 15
        while "m" not in self._js and time.time() < end:
            self.spin(0.2)
        m = self._js["m"]
        d = dict(zip(m.name, m.position))
        return np.array([d[j] for j in JOINTS]), d

    def finger_gap(self):
        _, d = self.joints()
        return d.get("panda_finger_joint1", float("nan")), d.get("panda_finger_joint2", float("nan"))

    def fk(self, q=None):
        """Return (tcp_world_xyz, R_hand_world)."""
        if q is None:
            q, _ = self.joints()
        if not self.fk_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        R = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        hand = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        tcp = hand + TCP_OFF * R[:, 2]
        return tcp, R

    # ---------- IK ----------
    def ik(self, tcp_world, R, seed=None, timeout=5.0):
        """IK for the hand so that the TCP lands at tcp_world with rotation R.
        Returns joint array or None."""
        tcp_world = np.asarray(tcp_world, float)
        hand_world = tcp_world - TCP_OFF * R[:, 2]
        hand_base = hand_world - BASE_IN_WORLD
        if seed is None:
            seed, _ = self.joints()
        if not self.ik_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"  # group tip is panda_link8 (45 deg off)
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_base)
        qx, qy, qz, qw = R_to_quat(R)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    def ik_near(self, tcp_world, R, seed=None, tries=6, max_dist=2.5):
        """IK, retrying with perturbed seeds; prefers the solution closest
        to the seed. Returns (q, dist) or (None, None)."""
        if seed is None:
            seed, _ = self.joints()
        best, bd = None, None
        rng = np.random.default_rng(0)
        for i in range(tries):
            s = seed if i == 0 else seed + rng.normal(0, 0.3, len(seed))
            s = np.clip(s, [l[0] for l in LIMITS], [l[1] for l in LIMITS])
            q = self.ik(tcp_world, R, seed=s)
            if q is None:
                continue
            d = float(np.abs(q - seed).max())
            if bd is None or d < bd:
                best, bd = q, d
            if d < 0.8:
                break
        if best is not None and bd > max_dist:
            print(f"[ik_near] warning: best solution is far from seed (max joint delta {bd:.2f})")
        return best, bd

    # ---------- action ----------
    def _send_traj(self, waypoints, times):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for q, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        r = rf.result()
        return r.result.error_code if r else None

    def movej(self, waypoints, times, settle_tol=0.01, settle_tries=3):
        """waypoints: list of joint arrays; times: cumulative seconds per point.
        The controller lags on big moves (sim clock only runs while a goal is
        active), so after the goal we re-send the final point as a hold until
        the joints settle within settle_tol."""
        code = self._send_traj(waypoints, times)
        tgt = np.asarray(waypoints[-1])
        q, _ = self.joints()
        err = float(np.abs(q - tgt).max())
        print(f"[movej] error_code={code} final max joint err={err:.4f}")
        for _ in range(settle_tries):
            if err <= settle_tol:
                break
            code = self._send_traj([tgt], [3.0])
            q, _ = self.joints()
            err = float(np.abs(q - tgt).max())
            print(f"[movej]   settle: error_code={code} max joint err={err:.4f}")
        return code, err

    def move_tcp(self, tcp_world, R, secs=3.0, seed=None, via=None):
        q, d = self.ik_near(tcp_world, R, seed=seed)
        if q is None:
            print(f"[move_tcp] IK FAILED for {np.round(tcp_world,3)}")
            return None
        print(f"[move_tcp] -> {np.round(tcp_world,3)} joint delta {d:.2f}")
        wps, ts = ([], [])
        if via is not None:
            wps.append(via); ts.append(secs * 0.5)
        wps.append(q); ts.append(secs)
        self.movej(wps, ts)
        tcp, Rn = self.fk()
        print(f"[move_tcp] reached tcp={np.round(tcp,4)} (target {np.round(tcp_world,4)}), pos err={np.linalg.norm(tcp-tcp_world):.4f}")
        return q

    def move_tcp_cl(self, tcp_world, R, secs=3.0, seed=None, iters=3, tol=0.004, check=None):
        """Closed-loop move: the controller sags under load in some
        configurations, so after each move we measure the TCP via FK and
        re-command with the residual added. `check(q0, q)` may veto a path."""
        tcp_world = np.asarray(tcp_world, float)
        p_cmd, R_cmd = tcp_world.copy(), R.copy()
        q = None
        for i in range(iters + 1):
            q0, _ = self.joints()
            q, d = self.ik_near(p_cmd, R_cmd, seed=seed if (seed is not None and q is None) else (q if q is not None else q0))
            if q is None:
                print(f"[move_cl] IK FAILED for {np.round(p_cmd,3)}")
                return None
            if check is not None and not check(q0, q):
                print("[move_cl] path check vetoed move")
                return None
            self.movej([q], [secs if i == 0 else 2.0])
            tcp, Rm = self.fk()
            dp = tcp_world - tcp
            ang = math.degrees(math.acos(np.clip((np.trace(R @ Rm.T) - 1) / 2, -1, 1)))
            print(f"[move_cl] iter {i}: tcp={np.round(tcp,4).tolist()} pos err={np.linalg.norm(dp):.4f} rot err={ang:.1f}deg")
            if np.linalg.norm(dp) <= tol and ang < 2.0:
                break
            p_cmd = p_cmd + dp
            R_cmd = (R @ Rm.T) @ R_cmd
        return q

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result().result
        gap = self.finger_gap()
        print(f"[gripper] cmd={width} reached={res.reached_goal} stalled={res.stalled} fingers={gap}")
        return gap


def main():
    r = Rob()
    cmd = sys.argv[1]
    if cmd == "js":
        q, d = r.joints()
        print("arm:", np.round(q, 4).tolist())
        print("fingers:", r.finger_gap())
    elif cmd == "fk":
        tcp, R = r.fk()
        print("tcp world:", np.round(tcp, 4).tolist())
        print("R:\n", np.round(R, 3))
    elif cmd == "grip":
        r.gripper(GRIP["open_m"] if sys.argv[2] == "open" else GRIP["closed_m"])
    elif cmd == "movej":
        q = [float(x) for x in sys.argv[2].split(",")]
        r.movej([q], [float(sys.argv[3])])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
