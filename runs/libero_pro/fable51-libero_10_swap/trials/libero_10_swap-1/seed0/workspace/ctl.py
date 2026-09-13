#!/usr/bin/env python3
"""Pick-and-place controller for this Panda. All coordinates are WORLD frame.

Subcommands (chain several in one invocation, separated by '--'):
  js                          print arm joints + finger positions
  fk                          FK of panda_hand / panda_link8 (world frame)
  grip open|close             gripper, then report finger gap
  tcp X Y Z [secs] [yaw_deg]  IK+trajectory so the TCP (fingertip point) lands
                              at world X Y Z, hand pointing down, fingers
                              closing along world Y (yaw 0) or rotated yaw_deg
  home                        go to the start configuration
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
ARM = FJT["joints"]
LIM = FJT["limits_rad"]
TCP_OFF = M["hand"]["tcp_offset_m"]
W2B = np.array([0.0, 0.0, 0.0])  # measured: FK/IK model frame == world on this machine (FK of home pose == TF world->panda_hand)
HOME = [0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854]


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


class Ctl:
    def __init__(self):
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

    def wait(self, fut, timeout):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            self.spin(0.1)
        return fut.result() if fut.done() else None

    # ---- sensing
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 30:
            self.spin(0.2)
        m = self._js["m"]
        d = dict(zip(m.name, m.position))
        return [d[j] for j in ARM], d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def cmd_js(self):
        q, f1, f2 = self.joints()
        log("arm", np.round(q, 4).tolist(), "fingers", round(f1, 4), round(f2, 4))

    def cmd_fk(self):
        q, _, _ = self.joints()
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand", "panda_link8"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = q
        res = self.wait(self.fk.call_async(req), 60)
        for name, ps in zip(res.fk_link_names, res.pose_stamped):
            p, o = ps.pose.position, ps.pose.orientation
            w = np.array([p.x, p.y, p.z]) + W2B
            log(f"FK {name}: frame='{ps.header.frame_id}' base=({p.x:.4f},{p.y:.4f},{p.z:.4f}) "
                f"world=({w[0]:.4f},{w[1]:.4f},{w[2]:.4f}) q=({o.x:.3f},{o.y:.3f},{o.z:.3f},{o.w:.3f})")

    # ---- acting
    def move(self, target, secs, tol=0.02, retries=2):
        self.fjt.wait_for_server(10)
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ARM
            pt = JointTrajectoryPoint(positions=[float(x) for x in target])
            pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
            goal.trajectory.points = [pt]
            gh = self.wait(self.fjt.send_goal_async(goal), 120)
            if gh is None:
                log("FJT goal not accepted in time")
                return False
            res = self.wait(gh.get_result_async(), 900)
            code = res.result.error_code if res else None
            q, _, _ = self.joints()
            err = float(np.max(np.abs(np.array(q) - np.array(target))))
            log(f"FJT done code={code} max_joint_err={err:.4f}")
            if err < tol:
                return True
            log("target not reached, resending")
        return False

    def cmd_iktest(self):
        """IK for the current hand pose must give back ~the current joints."""
        q, _, _ = self.joints()
        sol, code = self.solve_ik(np.array([-0.053, 0.0, 0.7776]) - W2B, (0.924, -0.383, -0.026, 0.011), q)
        log("iktest code", code, "sol", None if sol is None else np.round(sol, 3).tolist(), "cur", np.round(q, 3).tolist())

    def cmd_home(self, secs=4.0):
        return self.move(HOME, secs)

    def cmd_grip(self, what):
        width = GRIP["open_m"] if what == "open" else GRIP["closed_m"]
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        gh = self.wait(self.grip.send_goal_async(g), 120)
        res = self.wait(gh.get_result_async(), 600)
        r = res.result if res else None
        _, f1, f2 = self.joints()
        log(f"grip {what}: reached={getattr(r,'reached_goal',None)} stalled={getattr(r,'stalled',None)} "
            f"fingers={f1:.4f},{f2:.4f} gap={f1 - f2:.4f}")
        return f1, f2

    def solve_ik(self, pos_base, quat, seed):
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = [float(x) for x in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        res = self.wait(self.ik.call_async(req), 120)
        if res is None or res.error_code.val != 1:
            return None, (None if res is None else res.error_code.val)
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM], 1

    def cmd_tcp(self, x, y, z, secs=3.0, yaw_deg=0.0):
        """TCP at world (x,y,z), hand z pointing down, fingers along world Y rotated by yaw."""
        # hand orientation: R = Rz(yaw) * Rx(pi)  -> hand x ~ world x, hand z = -world z
        # measured: the IK tip link is panda_link8 (hand = link8 rotated -45deg about z),
        # so the link8 target rotation is R_hand * Rz(+45deg)
        from scipy.spatial.transform import Rotation as R
        R_hand = R.from_euler("z", yaw_deg, degrees=True) * R.from_euler("x", 180, degrees=True)
        qx, qy, qz, qw = (R_hand * R.from_euler("z", 45, degrees=True)).as_quat()
        # hand frame origin = TCP - TCP_OFF * (hand z axis in world) = TCP + TCP_OFF * world z
        hand_world = np.array([x, y, z + TCP_OFF])
        hand_base = hand_world - W2B
        seed, _, _ = self.joints()
        best = None
        for k in range(6):
            sol, code = self.solve_ik(hand_base, (qx, qy, qz, qw), seed)
            if sol is None:
                log(f"IK attempt {k} failed code={code}")
                continue
            d = np.abs(np.array(sol) - np.array(seed))
            d = np.minimum(d, 2 * np.pi - d)
            jump = float(d.max())
            ok_lim = all(lo <= v <= hi for v, (lo, hi) in zip(sol, LIM))
            log(f"IK attempt {k}: max jump {jump:.3f} limits_ok={ok_lim} sol={np.round(sol,3).tolist()}")
            if ok_lim and (best is None or jump < best[0]):
                best = (jump, sol)
            if ok_lim and jump < 1.0:
                break
        if best is None:
            log("IK: no solution; NO MOTION")
            return False
        jump, sol = best
        if jump > 2.0:
            log("IK: solution jumps too far from current config; refusing")
            return False
        ok = self.move(sol, secs)
        self.cmd_fk()
        return ok


def main():
    argv = sys.argv[1:]
    if not argv:
        raise SystemExit(__doc__)
    cmds, cur = [], []
    for a in argv:
        if a == "--":
            cmds.append(cur); cur = []
        else:
            cur.append(a)
    cmds.append(cur)
    c = Ctl()
    for cmd in cmds:
        if not cmd:
            continue
        log(">>", " ".join(cmd))
        name, args = cmd[0], cmd[1:]
        if name == "js":
            c.cmd_js()
        elif name == "fk":
            c.cmd_fk()
        elif name == "iktest":
            c.cmd_iktest()
        elif name == "grip":
            c.cmd_grip(args[0])
        elif name == "home":
            c.cmd_home(*map(float, args))
        elif name == "tcp":
            ok = c.cmd_tcp(*map(float, args))
            if not ok:
                log("ABORT: tcp move failed")
                break
        else:
            log("unknown command", name)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
