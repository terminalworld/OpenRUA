#!/usr/bin/env python3
"""Motion helper for the Panda (world-frame poses, hand-frame semantics).

Subcommands:
  js                                  print arm joints + finger positions
  fk                                  print current panda_hand pose (world)
  ik  x y z  R(9 values row-major) [--tcp]   IK for a HAND pose (or TCP pose with --tcp); prints joints
  goto x y z R(9) secs [--tcp] [--dry]        IK then trajectory (single point)
  path secs_per_pt "x,y,z,R9[;...]" [--tcp]   multi-waypoint smooth trajectory
  joints j1,...,j7 secs
  grip open|close
Notes: IK tip is panda_link8 = hand rotated by -45deg about z; handled here.
"""
import sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034
FJT = "/panda_arm_controller/follow_joint_trajectory"
GRIP = "/franka_gripper/gripper_action"


def R_to_quat(R):
    # returns x,y,z,w
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s; x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return np.array([x, y, z, w])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


HAND_TO_L8 = Rz(np.pi / 4)  # R_link8 = R_hand @ Rz(+45deg)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("motion_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT)
        self.grip = ActionClient(self.node, GripperCommand, GRIP)

    def _on_js(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js = {}
        end = time.time() + 15
        while not all(j in self.js for j in JOINTS) and time.time() < end:
            self.spin()
        return [self.js[j] for j in JOINTS]

    def fingers(self):
        self.joints()
        return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")

    def fk(self, q=None):
        q = q if q is not None else self.joints()
        self.fk_cli.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        R = quat_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return np.array([p.position.x, p.position.y, p.position.z]), R

    def ik(self, pos, R_hand, seed=None, tcp=False, tries=3):
        """pos: world position of hand (or TCP if tcp=True); R_hand: 3x3 hand rotation (world)."""
        pos = np.array(pos, float)
        R_hand = np.array(R_hand, float).reshape(3, 3)
        if tcp:
            pos = pos - TCP * R_hand[:, 2]
        R8 = R_hand @ HAND_TO_L8
        q = R_to_quat(R8)
        seed = list(seed) if seed is not None else self.joints()
        self.ik_cli.wait_for_service(10)
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state.name = JOINTS
            req.ik_request.robot_state.joint_state.position = [float(s) for s in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                out = [sol[j] for j in JOINTS]
                # verify
                fp, fR = self.fk(out)
                err_p = np.linalg.norm(fp - pos)
                err_R = np.linalg.norm(fR - R_hand)
                if err_p < 2e-3 and err_R < 0.02:
                    return out
                print(f"  ik verify mismatch pos {err_p:.4f} rot {err_R:.4f}", file=sys.stderr)
            # perturb seed slightly and retry
            seed = [s + np.random.uniform(-0.15, 0.15) for s in seed]
        return None

    def move_joints(self, waypoints, secs_per_pt, first_secs=None):
        """waypoints: list of 7-lists. Returns error_code."""
        if not self.fjt.wait_for_server(timeout_sec=15):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        t = 0.0
        for i, w in enumerate(waypoints):
            t += (first_secs if (i == 0 and first_secs) else secs_per_pt)
            pt = JointTrajectoryPoint(positions=[float(v) for v in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise SystemExit("goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=1500)
        if rf.result() is None:
            print("no result (timeout)"); return -99
        code = rf.result().result.error_code
        q = self.joints()
        err = max(abs(a - b) for a, b in zip(q, waypoints[-1]))
        print(f"traj error_code={code} final_joint_err={err:.4f}")
        return code

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=15):
            raise SystemExit("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        f = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}")


def parse_R(vals):
    return np.array([float(v) for v in vals]).reshape(3, 3)


def main():
    a = sys.argv[1:]
    flags = [x for x in a if x.startswith("--")]
    a = [x for x in a if not x.startswith("--")]
    cmd = a[0]
    r = Robot()
    if cmd == "js":
        print("arm:", [round(v, 4) for v in r.joints()], "fingers:", r.fingers())
    elif cmd == "fk":
        p, R = r.fk(); print("hand pos", p.round(4)); print("R", R.round(3)); print("tcp", (p + TCP * R[:, 2]).round(4))
    elif cmd == "ik":
        pos = [float(v) for v in a[1:4]]; R = parse_R(a[4:13])
        sol = r.ik(pos, R, tcp="--tcp" in flags)
        print("IK FAILED" if sol is None else ",".join(f"{v:.5f}" for v in sol))
    elif cmd == "goto":
        pos = [float(v) for v in a[1:4]]; R = parse_R(a[4:13]); secs = float(a[13])
        seed = None
        for f in flags:
            if f.startswith("--seed="):
                seed = [float(v) for v in f[7:].split(",")]
        sol = r.ik(pos, R, seed=seed, tcp="--tcp" in flags)
        if sol is None:
            print("IK FAILED"); sys.exit(1)
        cur = r.joints()
        print("target joints", [round(v, 4) for v in sol], "max_dq", round(max(abs(x - y) for x, y in zip(sol, cur)), 3))
        if "--dry" not in flags:
            r.move_joints([sol], secs)
            p, R2 = r.fk(); print("hand now", p.round(4), "tcp", (p + TCP * R2[:, 2]).round(4))
    elif cmd == "path":
        secs = float(a[1]); pts = a[2].split(";")
        seed = r.joints(); sols = []
        for s in pts:
            v = [float(x) for x in s.split(",")]
            sol = r.ik(v[:3], parse_R(v[3:12]), seed=seed, tcp="--tcp" in flags)
            if sol is None:
                print("IK FAILED at", s); sys.exit(1)
            sols.append(sol); seed = sol
        for s in sols: print("wp", [round(v, 3) for v in s])
        if "--dry" not in flags:
            r.move_joints(sols, secs)
            p, R2 = r.fk(); print("hand now", p.round(4), "tcp", (p + TCP * R2[:, 2]).round(4))
    elif cmd == "joints":
        q = [float(v) for v in a[1].split(",")]; r.move_joints([q], float(a[2]))
    elif cmd == "grip":
        r.gripper(0.04 if a[1] == "open" else 0.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
