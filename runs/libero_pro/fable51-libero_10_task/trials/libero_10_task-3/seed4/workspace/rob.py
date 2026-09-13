#!/usr/bin/env python3
"""Robot helper: world-frame IK moves, FK, gripper, joint state.

Usage:
  python3 rob.py js                         # joint state dict
  python3 rob.py fk                         # hand pose in world (via /compute_fk)
  python3 rob.py move x y z qx qy qz qw [--t 3] [--tcp] [--via]  # IK -> trajectory
  python3 rob.py joints j1,...,j7 [--t 3]   # raw joint target
  python3 rob.py grip open|close|<width_m>
World->base offset from TF: base panda_link0 at world (-0.66, 0, 0.912), same orientation.
"""
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
# verified: /compute_fk and /compute_ik on this machine work in the WORLD
# frame (FK of panda_link0 returns (-0.66, 0, 0.912)); no offset needed
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])
TCP_OFF = 0.1034
LIMITS = [(-2.9, 2.9), (-1.76, 1.76), (-2.9, 2.9), (-3.07, -0.07), (-2.9, 2.9), (-0.02, 3.75), (-2.9, 2.9)]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    # returns x,y,z,w
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))
        self._js_msg = msg

    def spin(self, t):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joint_state(self, fresh=True):
        if fresh:
            self._js = {}
        end = time.time() + 15
        while not self._js and time.time() < end:
            self.spin(0.2)
        if not self._js:
            raise RuntimeError("no /joint_states")
        return dict(self._js)

    def arm_q(self):
        js = self.joint_state()
        return np.array([js[j] for j in ARM])

    def finger(self):
        js = self.joint_state()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    def _seed(self, q=None):
        q = self.arm_q() if q is None else q
        s = JointState()
        s.name = list(ARM)
        s.position = [float(v) for v in q]
        return s

    def fk_world(self, q=None, link="panda_hand"):
        if not self.fk.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def ik_world(self, pos, quat, seed=None, tcp=False, timeout=5.0, avoid=False):
        pos = np.array(pos, float)
        if tcp:
            R = quat_to_R(*quat)
            pos = pos - TCP_OFF * R[:, 2]
        if not self.ik.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.avoid_collisions = avoid
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        b = pos - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=90)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    def traj(self, points, times):
        """points: list of 7-vectors; times: list of seconds (cumulative)."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(points, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("FJT goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        r = rf.result()
        if r is None:
            raise RuntimeError("FJT no result (timeout)")
        return r.result.error_code

    def move_q(self, q, t=3.0):
        q = np.array(q, float)
        for i, (lo, hi) in enumerate(LIMITS):
            if not (lo - 1e-6 <= q[i] <= hi + 1e-6):
                raise RuntimeError(f"joint {i+1} target {q[i]:.3f} outside [{lo},{hi}]")
        code = self.traj([q], [t])
        qn = self.arm_q()
        err = np.abs(qn - q).max()
        return code, err

    def move_pose(self, pos, quat, t=3.0, tcp=False, seed=None, max_jump=None):
        q = self.ik_world(pos, quat, seed=seed, tcp=tcp)
        if q is None:
            raise RuntimeError(f"IK failed for {pos} {quat}")
        q0 = self.arm_q()
        jump = np.abs(q - q0).max()
        if max_jump is not None and jump > max_jump:
            raise RuntimeError(f"IK solution jumps {jump:.2f} rad (max {max_jump}); q={q}")
        code, err = self.move_q(q, t)
        return q, code, err, jump

    def gripper(self, width, timeout=300):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result()
        if r is None:
            return None
        return r.result.position, r.result.reached_goal, r.result.stalled


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    t = 3.0
    if "--t" in a:
        i = a.index("--t"); t = float(a[i + 1]); del a[i:i + 2]
    tcp = "--tcp" in a
    a = [x for x in a if not x.startswith("--")]
    r = Robot()
    cmd = a[0]
    if cmd == "js":
        js = r.joint_state()
        for k, v in js.items():
            print(f"{k}: {v:.4f}")
    elif cmd == "fk":
        pos, quat = r.fk_world()
        R = quat_to_R(*quat)
        print("hand world pos", np.round(pos, 4), "quat xyzw", np.round(quat, 4))
        print("tcp world pos", np.round(pos + TCP_OFF * R[:, 2], 4))
        print("hand axes (cols x,y,z):\n", np.round(R, 3))
    elif cmd == "move":
        vals = list(map(float, a[1:8]))
        q, code, err, jump = r.move_pose(vals[:3], vals[3:7], t=t, tcp=tcp)
        print(f"q={np.round(q,4).tolist()} code={code} final_err={err:.4f} jump={jump:.2f}")
        pos, quat = r.fk_world()
        print("hand now", np.round(pos, 4), np.round(quat, 4))
    elif cmd == "ik":
        vals = list(map(float, a[1:8]))
        q = r.ik_world(vals[:3], vals[3:7], tcp=tcp)
        print("IK:", None if q is None else np.round(q, 4).tolist())
    elif cmd == "joints":
        q = list(map(float, a[1].split(",")))
        code, err = r.move_q(q, t)
        print(f"code={code} final_err={err:.4f}")
    elif cmd == "grip":
        w = {"open": 0.04, "close": 0.0}.get(a[1], None)
        w = float(a[1]) if w is None else w
        print("gripper result", r.gripper(w))
        print("fingers", r.finger())
    else:
        raise SystemExit(__doc__)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


def tilt_quat(th_deg):
    """Hand pointing down, pitched th_deg toward -x (toward the robot); fingers close along world y."""
    th = np.radians(th_deg)
    z = np.array([-np.sin(th), 0, -np.cos(th)])
    y = np.array([0, -1, 0])
    x = np.cross(y, z)
    return R_to_quat(np.column_stack([x, y, z]))


def best_ik(r, pos, quat, tcp=True, extra_seeds=()):
    """Try several seeds; prefer solutions with small |j1|,|j3|,|j5| and near current q."""
    q0 = r.arm_q()
    seeds = [q0] + list(extra_seeds) + [np.array([0, a, 0, b, 0, c, 0.785])
                                          for a in (-0.3, 0.0, 0.3, 0.6)
                                          for b in (-2.8, -2.4, -2.0)
                                          for c in (1.8, 2.3, 2.8)]
    best, bs = None, None
    for s in seeds:
        sol = r.ik_world(pos, quat, seed=s, tcp=tcp, timeout=0.3)
        if sol is None:
            continue
        score = np.abs(sol - q0).max() + 0.5 * (abs(sol[0]) + abs(sol[2]) + abs(sol[4]))
        if best is None or score < bs:
            best, bs = sol, score
    return best


def go(r, pos, quat, t=3.0, tcp=True, label=""):
    sol = best_ik(r, pos, quat, tcp=tcp)
    if sol is None:
        raise RuntimeError(f"no IK for {label} {pos}")
    code, err = r.move_q(sol, t)
    if err > 0.03:
        code, err = r.move_q(sol, t)
    p, qq = r.fk_world()
    tcp_p = p + TCP_OFF * quat_to_R(*qq)[:, 2]
    print(f"{label}: q={np.round(sol,3).tolist()} code={code} err={err:.4f} tcp={np.round(tcp_p,3).tolist()}")
    return sol
