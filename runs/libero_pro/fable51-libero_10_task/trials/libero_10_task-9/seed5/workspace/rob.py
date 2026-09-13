#!/usr/bin/env python3
"""Robot helper library + CLI (Panda via MoveIt IK/FK + FJT + gripper).

CLI:
  python3 rob.py js                 # joint state dict
  python3 rob.py fk                 # hand pose in world (xyz, quat, rpy) + tcp
  python3 rob.py ik x y z qx qy qz qw   # print IK solution (world frame pose of hand)
  python3 rob.py go x y z qx qy qz qw [secs] [--tcp]   # IK + move
  python3 rob.py joints j1,..,j7 [secs]
  python3 rob.py grip open|close
  python3 rob.py wrench
"""
import math
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE_T = np.array([0.0, 0.0, 0.0])  # MoveIt FK/IK already report in WORLD (verified: fk == camera TF)
TCP_OFF = 0.1034
FJT = "/panda_arm_controller/follow_joint_trajectory"
GRIP = "/franka_gripper/gripper_action"
LIMITS = [(-2.9, 2.9), (-1.76, 1.76), (-2.9, 2.9), (-3.07, -0.07), (-2.9, 2.9),
          (-0.02, 3.75), (-2.9, 2.9)]


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()  # x y z w


def rpy_quat(r, p, y):
    from scipy.spatial.transform import Rotation
    return Rotation.from_euler("xyz", [r, p, y]).as_quat()


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT)
        self.grip = ActionClient(self.node, GripperCommand, GRIP)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr.pop("m", None)
        for _ in range(50):
            self.spin(0.2)
            if "m" in self._wr:
                break
        if "m" not in self._wr:
            return None
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z,
                         w.torque.x, w.torque.y, w.torque.z])

    def fk(self, q=None, link="panda_hand"):
        """Hand pose in WORLD frame: (pos[3], quat[4])."""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_T
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        return pos + TCP_OFF * quat_R(quat)[:, 2], quat

    # ---------- IK ----------
    def ik(self, pos_w, quat, seed=None, tcp=False, timeout=1.0, attempts=1):
        """IK for hand at world pose. If tcp, pos_w is the fingertip point."""
        pos_w = np.asarray(pos_w, float)
        quat = np.asarray(quat, float)
        if tcp:
            pos_w = pos_w - TCP_OFF * quat_R(quat)[:, 2]
        pos_b = pos_w - BASE_T
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(
            map(float, seed if seed is not None else self.arm_q()))
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout),
                                          nanosec=int((timeout % 1) * 1e9))
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in ARM]

    # ---------- motion ----------
    def move_joints(self, q, secs=3.0, via=None):
        """Send trajectory to q (list of 7) over secs; via = list of
        (q, t) intermediate points. Returns error_code."""
        for i, (v, (lo, hi)) in enumerate(zip(q, LIMITS)):
            if not (lo <= v <= hi):
                raise ValueError(f"joint{i+1}={v:.3f} outside [{lo},{hi}]")
        cur = np.array(self.arm_q())
        dq = np.abs(np.array(q) - cur)
        need = max(dq[6] / 0.15, dq.max() / 1.0) + 0.5  # j7 is capped ~0.18 rad/s
        if secs < need and not via:
            print(f"[move] stretching duration {secs:.1f}->{need:.1f}s", flush=True)
            secs = need
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        for vq, vt in (via or []):
            pt = JointTrajectoryPoint(positions=list(map(float, vq)))
            pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=list(map(float, q)))
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        if not gh.accepted:
            raise RuntimeError("FJT goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"[move] code={code} max_joint_err={err:.4f}", flush=True)
        return code

    def move_pose(self, pos_w, quat, secs=3.0, tcp=False, seed=None):
        q = self.ik(pos_w, quat, seed=seed, tcp=tcp)
        if q is None:
            print(f"[move_pose] IK FAILED for {np.round(pos_w,3)}", flush=True)
            return None
        return self.move_joints(q, secs)

    def gripper(self, width):
        """width = per-finger position (0.04 open, 0.0 closed)."""
        self.grip.wait_for_server(timeout_sec=10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        send = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"[grip] reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}", flush=True)
        return f


def main():
    a = sys.argv[1:]
    r = Robot()
    cmd = a[0]
    if cmd == "js":
        print(r.joints())
    elif cmd == "fk":
        pos, q = r.fk()
        from scipy.spatial.transform import Rotation
        print("hand pos", np.round(pos, 4), "quat", np.round(q, 4),
              "rpy", np.round(Rotation.from_quat(q).as_euler("xyz"), 3))
        print("tcp", np.round(r.tcp()[0], 4))
        print("hand R (cols = hand x,y,z in world):\n", np.round(quat_R(q), 3))
    elif cmd == "ik":
        v = list(map(float, a[1:8]))
        print(r.ik(v[:3], v[3:], tcp="--tcp" in a))
    elif cmd == "go":
        v = list(map(float, [x for x in a[1:] if not x.startswith("--")]))
        secs = v[7] if len(v) > 7 else 3.0
        r.move_pose(v[:3], v[3:7], secs, tcp="--tcp" in a)
        print("tcp now", np.round(r.tcp()[0], 4))
    elif cmd == "joints":
        q = list(map(float, a[1].split(",")))
        secs = float(a[2]) if len(a) > 2 else 3.0
        r.move_joints(q, secs)
    elif cmd == "grip":
        r.gripper(0.04 if a[1] == "open" else 0.0)
    elif cmd == "wrench":
        print(r.wrench())
    rclpy.shutdown()


if __name__ == "__main__":
    main()


def links_fk(r, q, links=("panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7", "panda_hand")):
    """World positions of several links for a joint vector q."""
    r.fk_cli.wait_for_service(timeout_sec=10)
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = list(links)
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = list(map(float, q))
    fut = r.fk_cli.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    out = {}
    for n, ps in zip(res.fk_link_names, res.pose_stamped):
        p = ps.pose.position
        out[n] = np.array([p.x, p.y, p.z])
    return out


def hand_quat(z_dir, x_dir):
    """Quaternion for hand with approach axis z_dir and hand-x along x_dir (world)."""
    z = np.asarray(z_dir, float); z /= np.linalg.norm(z)
    x = np.asarray(x_dir, float); x = x - z * (x @ z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return R_quat(np.stack([x, y, z], 1))


NOMINAL = np.array([0.0, 0.4, 0.0, -1.9, 0.0, 2.3, 0.8])


def best_ik(r, pos, quat, tcp=True, n=30, seed=None, prefer=None, avoid=None, rng=None):
    """Try IK from several seeds; return the solution closest to `prefer`
    (joint vector) / NOMINAL, with joint-limit margin, optionally
    rejecting solutions where a link enters an `avoid(links)->bool` test."""
    rng = rng or np.random.default_rng(1)
    seeds = [seed] if seed is not None else []
    seeds += [r.arm_q(), list(NOMINAL)]
    seeds += [[rng.uniform(lo, hi) for lo, hi in LIMITS] for _ in range(n)]
    best, best_c = None, 1e9
    ref = np.array(prefer if prefer is not None else NOMINAL)
    for sd in seeds:
        s = r.ik(pos, quat, seed=sd, tcp=tcp, timeout=0.3)
        if s is None:
            continue
        if any(v < lo + 0.08 or v > hi - 0.08 for v, (lo, hi) in zip(s, LIMITS)):
            continue
        if avoid is not None:
            L = links_fk(r, s)
            if avoid(L):
                continue
        c = np.linalg.norm((np.array(s) - ref) * np.array([1, 1, 1, 1, 0.5, 1, 0.3]))
        if c < best_c:
            best, best_c = s, c
    return best


def tilt_dir(yaw_deg, tilt_deg):
    """Approach direction pointing at horizontal angle yaw (deg, from +x)
    and tilt_deg below horizontal."""
    a, b = np.deg2rad(yaw_deg), np.deg2rad(tilt_deg)
    return np.array([np.cos(a) * np.cos(b), np.sin(a) * np.cos(b), -np.sin(b)])


def clear_of_door(L, margin=0.06):
    """False if wrist/hand links come near the open door / microwave box or table."""
    boxes = [(-0.20, -0.14, 0.0, 0.30, 0.0, 1.12),   # open door
             (-0.18, 0.17, 0.27, 0.48, 0.0, 1.12)]   # microwave body
    for k in ('panda_link5', 'panda_link6', 'panda_link7', 'panda_hand'):
        p = L[k]
        if p[2] < 0.9 + margin:
            return False
        for (x0, x1, y0, y1, z0, z1) in boxes:
            dx = max(x0 - p[0], 0, p[0] - x1); dy = max(y0 - p[1], 0, p[1] - y1)
            dz = max(z0 - p[2], 0, p[2] - z1)
            if np.sqrt(dx * dx + dy * dy + dz * dz) < margin:
                return False
    return True
