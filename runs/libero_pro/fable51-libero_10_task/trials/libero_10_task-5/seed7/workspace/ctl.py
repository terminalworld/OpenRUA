#!/usr/bin/env python3
"""Small controller: IK -> trajectory, gripper, joint-state, all in one node.

Usage:
  python3 ctl.py js                              # print arm + finger joints
  python3 ctl.py grip <width_m>                  # per-finger width
  python3 ctl.py goto <x> <y> <z> <yaw_deg> [secs]   # TCP (fingertip point) target in WORLD frame,
                                                 # hand pointing straight down, fingers along
                                                 # world Y rotated by yaw about Z
  python3 ctl.py joints p1,...,p7 [secs]
Poses are converted world -> panda_link0 using the fixed base offset.
"""
import sys
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])
TCP = float(M["hand"]["tcp_offset_m"])


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

    def js(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        js = self.js()
        return [js[j] for j in JOINTS]

    def solve_ik(self, pos_world, quat_xyzw, seed=None):
        if not self.ik.wait_for_service(timeout_sec=10):
            raise SystemExit("IK service unavailable")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        # verified via /compute_fk: the planner's model frame here IS world
        pb = np.asarray(pos_world)
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_xyzw)
        seed = seed if seed is not None else self.arm_q()
        s = JointState(); s.name = list(JOINTS); s.position = [float(v) for v in seed]
        req.ik_request.robot_state.joint_state = s
        req.ik_request.timeout.sec = 5
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise SystemExit("IK no answer")
        if res.error_code.val != 1:
            raise SystemExit(f"IK FAILED error_code={res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move(self, q, seconds):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"move done error_code={code} max_joint_err={err:.4f}")
        return code

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise SystemExit("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        js = self.js()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={js['panda_finger_joint1']:.4f},{js['panda_finger_joint2']:.4f}")

    def goto_tcp(self, x, y, z, yaw_deg, seconds=4.0):
        # hand pointing down: 180deg about X, then yaw about world Z
        R = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_euler("x", 180, degrees=True)
        # verified via /compute_fk: the IK tip link is panda_link8, which sits
        # 45deg yawed relative to panda_hand (same origin) -> request link8
        # orientation = Rz(-45) * desired hand orientation
        q = (Rot.from_euler("z", -45, degrees=True) * R).as_quat()  # xyzw
        # TCP is +TCP along hand Z (pointing down) -> hand origin is TCP - TCP*hand_z
        hand_z = R.as_matrix()[:, 2]
        hand_pos = np.array([x, y, z]) - TCP * hand_z
        qj = self.solve_ik(hand_pos, q)
        return self.move(qj, seconds)


if __name__ == "__main__" and sys.argv[1] not in ("gotop", "gotof"):
    c = Ctl()
    cmd = sys.argv[1]
    if cmd == "js":
        js = c.js()
        for k, v in js.items():
            print(f"{k}: {v:.4f}")
    elif cmd == "grip":
        c.gripper(float(sys.argv[2]))
    elif cmd == "goto":
        x, y, z, yaw = map(float, sys.argv[2:6])
        secs = float(sys.argv[6]) if len(sys.argv) > 6 else 4.0
        c.goto_tcp(x, y, z, yaw, secs)
    elif cmd == "joints":
        q = [float(v) for v in sys.argv[2].split(",")]
        secs = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
        c.move(q, secs)
    rclpy.shutdown()


def goto_tcp_pitch(c, x, y, z, yaw_deg, pitch_deg, seconds=4.0, seed=None):
    """Like goto_tcp but with an extra pitch about world Y applied last
    (pitch>0 tilts the hand's pointing direction toward -X)."""
    R = (Rot.from_euler("y", pitch_deg, degrees=True) * Rot.from_euler("z", yaw_deg, degrees=True)
         * Rot.from_euler("x", 180, degrees=True))
    # link8 = hand * Rz(+45) in the hand's local frame (same as the world
    # Rz(-45) pre-multiplication in goto_tcp when the hand points straight down)
    q = (R * Rot.from_euler("z", 45, degrees=True)).as_quat()
    hand_z = R.as_matrix()[:, 2]
    hand_pos = np.array([x, y, z]) - TCP * hand_z
    qj = c.solve_ik(hand_pos, q, seed=seed)
    print("ik q", np.round(qj, 2).tolist())
    return c.move(qj, seconds)


if __name__ == "__main__" and sys.argv[1] == "gotop":
    # gotop x y z yaw pitch [secs] [seed q1,...,q7]
    c = Ctl()
    x, y, z, yaw, pitch = map(float, sys.argv[2:7])
    secs = float(sys.argv[7]) if len(sys.argv) > 7 else 4.0
    seed = [float(v) for v in sys.argv[8].split(",")] if len(sys.argv) > 8 else None
    goto_tcp_pitch(c, x, y, z, yaw, pitch, secs, seed)
    rclpy.shutdown()


def goto_tcp_frame(c, tcp, f, d, seconds=4.0, seed=None):
    """General pose: f = finger (hand y) axis direction, d = hand pointing (hand z) direction, world frame."""
    f = np.asarray(f, float); d = np.asarray(d, float)
    d = d / np.linalg.norm(d); f = f - d * (f @ d); f = f / np.linalg.norm(f)
    x = np.cross(f, d)
    R = Rot.from_matrix(np.column_stack([x, f, d]))
    q = (R * Rot.from_euler("z", 45, degrees=True)).as_quat()
    hand_pos = np.asarray(tcp, float) - TCP * d
    qj = c.solve_ik(hand_pos, q, seed=seed)
    print("ik q", np.round(qj, 2).tolist())
    return c.move(qj, seconds)


if __name__ == "__main__" and sys.argv[1] == "gotof":
    # gotof x y z fx fy fz dx dy dz [secs] [seed]
    c = Ctl()
    v = list(map(float, sys.argv[2:11]))
    secs = float(sys.argv[11]) if len(sys.argv) > 11 else 4.0
    seed = [float(t) for t in sys.argv[12].split(",")] if len(sys.argv) > 12 else None
    goto_tcp_frame(c, v[0:3], v[3:6], v[6:9], secs, seed)
    rclpy.shutdown()
