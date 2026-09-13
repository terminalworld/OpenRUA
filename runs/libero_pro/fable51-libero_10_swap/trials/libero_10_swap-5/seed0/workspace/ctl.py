#!/usr/bin/env python3
"""Reusable arm controller: IK -> trajectory, FK check, gripper, joint state.
World<->base offset from TF (world -> panda_link0 = (-0.75, 0, 0.912)).
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = M["hand"]["tcp_offset_m"]
# Verified empirically: /compute_fk and /compute_ik on this machine speak
# WORLD coordinates (FK header frame_id == "world", values match world),
# so no base offset is applied.
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])


def topdown_quat(theta_deg):
    """Hand z down; hand x axis at theta (deg) from world +x. Returns xyzw."""
    th = np.radians(theta_deg)
    xh = np.array([np.cos(th), np.sin(th), 0.0])
    zh = np.array([0.0, 0.0, -1.0])
    yh = np.cross(zh, xh)
    R = np.column_stack([xh, yh, zh])
    return Rot.from_matrix(R).as_quat()  # xyzw


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)

    def spin(self, fut, timeout):
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 20:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def _seed(self, q):
        js = JointState()
        js.name = list(JOINTS)
        js.position = [float(v) for v in q]
        return js

    def fk_hand(self, q=None):
        """Hand pose in WORLD: (xyz, quat xyzw)."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        res = self.spin(self.fk.call_async(req), 30)
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return xyz, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def tcp_world(self, q=None):
        xyz, quat = self.fk_hand(q)
        R = Rot.from_quat(quat).as_matrix()
        return xyz + TCP * R[:, 2], quat

    def solve_ik(self, tcp_world_xyz, quat_xyzw, seed=None, tries=5):
        """IK for a TCP target in world frame. Returns joint list or None."""
        R = Rot.from_quat(quat_xyzw).as_matrix()
        hand_world = np.asarray(tcp_world_xyz) - TCP * R[:, 2]
        hand_base = hand_world - BASE_IN_WORLD
        seed = self.arm_q() if seed is None else seed
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            # group tip is panda_link8 (45 deg off panda_hand); ask for the hand
            req.ik_request.ik_link_name = "panda_hand"
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, hand_base)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_xyzw)
            s = np.array(seed, dtype=float)
            if k > 0:
                s = s + np.random.uniform(-0.3, 0.3, size=7)
                s = np.clip(s, [l[0] for l in LIMITS], [l[1] for l in LIMITS])
            req.ik_request.robot_state.joint_state = self._seed(s)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            res = self.spin(self.ik.call_async(req), 60)
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                q = [sol[j] for j in JOINTS]
                # sanity: FK back to the target
                got, gq = self.tcp_world(q)
                err = np.linalg.norm(got - np.asarray(tcp_world_xyz))
                ang = (Rot.from_quat(gq) * Rot.from_quat(quat_xyzw).inv()).magnitude()
                if err < 0.005 and ang < 0.02:
                    return q
                print(f"  ik try {k}: FK mismatch pos {err:.4f} ang {np.degrees(ang):.1f}deg")
            else:
                print(f"  ik try {k}: fail code={None if res is None else res.error_code.val}")
        return None

    def move_q(self, q, seconds=3.0, tol=0.02):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        gh = self.spin(self.fjt.send_goal_async(goal), 60)
        res = self.spin(gh.get_result_async(), 600)
        code = res.result.error_code if res else None
        now = np.array(self.arm_q())
        err = np.abs(now - np.array(q)).max()
        print(f"  move: code={code} max_joint_err={err:.4f}")
        return err < tol

    def move_tcp(self, xyz, quat, seconds=3.0, seed=None):
        q = self.solve_ik(xyz, quat, seed=seed)
        if q is None:
            print("  IK FAILED for", np.round(xyz, 3))
            return False
        ok = self.move_q(q, seconds)
        for _ in range(2):
            if ok:
                break
            print("  resending goal (controller lag)")
            ok = self.move_q(q, max(2.0, seconds / 2))
        got, _ = self.tcp_world()
        print(f"  tcp now {np.round(got, 4)} target {np.round(xyz, 4)} err {np.linalg.norm(got - xyz):.4f}")
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        gh = self.spin(self.grip.send_goal_async(goal), 60)
        res = self.spin(gh.get_result_async(), 300)
        f = self.fingers()
        print(f"  gripper -> {width}: reached={res.result.reached_goal} stalled={res.result.stalled} fingers={np.round(f, 4)}")
        return f
