"""Reusable robot helpers: joint state, FK, seeded IK, trajectories, gripper.

Build one Robot() and reuse it (clients are expensive to rebuild).
MoveIt's model frame here is `world` (FK header confirms), so poses go in as-is.
"""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # MoveIt model frame IS world here (FK header says world; verified)
TCP_OFF = float(M["hand"]["tcp_offset_m"])


def hand_pose_from_tcp(tcp_xyz, R):
    """Hand-frame origin given the TCP point and rotation matrix R (hand->world)."""
    return np.asarray(tcp_xyz) - TCP_OFF * R[:, 2]


def side_grasp_R(yaw_deg, pitch_deg):
    """Hand rotation for a side approach: hand z points horizontally along
    `yaw` (deg, from +x, ccw) and is pitched down by `pitch`; hand y (finger
    axis) stays horizontal."""
    yaw, pitch = math.radians(yaw_deg), math.radians(pitch_deg)
    z = np.array([math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch)])
    y = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def top_down_R(yaw_deg):
    """Hand z straight down, finger axis (hand y) along world direction yaw."""
    yaw = math.radians(yaw_deg)
    z = np.array([0, 0, -1.0])
    y = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk.wait_for_service(10); self.ik.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))
        self._js_t = time.time()

    def joints(self, fresh=True):
        if fresh:
            self._js = {}
        while not self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in ARM]

    def finger_gap(self):
        js = self.joints()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    def _call(self, cli, req, timeout=60):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def fk_hand(self, q=None):
        """World-frame (xyz, R) of panda_hand for joint vector q (default current)."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        res = self._call(self.fk, req)
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = Rot.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
        return xyz, R

    def tcp(self, q=None):
        xyz, R = self.fk_hand(q)
        return xyz + TCP_OFF * R[:, 2], R

    def solve_ik(self, hand_xyz_world, R, seed=None, attempts=3):
        """Seeded IK for a hand pose in world frame. Returns joint list or None."""
        seed = self.arm_q() if seed is None else seed
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pl = np.asarray(hand_xyz_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, pl)
        # the IK tip link is panda_link8; panda_hand = link8 * Rz(-45deg)
        R8 = np.asarray(R) @ Rot.from_euler("z", 45, degrees=True).as_matrix()
        qx, qy, qz, qw = Rot.from_matrix(R8).as_quat()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, (qx, qy, qz, qw))
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(map(float, seed))
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1
        for _ in range(attempts):
            res = self._call(self.ik, req)
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def ik_tcp(self, tcp_xyz, R, seed=None):
        return self.solve_ik(hand_pose_from_tcp(tcp_xyz, R), R, seed)

    def move_joints(self, waypoints, seconds, timeout=600):
        """Send a multi-point trajectory (list of joint vectors, list of times)."""
        if not isinstance(seconds, (list, tuple)):
            n = len(waypoints)
            seconds = [seconds * (i + 1) / n for i in range(n)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        for q, t in zip(waypoints, seconds):
            pt = JointTrajectoryPoint(positions=list(map(float, q)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=timeout)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        code = res.result().result.error_code
        q = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q, waypoints[-1]))
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap


    def fk_links(self, q, links=("panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7", "panda_hand")):
        req = GetPositionFK.Request()
        req.fk_link_names = list(links)
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        res = self._call(self.fk, req)
        return {n: np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
                for n, ps in zip(links, res.pose_stamped)}

    def best_ik_tcp(self, tcp_xyz, R, seeds, min_link_z=0.95, verbose=True):
        """Try several seeds; keep solutions whose arm links stay above min_link_z
        (world) and whose wrist is not flipped; return the one closest to seeds[0]."""
        best, best_cost = None, 1e9
        for sd in seeds:
            sol = self.ik_tcp(tcp_xyz, R, seed=sd)
            if sol is None:
                continue
            links = self.fk_links(sol)
            lowest = min(v[2] for k, v in links.items() if k != "panda_hand")
            if lowest < min_link_z:
                if verbose: print(f"   reject: link z {lowest:.3f}")
                continue
            cost = sum((a - b) ** 2 for a, b in zip(sol, seeds[0])) + 2.0 * abs(sol[4])
            if verbose: print(f"   cand cost={cost:.2f} q={np.round(sol, 2)} lowest_link_z={lowest:.3f}")
            if cost < best_cost:
                best, best_cost = sol, cost
        return best

    def move_converged(self, waypoints, seconds, tol=0.005, retries=6):
        """Send the trajectory, then re-send the final point until converged."""
        code, err = self.move_joints(waypoints, seconds)
        for _ in range(retries):
            if err < tol:
                break
            print("   re-sending final point to converge")
            code, err = self.move_joints([waypoints[-1]], max(2.0, 4.0 * err))
        return code, err


SEEDS = [[0, -0.785, 0, -2.356, 0, 1.571, 0.785], [-0.9, 0.65, 0.0, -2.65, 2.3, 1.65, 0.65],
         [-0.2, 0.9, 0.0, -1.9, 2.3, 1.9, 0.6], [0, 0.6, 0, -2.0, 0, 2.6, 0.8],
         [0.3, 0.8, -0.3, -2.0, 2.4, 1.8, 0.9], [-0.5, 0.4, 0.3, -2.4, 2.2, 1.7, 0.5]]


def cart_line(r, R, a, b, step, seed, max_jump=1.2):
    """IK waypoints along the straight TCP segment a->b (inclusive of b),
    keeping joint-space continuity (falls back to other seeds if needed)."""
    a, b = np.asarray(a), np.asarray(b)
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
    out = []
    for i in range(1, n + 1):
        p = a + (b - a) * i / n
        sol = r.ik_tcp(p, R, seed=seed)
        if sol is None or max(abs(x - y) for x, y in zip(sol, seed)) > max_jump:
            cands = []
            for sd in [seed] + SEEDS:
                s2 = r.ik_tcp(p, R, seed=sd)
                if s2 is not None:
                    cands.append((max(abs(x - y) for x, y in zip(s2, seed)), s2))
            if not cands:
                raise SystemExit(f"IK failed at {p}")
            jump, sol = min(cands, key=lambda c: c[0])
            print(f"   (re-seeded IK at {p.round(3)}, jump {jump:.2f})")
        out.append(sol); seed = sol
    return out
