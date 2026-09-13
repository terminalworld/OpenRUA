#!/usr/bin/env python3
"""Reusable arm helper: FK/IK, trajectory, gripper, joint-state reads.

All poses in WORLD frame; converted to the planner's base frame using the
static world->panda_link0 transform read from TF (machine.yaml planning
facts: IK wants frame_id empty = base frame).
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
PLAN = M["planning"]
TCP_OFF = M["hand"]["tcp_offset_m"]
# Verified on this machine: compute_fk returns panda_link0 at world
# (-0.51, 0, 0.42) and IK accepts the raw FK pose back -> the planner's
# model frame IS world here, so no base offset is applied.
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])


def down_quat(yaw_deg=0.0):
    """Hand Z pointing down (world -Z); fingers open along world X when
    yaw=0 (Rz(90)*Rx(180)); yaw rotates the finger axis about world Z."""
    r = Rot.from_euler("z", 90 + yaw_deg, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    return r.as_quat()  # x y z w


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 10)
        self.ik = self.node.create_client(GetPositionIK, PLAN["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.traj.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.joints()

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            end = time.time() + 15
            while self._js is None and time.time() < end:
                self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            self.spin(0.1)
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        return np.array([f.x, f.y, f.z])

    def _seed(self, q=None):
        js = JointState()
        q = q if q is not None else self.arm_q()
        js.name = list(ARM)
        js.position = [float(v) for v in q]
        return js

    def fk_world(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def ik_world(self, pos_world, quat, seed=None, tcp=True, tries=3):
        """IK for a world-frame pose. If tcp, pos is the fingertip point
        (offset TCP_OFF along hand +Z)."""
        pos = np.array(pos_world, float)
        if tcp:
            R = Rot.from_quat(quat).as_matrix()
            pos = pos - TCP_OFF * R[:, 2]
        pb = pos - BASE_IN_WORLD
        last = None
        for i in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = PLAN["group"]
            # the group's default tip is panda_link8, which is yawed 45 deg
            # from panda_hand (finger axis = hand Y): target the hand itself
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pb)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state = self._seed(seed)
            req.ik_request.timeout = Duration(sec=2)
            req.ik_request.avoid_collisions = False
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
            last = None if res is None else res.error_code.val
            # perturb the seed for the retry
            seed = list(np.array(self.arm_q() if seed is None else seed) + np.random.uniform(-0.3, 0.3, 7))
        raise RuntimeError(f"IK failed (code {last}) for world {pos_world}")

    # measured on this machine: joint7 tracks at most ~0.2 rad/s (others
    # comfortably faster); a goal whose duration is too short for the j7
    # travel returns -5 and stops short.
    VMAX = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.18])

    LIMITS = np.array(FJT["limits_rad"])

    def ik_best(self, pos_world, yaw_deg, seed=None, tcp=True, n=4):
        """Top-down grasp IK: fingers are symmetric so yaw and yaw+180 are
        the same grasp; sample both and several seeds, return the solution
        closest (j7-weighted) to the seed/current config, away from limits."""
        base = np.array(self.arm_q() if seed is None else seed)
        best, best_cost = None, None
        for yaw in (yaw_deg, yaw_deg + 180.0, yaw_deg - 180.0):
            quat = down_quat(yaw)
            for i in range(n):
                s = base if i == 0 else base + np.random.uniform(-0.4, 0.4, 7)
                try:
                    q = np.array(self.ik_world(pos_world, quat, seed=list(s), tcp=tcp, tries=1))
                except RuntimeError:
                    continue
                margin = np.minimum(q - self.LIMITS[:, 0], self.LIMITS[:, 1] - q).min()
                if margin < 0.1:
                    continue
                cost = (np.abs(q - base) / self.VMAX).sum()
                if best is None or cost < best_cost:
                    best, best_cost = q, cost
        if best is None:
            raise RuntimeError(f"no acceptable IK for {pos_world} yaw {yaw_deg}")
        return list(best)

    def move_q(self, q, seconds=3.0, via=None, retries=2):
        cur = np.array(self.arm_q())
        pts = ([np.array(v[0]) for v in via] if via else []) + [np.array(q)]
        need = 0.0
        prev = cur
        for p in pts:
            need += (np.abs(p - prev) / self.VMAX).max()
            prev = p
        total = max(seconds, need + 0.5)
        if via:
            scale = total / seconds
            via = [(vq, vt * scale) for vq, vt in via]
        seconds = total
        code, err = self._move_q(q, seconds, via)
        n = 0
        while err > 0.02 and n < retries:
            n += 1
            print(f"  resend ({n}) to converge", flush=True)
            code, err = self._move_q(q, max(2.0, seconds / 2), None)
        return code, err

    def _move_q(self, q, seconds=3.0, via=None):
        """One trajectory goal to joint config q (optionally through via
        points as list of (q, t))."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            for vq, vt in via:
                pt = JointTrajectoryPoint(positions=[float(v) for v in vq])
                pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = None if r is None else r.result.error_code
        # the result can come back (often -5) while the sim is still
        # executing: wait until the joints stop changing
        err = self.settle(q)
        print(f"  traj done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def settle(self, q_target=None, timeout=240.0, tol=0.01):
        """Poll /joint_states (2 Hz wall) until arm joints are stationary
        for 3 consecutive reads (or within tol of q_target)."""
        prev, same = None, 0
        end = time.time() + timeout
        err = float("nan")
        while time.time() < end:
            cur = np.array(self.arm_q())
            if q_target is not None:
                err = np.abs(cur - np.array(q_target)).max()
                if err < tol:
                    same += 1
                    if same >= 2:
                        return err
            if prev is not None and np.abs(cur - prev).max() < 1e-5:
                same += 1
                if same >= 3:
                    return err
            elif prev is not None:
                same = 0
            prev = cur
            time.sleep(0.3)
        print("  settle: timeout", flush=True)
        return err

    def move_pose(self, pos_world, quat, seconds=3.0, seed=None, tcp=True):
        q = self.ik_world(pos_world, quat, seed=seed, tcp=tcp)
        code, err = self.move_q(q, seconds)
        p, _ = self.fk_world(link="panda_hand")
        R = Rot.from_quat(quat).as_matrix()
        tcp_p = p + TCP_OFF * R[:, 2]
        print(f"  TCP now at world {tcp_p.round(4)} (target {np.round(pos_world,4)})", flush=True)
        return q

    def gripper(self, width, effort=None):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(effort if effort is not None else GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        # fingers may still be moving when the result returns
        prev, same, end = None, 0, time.time() + 120
        while time.time() < end and same < 3:
            g = np.array(self.finger_gap())
            same = same + 1 if prev is not None and np.abs(g - prev).max() < 1e-5 else 0
            prev = g
            time.sleep(0.3)
        g = self.finger_gap()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={np.round(g,4)}", flush=True)
        return g
