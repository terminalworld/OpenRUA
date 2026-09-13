"""Reusable robot helpers: one node, clients built once (see docs/40 P1)."""
import time
import numpy as np
import rclpy
import yaml
import kin
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
# Verified: /compute_fk with empty frame_id returns poses whose frame_id is
# "world" and which match the TF chain world->panda_hand, so the planner's
# model frame IS world on this machine (base sits at world (-0.51,0,0.42)).
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        assert self.fjt.wait_for_server(10), "no FJT server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        """Fresh joint dict (arm + fingers)."""
        self._js = None
        end = time.time() + 10
        while self._js is None and time.time() < end:
            self.spin(0.2)
        assert self._js is not None, "no /joint_states"
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            self.spin(0.2)
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        return np.array([f.x, f.y, f.z])

    # ---- kinematics (base frame = panda_link0, frame_id left empty) ----
    def fk_hand(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        js = JointState()
        js.name = list(ARM)
        js.position = list(q if q is not None else self.arm_q())
        req.robot_state.joint_state = js
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        assert r is not None and r.error_code.val == 1, f"FK failed {r}"
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]),
                r.pose_stamped[0].header.frame_id)

    def ik_hand(self, pos_base, quat, seed=None):
        """IK for the HAND frame at pos (base frame). Returns joint list or None."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        js = JointState()
        js.name = list(ARM)
        js.position = list(seed if seed is not None else self.arm_q())
        req.ik_request.robot_state.joint_state = js
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print("IK failed", None if r is None else r.error_code.val)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp_world(self, pos_world, quat, seed=None):
        """IK with the fingertip-centre (TCP) at a WORLD position."""
        R = quat_to_R(*quat)
        hand_world = np.asarray(pos_world, float) - TCP * R[:, 2]
        return self.ik_hand(hand_world - BASE_IN_WORLD, quat, seed)

    @staticmethod
    def yaw_of(q):
        """Yaw of the hand x axis in world (deg); 0 = fingers along world y."""
        R = quat_to_R(*q)
        return np.degrees(np.arctan2(R[1, 0], R[0, 0])), np.degrees(np.arccos(-R[2, 2]))

    def ik_down(self, pos_world, yaw_deg=0.0, seed=None, verbose=True):
        """Top-down TCP pose at a world position with a given finger yaw,
        via the analytic 6-DoF IK in kin.py (the machine's /compute_ik is
        position-only and null-space-unstable, verified)."""
        seed = list(seed if seed is not None else self.arm_q())
        sol, (perr, rerr) = kin.ik(pos_world, yaw_deg, seed)
        if verbose:
            print(f"  ik_down: {'ok' if sol is not None else 'FAIL'} perr={perr*1000:.2f}mm rerr={rerr:.2f}deg")
        return None if sol is None else list(sol)

    def move_line(self, p1, yaw_deg=0.0, seconds=3.0, step=0.02, tol=0.02):
        """Straight-line TCP move from the current pose to world p1 via a
        multi-waypoint trajectory (joint-space interpolation alone bends
        the TCP path in XY by centimetres)."""
        p0, _ = self.tcp_world()
        p1 = np.asarray(p1, float)
        n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / step)) + 1)
        q = self.arm_q()
        qs = []
        for i in range(1, n):
            p = p0 + (p1 - p0) * i / (n - 1)
            q = self.ik_down(p, yaw_deg, seed=q, verbose=False)
            if q is None:
                print("  move_line: IK failed at", p.round(3))
                return False
            qs.append(q)
        jumps = np.abs(np.diff(np.array([self.arm_q()] + qs), axis=0)).max(axis=1)
        if jumps[1:].max(initial=0) > 0.4:
            print("  move_line: discontinuous waypoints, max jump", jumps.round(3))
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for i, qq in enumerate(qs, 1):
            t = seconds * i / len(qs)
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(qs[-1])).max()
        tcp, _ = self.tcp_world()
        print(f"  move_line: error_code={code} max_joint_err={err:.4f} tcp={tcp.round(3)} "
              f"(target {p1.round(3)}, off {np.linalg.norm(tcp-p1)*1000:.1f}mm)")
        if err >= tol:
            # controller lag on the last point: settle with a short single-point goal
            print("  move_line: settling on final point")
            return self.move_q(qs[-1], 2.0, tol=tol, retries=1)
        return True

    def tcp_world(self):
        p, q, _ = self.fk_hand()
        R = quat_to_R(*q)
        return p + BASE_IN_WORLD + TCP * R[:, 2], q

    # ---- motion ----
    def move_q(self, q, seconds=3.0, tol=0.02, retries=2):
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            gh = send.result()
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"  move_q: error_code={code} max_joint_err={err:.4f}")
            if err < tol:
                return True
        return False

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def servo(self, v, ticks=20):
        """Stream a base-frame linear velocity (m/s) for `ticks` messages."""
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)
