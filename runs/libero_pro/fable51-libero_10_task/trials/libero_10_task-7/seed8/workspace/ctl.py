#!/usr/bin/env python3
"""Reusable arm controller: joint state, FK, IK, trajectory, gripper.

Poses are hand-frame poses; helpers convert TCP poses (fingertip point)
to hand poses with machine.yaml's tcp_offset_m.
"""
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

M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = TRAJ["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)


# IK solves for panda_link8; panda_hand = link8 * Rz(-45deg). To get a
# desired HAND orientation, ask IK for hand * Rz(+45deg).
RZ45 = (0.0, 0.0, 0.3826834, 0.9238795)


def hand_to_ik(q_hand):
    return qmul(q_hand, RZ45)


# desired HAND orientations (approach = -Z world)
Q_HAND_X = (0.7071068, -0.7071068, 0.0, 0.0)   # fingers close along world X
Q_HAND_Y = (1.0, 0.0, 0.0, 0.0)                # fingers close along world Y


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tcp_to_hand(p, q):
    """TCP point -> hand origin (back along the hand's +Z approach axis)."""
    R = quat_to_R(*q)
    return np.asarray(p, float) - TCP * R[:, 2]


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.traj = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk.wait_for_service(10); self.ik.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, m):
        self._js["m"] = m
        self._js["t"] = time.time()

    def joints(self, fresh=True):
        """dict name->position from a fresh /joint_states."""
        t0 = time.time()
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if time.time() - t0 > 20:
                raise RuntimeError("no /joint_states")
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def _seed(self, q=None):
        js = JointState()
        q = q if q is not None else self.arm_q()
        js.name = list(ARM); js.position = [float(v) for v in q]
        return js

    def hand_pose(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p, o = r.pose_stamped[0].pose.position, r.pose_stamped[0].pose.orientation
        return np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w])

    def tcp_pose(self, q=None):
        p, o = self.hand_pose(q)
        R = quat_to_R(*o)
        return p + TCP * R[:, 2], o

    def solve_ik(self, hand_p, q, seed=None, attempts=3):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, hand_p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, hand_to_ik(q))
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        for _ in range(attempts):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[n] for n in ARM]
        raise RuntimeError(f"IK failed ({r and r.error_code.val}) for {np.round(hand_p,3)} {q}")

    def ik_tcp(self, tcp_p, q, seed=None):
        return self.solve_ik(tcp_to_hand(tcp_p, q), q, seed)

    def move_joints(self, points, seconds, retries=2):
        """points: list of 7-vectors; seconds: list of cumulative times.
        On a tolerance-violation result with a real residual, resend the
        final point (docs: usually controller lag, converges on resend)."""
        code, err = self._send(points, seconds)
        n = 0
        while (code != 0 or err > 0.03) and err > 0.02 and n < retries:
            n += 1
            print(f"  resend final point (attempt {n})")
            code, err = self._send([points[-1]], [max(2.0, seconds[-1] / len(points))])
        return code, err

    def _send(self, points, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for p, s in zip(points, seconds):
            pt = JointTrajectoryPoint(positions=[float(v) for v in p])
            pt.time_from_start = Duration(sec=int(s), nanosec=int((s % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        if h is None or not h.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        res = rf.result()
        code = res.result.error_code if res else None
        got = np.array(self.arm_q())
        err = np.abs(got - np.array(points[-1])).max()
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, tcp_p, q, seconds=3.0, seed=None):
        sol = self.ik_tcp(tcp_p, q, seed)
        code, err = self.move_joints([sol], [seconds])
        p, o = self.tcp_pose()
        print(f"  tcp now {np.round(p,4)} target {np.round(tcp_p,4)} q={np.round(o,3)}")
        return sol

    def move_tcp_line(self, tcp_to, q, n=4, seconds=3.0):
        """Straight TCP line from the current TCP to tcp_to in n IK steps."""
        p0, _ = self.tcp_pose()
        wps = [p0 + (np.asarray(tcp_to, float) - p0) * (i / n) for i in range(1, n + 1)]
        return self.move_tcp_path(wps, q, dt=seconds / n)

    def move_tcp_path(self, waypoints, q, dt=2.0):
        """Several TCP waypoints in one trajectory (IK chained by seed)."""
        sols, seed, times = [], None, []
        t = 0.0
        for wp in waypoints:
            seed = self.ik_tcp(wp, q, seed)
            sols.append(seed); t += dt; times.append(t)
        code, err = self.move_joints(sols, times)
        p, o = self.tcp_pose()
        print(f"  tcp now {np.round(p,4)} target {np.round(waypoints[-1],4)}")
        return sols

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={np.round(gap,4)}")
        return gap
