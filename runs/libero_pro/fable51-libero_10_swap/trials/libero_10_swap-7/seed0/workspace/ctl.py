"""Reusable arm controller: joint state, FK, IK, trajectory, gripper.

World frame = panda_link0 frame + BASE offset (from TF world->panda_link0).
Poses passed to ik()/fk() here are WORLD-frame hand poses; conversion to
the planner's base frame happens inside.
"""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE = np.zeros(3)  # FK/IK service poses are already world-frame (verified vs TF)
TCP_OFF = float(M["hand"]["tcp_offset_m"])
TABLE_Z = 0.4255
# top-down grasp: hand z down, fingers open along world y
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def q_down_yaw(yaw):
    """Top-down grasp rotated by yaw (rad) about world z."""
    qz = (0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2))
    return quat_mul(qz, Q_DOWN)


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no fjt server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik_cli.wait_for_service(10), "no ik"
        assert self.fk_cli.wait_for_service(10), "no fk"

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

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

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def wrench(self):
        self._wr.pop("m", None)
        while "m" not in self._wr:
            self.spin(0.2)
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z,
                         w.torque.x, w.torque.y, w.torque.z])

    def _call(self, cli, req, timeout=60):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def fk(self, q=None, link="panda_hand"):
        """World-frame pose (xyz, quat) of link for arm config q."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        res = self._call(self.fk_cli, req)
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z,
                     p.orientation.w)

    def tcp(self, q=None):
        xyz, quat = self.fk(q)
        return xyz + TCP_OFF * quat_to_R(quat)[:, 2], quat

    def ik(self, xyz_world, quat, seed=None, at_tcp=True, tries=3):
        """Arm joints placing hand (or TCP) at a world pose; None if fail."""
        xyz = np.array(xyz_world, dtype=float)
        if at_tcp:
            xyz = xyz - TCP_OFF * quat_to_R(quat)[:, 2]
        xyz = xyz - BASE
        if seed is None:
            seed = self.arm_q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            req.ik_request.ik_link_name = "panda_hand"
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, xyz)
            (p.orientation.x, p.orientation.y,
             p.orientation.z, p.orientation.w) = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            res = self._call(self.ik_cli, req)
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def move(self, q, seconds=3.0, via=None):
        """Send trajectory to q (optionally through via points); wait."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        for i, w in enumerate(wps):
            pt = JointTrajectoryPoint(positions=[float(v) for v in w])
            t = seconds * (i + 1) / len(wps)
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        h = send.result()
        res = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        r = res.result()
        code = r.result.error_code if r else None
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move: code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result()
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.result.reached_goal} "
              f"stalled={r.result.stalled} gap={gap:.4f}", flush=True)
        return gap

    def goto(self, xyz, quat=Q_DOWN, seconds=3.0, at_tcp=True, seed=None):
        q = self.ik(xyz, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            print(f"  IK FAILED for {np.round(xyz,3)}", flush=True)
            return None
        self.move(q, seconds)
        p, _ = self.tcp() if at_tcp else self.fk()
        print(f"  now at {'tcp' if at_tcp else 'hand'}={np.round(p,4)} "
              f"target={np.round(xyz,4)} err={np.linalg.norm(p-np.array(xyz)):.4f}",
              flush=True)
        return q
