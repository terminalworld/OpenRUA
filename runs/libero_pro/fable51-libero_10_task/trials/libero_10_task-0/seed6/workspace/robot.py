#!/usr/bin/env python3
"""Small helper library for this Panda workstation: one node, reusable clients.

world->panda_link0 = (-0.51, 0, 0.42). Verified: MoveIt FK/IK here take/return
poses in the WORLD frame when frame_id is left EMPTY.
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

M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM_JOINTS = TRAJ["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
# Verified empirically: MoveIt FK/IK on this machine use the WORLD frame as model
# frame (panda_link0 sits at world (-0.51, 0, 0.42)); so no base offset is applied.
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])
TABLE_Z_WORLD = 0.425


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("agent_robot")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no FJT server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"

    # ---- sensing ----
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        end = time.time() + 15
        while "m" not in self._js and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_positions(self):
        j = self.joints()
        return [j[n] for n in ARM_JOINTS]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def _arm_state(self, positions=None):
        js = JointState()
        js.name = list(ARM_JOINTS)
        js.position = list(positions if positions is not None else self.arm_positions())
        return js

    def fk_hand(self, positions=None):
        """Hand (panda_hand) pose in WORLD frame: (xyz, quat xyzw)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._arm_state(positions)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_world(self, positions=None):
        pos, q = self.fk_hand(positions)
        R = quat_to_R(*q)
        tcp = pos + TCP_OFF * R[:, 2]
        return tcp + BASE_IN_WORLD, q

    # ---- planning ----
    def ik_tcp_world(self, tcp_world, q, seed=None):
        """IK for a TCP position in WORLD frame with hand quaternion q (xyzw). Returns arm positions or None."""
        R = quat_to_R(*q)
        hand_world = np.asarray(tcp_world, float) - TCP_OFF * R[:, 2]
        hand_base = hand_world - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = self._arm_state(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            log("IK failed", None if res is None else res.error_code.val, "for tcp", tcp_world)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM_JOINTS]

    # ---- acting ----
    def move_joints(self, positions, seconds=3.0, via=None):
        """Send one trajectory (optionally through 'via' waypoints, list of (positions, t))."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        pts = []
        for pos, t in (via or []):
            pt = JointTrajectoryPoint(positions=[float(x) for x in pos])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        assert gh is not None and gh.accepted, "FJT goal rejected"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = None if r is None else r.result.error_code
        now = self.arm_positions()
        err = float(np.max(np.abs(np.array(now) - np.array(positions))))
        log(f"move done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp_world(self, tcp_world, q, seconds=3.0, seed=None):
        sol = self.ik_tcp_world(tcp_world, q, seed)
        if sol is None:
            return None
        code, err = self.move_joints(sol, seconds)
        tcp, _ = self.tcp_world()
        log(f"tcp now world={tcp.round(4)} target={np.asarray(tcp_world).round(4)}")
        return code, err, tcp

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        log(f"gripper cmd={width} reached={r.reached_goal} stalled={r.stalled} pos={r.position:.4f} gap={gap:.4f}")
        return gap
