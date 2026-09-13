#!/usr/bin/env python3
"""Small control library for this Panda: joint state, FK, IK, trajectory, gripper.
World<->base: base = panda_link0 at world (-0.75, 0, 0.912), no rotation.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])
M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = TRAJ["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def qmul(a, b):
    """Hamilton product, quats as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


# The IK group's tip is panda_link8; panda_hand = link8 * Rz(-45deg). So a
# desired panda_hand orientation must be sent to IK as hand * Rz(+45deg).
Q_HAND_TO_LINK8 = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))


def topdown_quat(theta):
    """Hand pointing down (-Z world), rotated theta about world Z. Returns x,y,z,w."""
    return (math.cos(theta / 2), math.sin(theta / 2), 0.0, 0.0)


class Rob:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no fjt server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no ik"
        assert self.fk.wait_for_service(10), "no fk"

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        end = time.time() + 20
        while "m" not in self._js and time.time() < end:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def fk_world(self, q=None):
        """Hand pose in world: (pos[3], quat xyzw)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])  # FK answers in world
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik_world(self, pos, quat, seed=None, tries=5):
        """IK for hand pose in world frame. Returns joint list (manifest order)."""
        pos = np.asarray(pos, float)  # IK (empty frame_id) is interpreted in world here
        if seed is None:
            seed = self.arm_q()
        best = None
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            q8 = qmul(quat, Q_HAND_TO_LINK8)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            s = list(seed) if k == 0 else list(np.array(seed) + np.random.uniform(-0.15, 0.15, len(seed)))
            req.ik_request.robot_state.joint_state.position = [float(v) for v in s]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is None or res.error_code.val != 1:
                continue
            sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            q = [sol[n] for n in ARM]
            dist = float(np.abs(np.array(q) - np.array(seed)).max())
            if best is None or dist < best[1]:
                best = (q, dist)
            if dist < 0.6:
                break
        if best is None:
            raise RuntimeError(f"IK failed for world pos {pos}")
        return best[0]

    def move_q(self, q, seconds=3.0, via=None, retries=2):
        code, err = self._move_q(q, seconds, via)
        while err > 0.01 and retries > 0:  # controller lag: resend converges
            print("  resending (lag)", flush=True)
            code, err = self._move_q(q, max(2.0, seconds / 2), None)
            retries -= 1
        return code, err

    def _move_q(self, q, seconds, via):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via)
            for i, v in enumerate(via):
                t = seconds * (i + 1) / (n + 1)
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        assert gh is not None and gh.accepted, "goal rejected"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        cur = np.array(self.arm_q())
        err = float(np.abs(cur - np.array(q)).max())
        print(f"  traj error_code={code} max joint err={err:.4f}", flush=True)
        return code, err

    def move_world(self, pos, quat, seconds=3.0):
        q = self.ik_world(pos, quat)
        code, err = self.move_q(q, seconds)
        p, o = self.fk_world()
        print(f"  hand now at world {p.round(4)} quat {np.round(o, 3)}", flush=True)
        return p

    def move_tcp(self, tcp_pos, theta, seconds=3.0):
        """Move so the fingertip centre (TCP) is at tcp_pos (world), hand pointing down, yaw theta."""
        hand = np.asarray(tcp_pos, float) + np.array([0, 0, TCP])
        return self.move_world(hand, topdown_quat(theta), seconds)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f


if __name__ == "__main__":
    r = Rob()
    print("joints", r.joints())
    p, o = r.fk_world()
    print("hand world", p, "quat", o)
