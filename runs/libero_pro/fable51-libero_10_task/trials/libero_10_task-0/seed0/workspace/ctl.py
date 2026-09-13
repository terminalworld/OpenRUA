#!/usr/bin/env python3
"""Reusable robot control helpers: FK, IK, trajectory, gripper, joint state.

Coordinates in this module are WORLD frame unless suffixed _b (base frame).
world -> panda_link0 is a pure translation WB = (-0.51, 0, 0.42).
"""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

# verified empirically: FK/IK poses (frame_id empty) are already in WORLD
# (the model root sits at world (-0.51, 0, 0.42)); no offset to apply.
WB = np.array([0.0, 0.0, 0.0])
M = yaml.safe_load(open('/workspace/machine.yaml'))
TRAJ = next(a for a in M['actuators'] if a['kind'] == 'joint_trajectory')
GRIP = next(a for a in M['actuators'] if a['kind'] == 'gripper')
JOINTS = TRAJ['joints']
LIMITS = TRAJ['limits_rad']
TCP = M['hand']['tcp_offset_m']
# top-down grasp, fingers close along world Y (hand x = world x)
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_yaw_down(yaw):
    """Hand pointing down (z down), rotated by yaw about world z."""
    # q = Rz(yaw) * Rx(pi)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sy,cy)
    # product (w1,v1)*(w2,v2): w = w1w2 - v1.v2 ; v = w1 v2 + w2 v1 + v1 x v2
    w1, v1 = cy, np.array([0, 0, sy])
    w2, v2 = 0.0, np.array([1.0, 0, 0])
    w = w1 * w2 - v1 @ v2
    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    return (v[0], v[1], v[2], w)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('ctl')
        self._js = {}
        self.node.create_subscription(JointState, '/joint_states',
                                      lambda m: self._js.__setitem__('m', m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ['port'])
        self.grip = ActionClient(self.node, GripperCommand, GRIP['port'])
        self.ik = self.node.create_client(GetPositionIK, M['planning']['ik_service'])
        self.fk = self.node.create_client(GetPositionFK, '/compute_fk')
        assert self.fjt.wait_for_server(timeout_sec=20)
        assert self.grip.wait_for_server(timeout_sec=20)
        assert self.ik.wait_for_service(timeout_sec=20)
        assert self.fk.wait_for_service(timeout_sec=20)

    def joints(self):
        self._js.pop('m', None)
        while 'm' not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js['m']
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def finger_gap(self):
        j = self.joints()
        return j['panda_finger_joint1'] - j['panda_finger_joint2']

    def _arm_state(self, q):
        js = JointState()
        js.name = list(JOINTS)
        js.position = [float(x) for x in q]
        return js

    def fk_hand(self, q=None):
        """Hand pose in WORLD frame: (xyz, quat xyzw)."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ['panda_hand']
        req.robot_state.joint_state = self._arm_state(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        p = r.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + WB
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik_world(self, xyz_w, quat, seed=None, at_tcp=True):
        """IK for a WORLD pose. If at_tcp, xyz is the fingertip (TCP) point."""
        xyz = np.array(xyz_w, dtype=float) - WB
        if at_tcp:
            R = quat_R(*quat)
            xyz = xyz - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M['planning']['group']
        req.ik_request.ik_link_name = 'panda_hand'
        req.ik_request.pose_stamped.header.frame_id = ''
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._arm_state(seed if seed is not None else self.arm_q())
        req.ik_request.timeout.sec = 1
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print('IK failed', None if r is None else r.error_code.val)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        q = [sol[j] for j in JOINTS]
        for i, (lo, hi) in enumerate(LIMITS):
            if not (lo <= q[i] <= hi):
                print('IK solution violates limit', JOINTS[i], q[i])
                return None
        return q

    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        wps = list(waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(x) for x in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=120)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f'move done code={code} max_joint_err={err:.4f}')
        return code, err

    def move_world(self, xyz_w, quat, seconds=3.0, seed=None):
        q = self.ik_world(xyz_w, quat, seed=seed)
        if q is None:
            return None
        self.move_q(q, seconds)
        p, _ = self.fk_hand()
        R = quat_R(*quat)
        tcp = p + TCP * R[:, 2]
        print('TCP now (world):', tcp.round(4))
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP['max_effort'])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        print(f'gripper reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}')
        return r


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
