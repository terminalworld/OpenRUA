"""Helper: persistent clients for joint state, FK, IK, trajectory, gripper, servo.

Base frame panda_link0 sits at world (-0.51, 0, 0.42); helpers take WORLD
coordinates and convert. Fingertip (TCP) is 0.1034 m along hand +Z.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_W = np.array([0.0, 0.0, 0.0])  # FK/IK services already work in world frame (verified)
TCP = float(M["hand"]["tcp_offset_m"])
# hand pointing straight down, fingers opening along world Y
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down_quat(yaw):
    """Hand pointing down (180 deg about X) then yawed about world Z."""
    # q = qz(yaw) * qx(pi)
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    # qx(pi) = (1,0,0,0); qz = (0,0,sz,cz); product qz*qx:
    # w = cz*0 - sz*0 = 0 ... compute generally
    ax, ay, az, aw = 0.0, 0.0, sz, cz
    bx, by, bz, bw = 1.0, 0.0, 0.0, 0.0
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("robo_helper")
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.update(m=m), 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self._wr.update(m=m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.clear()
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
        self._wr.clear()
        end = time.time() + 5
        while "m" not in self._wr and time.time() < end:
            self.spin(0.2)
        if "m" not in self._wr:
            return None
        f = self._wr["m"].wrench.force
        return np.array([f.x, f.y, f.z])

    def hand_pose_world(self, q=None):
        """FK of panda_hand -> (xyz world, quat xyzw)."""
        q = q or self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = q
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z,
                     p.orientation.w)

    def tcp_world(self, q=None):
        xyz, quat = self.hand_pose_world(q)
        R = quat_to_R(*quat)
        return xyz + TCP * R[:, 2], quat

    # ---------- acting ----------
    def solve_ik(self, xyz_world, quat=Q_DOWN, at_tcp=True, seed=None):
        xyz = np.array(xyz_world, dtype=float)
        if at_tcp:
            xyz = xyz - TCP * quat_to_R(*quat)[:, 2]
        xyz = xyz - BASE_W
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        (p.orientation.x, p.orientation.y,
         p.orientation.z, p.orientation.w) = quat
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = seed or self.arm_q()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, q, seconds=3.0, via=None, retries=1):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via, 1):
                t = seconds * i / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = self._settle_err(q)
        print(f"  traj error_code={code} max_joint_err={err:.4f}", flush=True)
        if err > 0.02 and retries > 0:
            print("  resending (controller lag)", flush=True)
            return self.move_joints(q, max(seconds, 2.0), retries=retries - 1)
        return code, err

    def _settle_err(self, q):
        """Re-read joints until they stop changing; return max |q - target|."""
        prev = np.array(self.arm_q())
        for _ in range(20):
            cur = np.array(self.arm_q())
            if np.abs(cur - prev).max() < 1e-4:
                break
            prev = cur
        return float(np.abs(cur - np.array(q)).max())

    def move_to(self, xyz_world, quat=Q_DOWN, seconds=3.0, at_tcp=True):
        q = self.solve_ik(xyz_world, quat, at_tcp)
        if q is None:
            print(f"  IK FAILED for {xyz_world}", flush=True)
            return None
        code, err = self.move_joints(q, seconds)
        tcp, _ = self.tcp_world()
        print(f"  tcp now {tcp.round(4)} (target {np.round(xyz_world,4)})", flush=True)
        return tcp

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
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def servo(self, vx=0.0, vy=0.0, vz=0.0, ticks=20, hz=20):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(1.0 / hz)
        return self.tcp_world()[0]
