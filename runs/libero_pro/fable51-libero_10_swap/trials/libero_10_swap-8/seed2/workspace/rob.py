"""Persistent helper for this Panda: joint state, FK, IK, trajectory, gripper, servo."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
BASE = np.zeros(3)  # verified: FK/IK poses are already in the world frame
TCP = M["hand"]["tcp_offset_m"]

# hand pointing down, fingers closing along world x / world y
Q_DOWN_FX = (0.7071068, 0.7071068, 0.0, 0.0)
Q_DOWN_FY = (0.0, 1.0, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    from scipy.spatial.transform import Rotation
    return tuple(Rotation.from_matrix(R).as_quat())  # (x, y, z, w)


def q_down_yaw(yaw):
    """Hand pointing down; finger axis rotated by yaw from world x."""
    fy = np.array([math.cos(yaw), math.sin(yaw), 0.0])  # hand y in world
    fz = np.array([0.0, 0.0, -1.0])
    fx = np.cross(fy, fz)
    return R_quat(np.column_stack([fx, fy, fz]))


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(20)
        self.grip.wait_for_server(20)
        self.ik.wait_for_service(20)
        self.fk.wait_for_service(20)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm(self):
        d = self.joints()
        return [d[j] for j in JOINTS]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    # ---- kinematics (planning frame = panda_link0) ----
    def fk_world(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(q if q is not None else self.arm())
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik_world(self, pos, quat, seed=None, at_tcp=False, timeout=20.0):
        pos = np.asarray(pos, float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = (pos - BASE).tolist()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = list(seed if seed is not None else self.arm())
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout + 30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    # ---- motion ----
    def move_joints(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        wps = (via or []) + [q]
        for i, w in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(x) for x in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        return code, self.settle(q)

    def settle(self, q, tol=0.01, max_reads=200, resend=True):
        """Poll joint states until within tol of q; resend the goal once if stuck."""
        last = None
        stuck = 0
        for i in range(max_reads):
            cur = np.array(self.arm())
            err = np.abs(cur - np.array(q)).max()
            if err < tol:
                return err
            if last is not None and np.abs(cur - last).max() < 1e-4:
                stuck += 1
                if stuck >= 15:
                    if resend:
                        print(f"  settle: stuck at err={err:.3f}, resending goal", flush=True)
                        return self._resend(q)
                    return err
            else:
                stuck = 0
            last = cur
        return err

    def _resend(self, q):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        return self.settle(q, resend=False)

    def move_pose(self, pos, quat, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik_world(pos, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            return None
        return self.move_joints(q, seconds)

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        # fingers keep moving after the result; poll until stable
        last, stable = None, 0
        for _ in range(100):
            f = self.fingers()
            if last is not None and abs(f[0] - last[0]) < 1e-5:
                stable += 1
                if stable >= 8:
                    break
            else:
                stable = 0
            last = f
        return r.reached_goal, r.stalled, f

    def servo(self, v, n=20, dt=0.05):
        """Stream n twist messages with linear velocity v (m/s, base frame)."""
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = [float(x) for x in v]
        for _ in range(n):
            self.tw.publish(msg)
            self.spin(dt)
