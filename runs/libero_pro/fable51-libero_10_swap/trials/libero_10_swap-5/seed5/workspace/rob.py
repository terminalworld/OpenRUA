#!/usr/bin/env python3
"""Small helper library: joint state, FK, IK, trajectory, gripper (one node, reused clients)."""
import sys
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
from geometry_msgs.msg import WrenchStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    # returns x,y,z,w
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(R).as_quat()
    return q


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")

    def _on_js(self, m):
        self._js = dict(zip(m.name, m.position))

    def _on_wr(self, m):
        f = m.wrench.force
        t = m.wrench.torque
        self._wr = np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def spin(self, s=0.1):
        rclpy.spin_once(self.node, timeout_sec=s)

    def joints(self, fresh=True):
        if fresh:
            self._js = {}
            end = time.time() + 20
            while not self._js and time.time() < end:
                self.spin(0.2)
        return self._js

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in ARM]

    def fingers(self):
        js = self.joints()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    def wrench(self):
        self._wr = {}
        end = time.time() + 10
        while isinstance(self._wr, dict) and time.time() < end:
            self.spin(0.2)
        return self._wr

    def _seed(self, q=None):
        js = JointState()
        q = q if q is not None else self.arm_q()
        js.name = list(ARM)
        js.position = [float(x) for x in q]
        return js

    def fk(self, q=None, link="panda_hand"):
        self.fk_cli.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def ik_solve(self, pos, quat, seed=None, timeout=30):
        self.ik_cli.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.timeout.sec = 2
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_q(self, q, seconds=3.0, waypoints=None):
        """Send trajectory; waypoints: list of (q, t) before the final point."""
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        for wq, wt in (waypoints or []):
            pt = JointTrajectoryPoint(positions=[float(x) for x in wq])
            pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        h = send.result()
        res = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qa = np.array(self.arm_q())
        err = np.abs(qa - np.array(q)).max()
        print(f"move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik_solve(pos, quat, seed)
        if q is None:
            print("IK FAILED for", pos, quat, flush=True)
            return None
        return self.move_q(q, seconds)

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}", flush=True)
        return r


def topdown_quat(yaw_deg=0.0):
    """Hand z pointing down (world -z), hand x rotated by yaw about world z."""
    from scipy.spatial.transform import Rotation
    R = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix() @ np.diag([1.0, -1.0, -1.0])
    return R_quat(R)
