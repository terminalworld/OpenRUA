"""Helpers: IK / FK / trajectory / gripper / joint-state on one node."""
import math
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

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])  # world -> panda_link0 (TF, static)
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_quat(yaw):
    """Hand pointing straight down (hand z = -world z), hand x rotated by yaw
    about world z. Fingers close along hand y."""
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    return (c, s, 0.0, 0.0)  # qz(yaw) * qx(pi)


class Robot:
    def __init__(self, name="robot_lib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        t0 = time.time()
        while self._js is None and time.time() - t0 < 10:
            self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    # ---- kinematics (base frame = panda_link0) ----
    def ik(self, pos_base, quat, seed=None, timeout=60):
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"  # group tip is link8 (45 deg off)
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        js = JointState()
        js.name = list(ARM)
        js.position = [float(v) for v in seed]
        req.ik_request.robot_state.joint_state = js
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp_world(self, pos_world, yaw, seed=None, tcp_z_below=0.0):
        """IK for the TCP point (fingertip centre) at a WORLD position with the
        hand pointing down and yaw about world z."""
        q = down_quat(yaw)
        R = quat_R(*q)
        hand_world = np.array(pos_world) - TCP * R[:, 2]
        # machine fact (probed): MoveIt's model frame here IS world
        return self.ik(hand_world, q, seed)

    def fk(self, q, link="panda_hand"):
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        js = JointState()
        js.name = list(ARM)
        js.position = [float(v) for v in q]
        req.robot_state.joint_state = js
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, quat

    def tcp_world(self, q=None):
        q = q if q is not None else self.arm_q()
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        return pos + TCP * R[:, 2], quat  # FK already answers in world

    # ---- motion ----
    def move(self, waypoints, times):
        """waypoints: list of 7-vectors; times: cumulative seconds."""
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        # hold the last point briefly: the controller settles late
        pt = JointTrajectoryPoint(positions=[float(v) for v in waypoints[-1]])
        t = times[-1] + 1.0
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        goal.trajectory.points.append(pt)
        for attempt in range(3):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            q_now = self.arm_q()
            err = float(np.max(np.abs(np.array(q_now) - np.array(waypoints[-1]))))
            print(f"move: error_code={code} max_joint_err={err:.4f}")
            if err < 0.01:
                break
            # resend only the final point (short) to converge
            goal.trajectory.points = goal.trajectory.points[-1:]
            goal.trajectory.points[0].time_from_start = Duration(sec=1, nanosec=500000000)
        return code, err

    def gripper(self, width):
        self.grip.wait_for_server(timeout_sec=10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f
