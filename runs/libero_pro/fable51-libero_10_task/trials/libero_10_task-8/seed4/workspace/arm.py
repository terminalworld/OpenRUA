"""Reusable arm helpers: joint state, FK, IK, trajectory, gripper, servo."""
import math
import sys
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
TCP = M["hand"]["tcp_offset_m"]
BASE = np.array([0.0, 0.0, 0.0])  # FK/IK poses come back in world coords on this machine (verified)


def quat_mul(a, b):
    """hamilton product, xyzw."""
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


# the IK group's tip is panda_link8; panda_hand = link8 rotated -45deg about z
Q_Z45 = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))


def hand_to_link8(quat):
    return quat_mul(quat, Q_Z45)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self, name="arm_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def _on_js(self, msg):
        self._js = msg

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        """dict name->position (arm + fingers)."""
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    # ---- kinematics (planner frame = panda_link0) ----
    def fk(self, q=None, link="panda_hand"):
        """hand pose in WORLD: (xyz, quat xyzw)."""
        q = q or self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp(self, q=None):
        xyz, quat = self.fk(q)
        R = quat_to_R(*quat)
        return xyz + TCP * R[:, 2], quat

    def ik(self, xyz_world, quat, at_tcp=True, seed=None, timeout=60):
        """joint list for HAND (or TCP) pose in world; None on failure."""
        R = quat_to_R(*quat)
        quat8 = hand_to_link8(quat)
        xyz = np.array(xyz_world, float)
        if at_tcp:
            xyz = xyz - TCP * R[:, 2]
        xyz = xyz - BASE  # into planner frame
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = [float(v) for v in xyz]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = [float(v) for v in quat8]
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = list(seed or self.arm_q())
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            print("IK: no answer", file=sys.stderr)
            return None
        if res.error_code.val != 1:
            print(f"IK failed code={res.error_code.val}", file=sys.stderr)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    # ---- motion ----
    def move(self, q_list, secs, verbose=True):
        """one trajectory; q_list = list of joint vectors (waypoints) or one vector."""
        if not isinstance(q_list[0], (list, tuple, np.ndarray)):
            q_list = [q_list]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(q_list)
        for i, q in enumerate(q_list):
            t = secs * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code if res.result() else None
        cur = self.arm_q()
        err = max(abs(a - b) for a, b in zip(cur, q_list[-1]))
        if verbose:
            print(f"move: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, xyz, quat, secs=3.0, seed=None):
        q = self.ik(xyz, quat, at_tcp=True, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {xyz}")
        code, err = self.move(q, secs)
        got, _ = self.tcp()
        print(f"  tcp now {np.round(got, 4)} (target {np.round(xyz, 4)})")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def servo(self, vx=0.0, vy=0.0, vz=0.0, n=20, dt=0.05):
        """stream n twist messages (base frame)."""
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(dt)
