"""Helper library: joint state, FK, IK, trajectory, gripper on this Panda."""
import math
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
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_W = np.array([0.0, 0.0, 0.0])  # MoveIt model frame == world here (verified via FK)
TCP = M["hand"]["tcp_offset_m"]

# top-down grasp orientation: hand Z pointing down (world -Z), hand X along world +X
# quaternion (x,y,z,w) for rotation of pi about X axis
Q_DOWN = (1.0, 0.0, 0.0, 0.0)
# IK tip link is panda_link8; panda_hand = link8 * Rz(-45deg). Convert hand quat -> link8 quat.
Q_L8_FROM_HAND = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))  # Rz(+45deg)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def q_down_yaw(yaw):
    """Top-down orientation rotated by yaw about world Z."""
    qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return quat_mul(qz, Q_DOWN)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def spin(self, t=0.1):
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
        return [j[n] for n in JOINTS]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def fk(self, q=None):
        """Hand pose in WORLD frame: (pos[3], quat[4])."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik(self, pos_w, quat, seed=None, at_tcp=True, timeout=60):
        """IK for hand pose given in WORLD frame. pos is the TCP point if at_tcp."""
        pos = np.array(pos_w, dtype=float)
        if at_tcp:
            R = _quat_to_R(*quat)
            pos = pos - TCP * R[:, 2]
        pos_b = pos - BASE_W
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = pos_b
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat_mul(quat, Q_L8_FROM_HAND)
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = list(seed if seed is not None else self.arm_q())
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None:
            raise RuntimeError("IK no answer")
        if r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move_q(self, q, seconds=3.0, via=None):
        """Send trajectory; via = list of (q, t) intermediate points."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        for vq, vt in (via or []):
            pt = JointTrajectoryPoint(positions=[float(x) for x in vq])
            pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = max(abs(a - b) for a, b in zip(self.arm_q(), q))
        print(f"  move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_pose(self, pos_w, quat, seconds=3.0, seed=None, at_tcp=True):
        q = self.ik(pos_w, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            print(f"  IK FAILED for {pos_w}")
            return None
        self.move_q(q, seconds)
        p, _ = self.fk()
        tcp = p + TCP * _quat_to_R(*quat)[:, 2] if at_tcp else p
        print(f"  reached tcp={np.round(tcp, 4)} target={np.round(pos_w, 4)}")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap


def _quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
