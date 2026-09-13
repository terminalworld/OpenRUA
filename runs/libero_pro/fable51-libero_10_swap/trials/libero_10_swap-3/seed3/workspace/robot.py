"""Small helper lib: joint state, FK, IK, trajectories, gripper. World frame."""
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
BASE = np.array([0.0, 0.0, 0.0])  # FK/IK with empty frame_id are already in WORLD (verified)
TCP_OFF = M["hand"]["tcp_offset_m"]

# hand pointing down, fingers along world X
Q_DOWN_FX = (0.7071068, 0.7071068, 0.0, 0.0)
# hand pointing down, fingers along world Y
Q_DOWN_FY = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def qmul(a, b):
    """Hamilton product a*b, quats as (x, y, z, w)."""
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


# IK tip link is panda_link8; panda_hand = link8 * rotz(-45deg). So
# link8 = hand * rotz(+45deg) (right-multiply, local z).
Q_HAND_TO_LINK8 = (0.0, 0.0, 0.3826834, 0.9238795)


def hand_to_link8(q_hand):
    return qmul(q_hand, Q_HAND_TO_LINK8)


def yaw_quat(q, yaw):
    """Rotate quaternion q by yaw (rad) about world Z."""
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    w1, x1, y1, z1 = c, 0.0, 0.0, s
    x2, y2, z2, w2 = q
    w = w1 * w2 - (x1 * x2 + y1 * y2 + z1 * z2)
    x = w1 * x2 + w2 * x1 + (y1 * z2 - z1 * y2)
    y = w1 * y2 + w2 * y1 + (z1 * x2 - x1 * z2)
    z = w1 * z2 + w2 * z1 + (x1 * y2 - y1 * x2)
    return (x, y, z, w)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_lib")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        assert self.fjt.wait_for_server(10), "no FJT server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.fk_cli.wait_for_service(10), "no FK"
        assert self.ik_cli.wait_for_service(10), "no IK"

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

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    # ---- kinematics ----
    def fk(self, q=None, link="panda_hand"):
        """Returns (pos_world, quat) of link for arm config q (default current)."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res and res.error_code.val}"
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        return pos + TCP_OFF * quat_to_R(quat)[:, 2], quat

    def ik(self, pos_world, quat, seed=None, tcp=True, timeout=1.0):
        """IK for hand (or TCP if tcp=True) at world pose. Returns list of 7 or None."""
        pos = np.array(pos_world, float)
        if tcp:
            pos = pos - TCP_OFF * quat_to_R(quat)[:, 2]
        pos = pos - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        q8 = hand_to_link8(quat)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---- motion ----
    def move_joints(self, waypoints, times):
        """waypoints: list of 7-vectors; times: cumulative seconds per waypoint."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        assert gh is not None and gh.accepted, "goal rejected"
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        q = self.arm_q()
        err = float(np.max(np.abs(np.array(q) - np.array(waypoints[-1]))))
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, secs=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {pos}")
        return self.move_joints([q], [secs])

    def move_tcp_line(self, p0, p1, quat, n=5, secs=3.0):
        """Straight TCP line p0->p1 via n IK waypoints (seeded consecutively)."""
        seed = self.arm_q()
        wps, ts = [], []
        for i in range(1, n + 1):
            p = np.array(p0) + (np.array(p1) - np.array(p0)) * i / n
            q = self.ik(p, quat, seed=seed)
            if q is None:
                raise RuntimeError(f"IK failed at waypoint {i}: {p}")
            seed = q
            wps.append(q)
            ts.append(secs * i / n)
        return self.move_joints(wps, ts)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f
