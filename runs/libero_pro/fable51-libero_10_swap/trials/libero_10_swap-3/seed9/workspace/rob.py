"""Robot helper: joint state, FK, IK, trajectory, gripper, servo. World-frame poses.

world -> panda_link0 is a pure translation BASE (from /tf).
"""
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
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE = np.array([0.0, 0.0, 0.0])  # FK/IK on this machine are already in world coords (verified)
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def down_quat(yaw=0.0):
    """Hand pointing straight down (hand z = world -z); yaw rotates finger axis
    about world z. yaw=0: hand x along world +x (fingers close along... hand y).
    Panda hand: fingers open/close along hand Y axis."""
    # hand z -> -world z; hand x -> world x rotated by yaw; hand y accordingly
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])  # columns = hand axes in world
    return R_to_quat(R)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
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
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        t0 = time.time()
        while self._wr is None and time.time() - t0 < 5:
            self.spin(0.2)
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def fk(self, q=None, link="panda_hand"):
        """returns (pos_world, quat) of link"""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(5)
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = quat_to_R(quat)
        return pos + TCP * R[:, 2], quat

    # ---------- IK ----------
    def ik(self, pos_world, quat, seed=None, at_tcp=True, timeout=60):
        pos = np.array(pos_world, float)
        if at_tcp:
            R = quat_to_R(quat)
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE
        # IK tip link is panda_link8 = hand rotated +45deg about hand z
        Rh = quat_to_R(quat)
        c, s_ = np.cos(np.pi / 4), np.sin(np.pi / 4)
        R8 = Rh @ np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        quat = R_to_quat(R8)
        self.ik_cli.wait_for_service(5)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        if seed is None:
            seed = self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    # ---------- motion ----------
    def move_q(self, q, seconds=3.0, via=None):
        """via: list of (q, t) intermediate points"""
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            for vq, vt in via:
                pt = JointTrajectoryPoint(positions=[float(v) for v in vq])
                pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        qa = self.arm_q()
        err = np.abs(qa - np.array(q)).max()
        print(f"move_q: code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, seed=None, retries=3, tol=0.02):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            print(f"IK FAILED for {pos}")
            return None
        for _ in range(retries):
            code, err = self.move_q(q, seconds)
            if err < tol:
                break
        p, _ = self.tcp()
        print(f"  tcp now {p.round(4)} target {np.array(pos).round(4)}")
        return q

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        fg = self.fingers()
        print(f"gripper: reached={r.reached_goal} stalled={r.stalled} fingers={fg[0]:.4f},{fg[1]:.4f}")
        return fg

    def servo(self, v, n=20, frame=None):
        """publish n twist messages with linear velocity v (world/base frame, m/s)"""
        msg = TwistStamped()
        msg.header.frame_id = frame or TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)


def tilt_quat(theta):
    """fingers along world x; hand z points down and toward -y by theta (rad),
    so the wrist leans toward +y. theta=0 == down_quat(pi/2)."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[0, 1, 0], [c, 0, -s], [-s, 0, -c]])  # columns: hand x, y, z in world
    return R_to_quat(R)
