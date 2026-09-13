"""Reusable robot helpers for this Panda workstation (built once per process)."""
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
from tf2_ros import Buffer, TransformListener

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
# Verified: /compute_fk and /compute_ik on this machine already work in the
# `world` frame (FK header says world and matches TF world->panda_hand), so no
# base offset is needed.
BASE_IN_WORLD = np.zeros(3)
TCP_OFF = M["hand"]["tcp_offset_m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    """Rotation matrix -> (x, y, z, w)."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("agent_lib")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.fk_cli.wait_for_service(10); self.ik_cli.wait_for_service(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 20:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in ARM]

    def fingers(self):
        js = self.joints()
        return js["panda_finger_joint1"], js["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """Returns (pos_world, R_world) of link for arm config q (default: current)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = quat_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk(q)
        return pos + TCP_OFF * R[:, 2], R

    # ---------- planning ----------
    def ik(self, pos_world, R, seed=None, at_tcp=True, tries=1):
        """IK for hand pose in world; pos may be the TCP point if at_tcp."""
        pos = np.asarray(pos_world, float)
        if at_tcp:
            pos = pos - TCP_OFF * R[:, 2]
        pos_base = pos - BASE_IN_WORLD
        # /compute_ik solves for the group tip panda_link8; panda_hand shares its
        # origin but is rotated -45 deg about the local z (URDF; verified with
        # /compute_fk), so convert the desired HAND rotation into link8's.
        c, s = np.cos(np.pi / 4), np.sin(np.pi / 4)
        R8 = np.asarray(R) @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        qx, qy, qz, qw = R_quat(R8)
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, (qx, qy, qz, qw))
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=0, nanosec=200_000_000)
        for _ in range(tries):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    # ---------- acting ----------
    def move(self, q, seconds=3.0, waypoints=None, resend=2, jtol=0.02):
        """Send one trajectory (optionally through waypoints [(q, t), ...]) and
        wait; re-send the final point while the joints lag it by > jtol rad."""
        code = self._move_once(q, seconds, waypoints)
        for _ in range(resend):
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            if err <= jtol:
                break
            code = self._move_once(q, max(1.5, seconds / 2), None)
        return code

    def _move_once(self, q, seconds, waypoints):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if waypoints:
            for wq, wt in waypoints:
                pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
                pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q))
        print(f"move: error_code={code} max joint err={err.max():.4f}")
        return code

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        print(f"gripper({width}): pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}")
        return r

    def servo(self, v, n=20, dt=0.05):
        """Stream n twist messages with linear velocity v (m/s, base frame)."""
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            self.twist_pub.publish(msg)
            self.spin(dt)

    def move_tcp(self, pos, R, seconds=3.0, seed=None, tol=0.003, refine=2):
        """IK -> trajectory; then re-command with the measured offset folded in
        (the controller shows steady-state error under load) until within tol."""
        pos = np.asarray(pos, float)
        cmd = pos.copy()
        q = None
        for i in range(1 + refine):
            q = self.ik(cmd, R, seed=seed, tries=3)
            if q is None:
                print("IK failed for", cmd)
                return None
            self.move(q, seconds if i == 0 else max(1.0, seconds / 2))
            p, _ = self.tcp()
            err = pos - p
            print(f"tcp now {np.round(p, 4)} target {np.round(pos, 4)} err {np.linalg.norm(err):.4f}")
            if np.linalg.norm(err) < tol:
                break
            cmd = cmd + err
            seed = q
        return q


# Standard top-down grasp orientation: hand z pointing down (-z world),
# hand x axis along world direction given by yaw (fingers close along hand y).
def down_R(yaw=0.0):
    c, s = np.cos(yaw), np.sin(yaw)
    # columns: x_hand, y_hand, z_hand in world
    x = np.array([c, s, 0.0])
    z = np.array([0.0, 0.0, -1.0])
    y = np.cross(z, x)
    return np.stack([x, y, z], 1)
