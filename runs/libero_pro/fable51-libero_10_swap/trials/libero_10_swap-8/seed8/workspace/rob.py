#!/usr/bin/env python3
"""Small robot helper: joint state, FK, IK, trajectory, gripper, servo.

World frame = panda_link0 + BASE offset (from TF world->panda_link0).
All public poses are in WORLD coordinates; hand pose = panda_hand frame.
"""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GR = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
BASE = np.zeros(3)  # FK/IK services already answer in WORLD coordinates
# IK targets the group tip link panda_link8; panda_hand = link8 * Rz(-45deg)
RZ45 = np.array([[np.cos(np.pi/4), -np.sin(np.pi/4), 0],
                 [np.sin(np.pi/4), np.cos(np.pi/4), 0], [0, 0, 1]])
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(R).as_quat()  # x y z w
    return q if q[3] >= 0 else -q


def R_from_axes(hz, hy):
    """Rotation whose columns are hand x,y,z given hand z (approach) and
    hand y (finger-open axis) in world."""
    hz = np.asarray(hz, float); hz /= np.linalg.norm(hz)
    hy = np.asarray(hy, float); hy -= hz * hy.dot(hz); hy /= np.linalg.norm(hy)
    hx = np.cross(hy, hz)
    return np.stack([hx, hy, hz], 1)


class Rob:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GR["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return np.array([j[n] for n in JOINTS])

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def wrench(self):
        self._wr.pop("m", None)
        while "m" not in self._wr:
            self.spin(0.2)
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z,
                         w.torque.x, w.torque.y, w.torque.z])

    def fk(self, q=None, link="panda_hand"):
        """World pose (p, R) of link for arm config q (default current)."""
        if q is None:
            q = self.arm_q()
        if not self.fk_cli.wait_for_service(5):
            raise RuntimeError("no FK")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if res is None else res.error_code.val}")
        ps = res.pose_stamped[0].pose
        p = np.array([ps.position.x, ps.position.y, ps.position.z]) + BASE
        R = quat_to_R([ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w])
        return p, R

    def tcp(self, q=None):
        p, R = self.fk(q)
        return p + TCP * R[:, 2], R

    # ---------------- IK ----------------
    def ik(self, p_world, R, seed=None, at_tcp=False, timeout=30):
        p = np.asarray(p_world, float)
        if at_tcp:
            p = p - TCP * R[:, 2]
        p = p - BASE
        q = R_to_quat(np.asarray(R) @ RZ45)  # hand -> link8 orientation
        if seed is None:
            seed = self.arm_q()
        if not self.ik_cli.wait_for_service(5):
            raise RuntimeError("no IK")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(x) for x in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    # ---------------- acting ----------------
    def move_q(self, q, seconds=3.0, waypoints=None, retries=3):
        """Send trajectory to q (optionally via waypoints [(q, t), ...])."""
        if not self.fjt.wait_for_server(10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
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
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        if not gh.accepted:
            raise RuntimeError("goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        code = rf.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q)).max()
        print(f"[move_q] error_code={code} max_joint_err={err:.4f}", flush=True)
        # -5 = goal tolerance violated: this controller caps joint speed
        # (~0.19 rad/s); resending the same goal converges (docs/30-action)
        if code == -5 and retries > 0 and err > 0.01:
            return self.move_q(q, max(2.0, err / 0.15), retries=retries - 1)
        return code, err

    def move_pose(self, p_world, R, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik(p_world, R, seed=seed, at_tcp=at_tcp)
        if q is None:
            raise RuntimeError(f"IK failed for {p_world}")
        return self.move_q(q, seconds)

    def gripper(self, width, timeout=120):
        if not self.grip.wait_for_server(10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GR["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        f = self.fingers()
        print(f"[gripper] reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def servo(self, v_world, n=20, frame=None):
        """Stream n twist messages with linear velocity v (m/s) in base frame."""
        msg = TwistStamped()
        msg.header.frame_id = frame or TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v_world)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw",
                                            lambda m: got.setdefault("m", m), 1)
        while "m" not in got:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got["m"], "bgr8"))
        return out

    def depth_world(self, cam, t, q):
        """World point cloud (H,W,3) from a camera's depth image."""
        from cv_bridge import CvBridge
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/depth/image_raw",
                                            lambda m: got.setdefault("m", m), 1)
        while "m" not in got:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        d = CvBridge().imgmsg_to_cv2(got["m"], "passthrough").astype(float)
        got = {}
        sub = self.node.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                                            lambda m: got.setdefault("m", m), 1)
        while "m" not in got:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        k = got["m"].k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        H, W = d.shape
        vs, us = np.mgrid[0:H, 0:W]
        P = np.stack([(us - cx) * d / fx, (vs - cy) * d / fy, d], -1)
        return P @ quat_to_R(q).T + np.asarray(t)


BIRD = (np.array([-0.2, 0.0, 3.0]), (0.7071, 0.7071, 0.0, 0.0))
