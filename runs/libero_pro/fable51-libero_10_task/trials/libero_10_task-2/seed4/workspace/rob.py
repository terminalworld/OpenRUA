#!/usr/bin/env python3
"""Reusable helpers: joints, FK, IK, trajectory, servo, gripper, wrench."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
GR = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
HOME = [0.0, -0.5, 0.0, -2.0, 0.0, 1.6, 0.785]  # hand ~(-0.26, 0, 1.57) pointing down
BASE_IN_WORLD = np.zeros(3)  # FK/IK on this machine already report/accept WORLD-frame poses (verified vs TF)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def topdown_quat(yaw):
    """Hand Z pointing down (world -Z), hand X rotated by yaw about world Z.
    yaw=0 -> hand X along world +X, fingers close along world Y."""
    # (1,0,0,0) = 180deg about X: Z->-Z, Y->-Y. Then yaw about world Z.
    qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return quat_mul(qz, (1.0, 0.0, 0.0, 0.0))


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GR["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.tfbuf = Buffer(); self.tfl = TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.05): rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None: self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints(); return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints(); return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None: self.spin(0.2)
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk(self, q=None, link="panda_hand", world=True):
        """Pose of link. Returns (pos, quat xyzw). world=True adds base offset."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if r is None else r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        if world: pos = pos + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik(self, pos_world, quat, seed=None, timeout=60):
        """IK for hand pose in world frame. Returns joint list or None."""
        p = np.asarray(pos_world, float) - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"  # default tip is panda_link8 (45deg off)
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK failed: {None if r is None else r.error_code.val}")
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_q(self, q, seconds=3.0, waypoints=None, retry=1):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = list(waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        if code != 0 and err > 0.02 and retry > 0:
            print("  retrying (controller lag)")
            return self.move_q(q, seconds, retry=retry - 1)
        return code, err

    def move_pose(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik(pos_world, quat, seed)
        if q is None: return None
        self.move_q(q, seconds)
        p, _ = self.fk()
        print(f"  reached hand pos {p.round(4)} target {np.asarray(pos_world).round(4)}")
        return q

    def move_line(self, p_end, quat, step=0.02, speed=0.05, max_jump=0.4, seed=None):
        """Straight Cartesian line to p_end (world) as one multi-point joint trajectory.
        IK per waypoint, each seeded on the previous, so the branch cannot flip."""
        q = list(seed) if seed is not None else self.arm_q()
        p0, _ = self.fk(q)
        p_end = np.asarray(p_end, float)
        n = max(1, int(np.ceil(np.linalg.norm(p_end - p0) / step)))
        wps = []
        t = 0.0
        dt = max(0.3, (np.linalg.norm(p_end - p0) / n) / speed)
        for i in range(1, n + 1):
            p = p0 + (p_end - p0) * i / n
            sol = self.ik(p, quat, seed=q)
            if sol is None:
                print(f"move_line: IK failed at waypoint {i}/{n} {p.round(4)}"); return None
            jump = np.abs(np.array(sol) - np.array(q)).max()
            if jump > max_jump:
                print(f"move_line: branch jump {jump:.2f} at waypoint {i}/{n}; abort"); return None
            t += dt
            wps.append((sol, t))
            q = sol
        last_q, last_t = wps[-1]
        self.move_q(last_q, last_t, waypoints=wps[:-1])
        p, _ = self.fk()
        print(f"  line reached {p.round(4)} target {p_end.round(4)}")
        return last_q

    def servo(self, v_world, n=20, dt=0.05, w=(0, 0, 0)):
        """Stream twist (linear in base frame == world frame orientation)."""
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v_world)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, w)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg); self.spin(dt)

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width); g.command.max_effort = float(GR["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def grab(self, topic, msg_type, timeout=30.0):
        got = {}
        sub = self.node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
        end = time.time() + timeout
        while "m" not in got and time.time() < end:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        if "m" not in got:
            raise RuntimeError(f"no message on {topic}")
        return got["m"]

    def cloud(self, cam="birdview"):
        """World-frame HxWx3 point cloud from a depth camera (NaN where invalid)."""
        depth = self.grab(f"/{cam}/depth/image_raw", Image)
        info = self.grab(f"/{cam}/color/camera_info", CameraInfo)
        H, W = depth.height, depth.width
        d = np.frombuffer(depth.data, dtype=np.float32).reshape(H, W).copy()
        d[~(np.isfinite(d) & (d > 0))] = np.nan
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        frame = f"{cam}_optical_frame"
        end = time.time() + 10
        while time.time() < end and not self.tfbuf.can_transform("world", frame, rclpy.time.Time()):
            self.spin(0.2)
        t = self.tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        R = quat_R(q.x, q.y, q.z, q.w)
        T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
        return pc @ R.T + T

    def close(self):
        self.node.destroy_node(); rclpy.shutdown()
