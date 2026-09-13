#!/usr/bin/env python3
"""Reusable control helpers for the Panda (clients built once per process)."""
import sys, time, math
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import TwistStamped, WrenchStamped
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from cv_bridge import CvBridge
import cv2

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
# Verified: MoveIt FK/IK poses here already are in the WORLD frame (FK of the
# current config matches the eye-in-hand camera TF in world), so no offset.
BASE_IN_WORLD = np.zeros(3)
TCP_OFF = M["hand"]["tcp_offset_m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_quat(yaw):
    """Hand z pointing to world -z, fingers axis (hand y) rotated by yaw about world z.
    q = Rz(yaw) * Rx(pi)."""
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sin(yaw/2),cos(yaw/2))
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    # quaternion product Rz * Rx : (w1,x1,y1,z1)*(w2,x2,y2,z2)
    w1, x1, y1, z1 = cz, 0.0, 0.0, sz
    w2, x2, y2, z2 = 0.0, 1.0, 0.0, 0.0
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


class Ctl(Node):
    def __init__(self):
        super().__init__("ctl")
        self.js = None
        self.wrench = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._w_cb, 1)
        self.fjt = ActionClient(self, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self, GripperCommand, GRIP["port"])
        self.ik = self.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.create_publisher(TwistStamped, TW["port"], 10)
        self.bridge = CvBridge()
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        while self.js is None:
            rclpy.spin_once(self, timeout_sec=0.2)

    def _js_cb(self, m): self.js = m
    def _w_cb(self, m): self.wrench = m

    def spin(self, t=0.1):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def joints(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints(); return [d[j] for j in ARM]

    def finger(self):
        d = self.joints(); return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk_hand(self, q=None):
        """Hand pose in WORLD frame (position, R)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = q if q is not None else self.arm_q()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, R

    def tcp_world(self):
        pos, R = self.fk_hand()
        return pos + TCP_OFF * R[:, 2]

    def ik_world(self, pos_w, quat, seed=None, tcp=True):
        """IK for hand (or TCP if tcp=True) at world position with quaternion. Returns joint list or None."""
        pos_w = np.array(pos_w, float)
        if tcp:
            R = quat_to_R(*quat)
            pos_w = pos_w - TCP_OFF * R[:, 2]
        pb = pos_w - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = seed if seed is not None else self.arm_q()
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK failed code={r and r.error_code.val} for {pos_w}", flush=True)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_q(self, q, seconds=3.0, tol=0.02, retries=3):
        # controller lag often yields error_code -5 short of the goal; resend converges
        for i in range(retries):
            code, err = self._move_q_once(q, seconds)
            if err < tol:
                break
        return code, err

    def _move_q_once(self, q, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=600)
        code = res.result().result.error_code
        cur = np.array(self.arm_q()); err = np.abs(cur - np.array(q)).max()
        print(f"move_q code={code} maxerr={err:.4f}", flush=True)
        return code, err

    def move_path(self, qs, seconds_each=2.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        t = 0.0
        for q in qs:
            t += seconds_each
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=900)
        code = res.result().result.error_code
        cur = np.array(self.arm_q()); err = np.abs(cur - np.array(qs[-1])).max()
        print(f"move_path code={code} maxerr={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, pos_w, quat, seconds=3.0, seed=None):
        q = self.ik_world(pos_w, quat, seed=seed)
        if q is None:
            return None
        code, err = self.move_q(q, seconds)
        tcp = self.tcp_world()
        print(f"tcp now {tcp.round(4)} target {np.array(pos_w).round(4)}", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=300)
        r = res.result().result
        self.spin(0.3)
        f = self.finger()
        print(f"gripper({width}) reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}", flush=True)
        return f

    def servo(self, v, n=20, dt=0.05):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            self.twist_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=dt)

    def snap(self, cam, out=None):
        topic = cam if cam.startswith("/") else f"/{cam}/color/image_raw"
        out = out or f"/workspace/{cam.strip('/').split('/')[0]}.png"
        got = []
        sub = self.create_subscription(Image, topic, got.append, 1)
        while not got:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.destroy_subscription(sub)
        m = got[0]
        if "FC" in m.encoding or "16UC" in m.encoding:
            dep = self.bridge.imgmsg_to_cv2(m, "passthrough")
            np.save(out.rsplit(".", 1)[0] + ".npy", dep)
            return dep
        img = self.bridge.imgmsg_to_cv2(m, "bgr8")
        cv2.imwrite(out, img)
        return img


def birdview_objects(node, tag=""):
    """Snapshot birdview color+depth, segment objects above the table, return dict of measurements."""
    img = node.snap("birdview", f"/workspace/bird{tag}.png")
    dep = node.snap("/birdview/depth/image_raw", f"/workspace/bird{tag}_d.png")
    f = 579.4112549695428; cx, cy = 320, 240
    zw = 3.0 - dep
    out = []
    mask = ((zw > 0.44) & (zw < 0.70)).astype(np.uint8)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 40 or stats[i, cv2.CC_STAT_AREA] > 3000: continue
        m = lab == i
        vs, us = np.nonzero(m); dd = dep[m]
        X = -0.2 + (vs - cy) * dd / f; Y = (us - cx) * dd / f
        col = img[m].mean(0)
        out.append(dict(u=us.mean(), v=vs.mean(), X=(X.min(), X.max()), Y=(Y.min(), Y.max()),
                        cx=X.mean(), cy=Y.mean(), zmax=zw[m].max(), zmed=float(np.median(zw[m])), bgr=col.round(0), area=int(m.sum())))
    for o in out:
        print(f"u={o['u']:.0f} v={o['v']:.0f} X=[{o['X'][0]:.3f},{o['X'][1]:.3f}] Y=[{o['Y'][0]:.3f},{o['Y'][1]:.3f}] c=({o['cx']:.3f},{o['cy']:.3f}) zmax={o['zmax']:.3f} zmed={o['zmed']:.3f} bgr={o['bgr']} area={o['area']}", flush=True)
    return out, img, zw
