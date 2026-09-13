#!/usr/bin/env python3
"""Session helper: one node, reusable clients, world-frame poses.

World->base (panda_link0) is a pure translation on this machine (read
from TF at start).  All public functions take WORLD coordinates.
"""
import struct
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])

# hand pointing straight down, fingers along world Y
Q_DOWN = Rot.from_quat([1.0, 0.0, 0.0, 0.0])


def q_down_yaw(yaw_deg):
    """Downward hand, finger axis at (90 + yaw_deg) deg from world +X
    (0 -> fingers along Y).

    Machine fact (verified via TF): the IK group's tip is panda_link8 and
    panda_hand is yawed -45 deg from it, so the finger axis (hand Y) ends
    up at link8_yaw - 45 deg.  Compensate here.
    """
    return (Rot.from_euler("z", yaw_deg - 45.0, degrees=True) * Q_DOWN).as_quat()


class RB:
    def __init__(self, name="rb"):
        rclpy.init()
        self.n = rclpy.create_node(name)
        self.js = {}
        self.n.create_subscription(JointState, "/joint_states",
                                   lambda m: self.js.__setitem__("m", m), 1)
        self.tf = Buffer()
        TransformListener(self.tf, self.n)
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.n, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.gr.wait_for_server(10)
        self.spin(0.5)
        t = self.lookup("world", "panda_link0")
        self.base = np.array(t[0])
        self.log(f"base in world: {self.base}")

    def log(self, *a):
        print(time.strftime("%H:%M:%S"), *a, flush=True)

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    # ---------------- sensing ----------------
    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def lookup(self, a, b):
        end = time.time() + 10
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.1)
            if self.tf.can_transform(a, b, rclpy.time.Time()):
                break
        t = self.tf.lookup_transform(a, b, rclpy.time.Time())
        tr, r = t.transform.translation, t.transform.rotation
        return (tr.x, tr.y, tr.z), (r.x, r.y, r.z, r.w)

    def hand_pose(self):
        """Hand frame + TCP point in world (from TF, after a fresh spin)."""
        self.spin(0.3)
        p, q = self.lookup("world", "panda_hand")
        R = Rot.from_quat(q)
        tcp = np.array(p) + TCP * R.as_matrix()[:, 2]
        return np.array(p), np.array(q), tcp

    def grab(self, topic, typ, timeout=30):
        got = {}
        sub = self.n.create_subscription(typ, topic,
                                         lambda m: got.setdefault("m", m), 1)
        end = time.time() + timeout
        while "m" not in got and time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        self.n.destroy_subscription(sub)
        if "m" not in got:
            raise RuntimeError(f"no msg on {topic}")
        return got["m"]

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        m = self.grab(f"/{cam}/color/image_raw", Image)
        img = CvBridge().imgmsg_to_cv2(m, "bgr8")
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, img)
        return img

    def cloud(self, cam):
        """World-frame point cloud (H,W,3) of the camera's current depth."""
        d = self.grab(f"/{cam}/depth/image_raw", Image)
        info = self.grab(f"/{cam}/color/camera_info", CameraInfo)
        z = np.frombuffer(d.data, dtype="<f4").reshape(d.height, d.width)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        v, u = np.mgrid[0:d.height, 0:d.width]
        pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], -1)
        p, q = self.lookup("world", f"{cam}_optical_frame")
        R = Rot.from_quat(q).as_matrix()
        return pc @ R.T + np.array(p)

    # ---------------- acting ----------------
    def ik_world(self, xyz, quat, at_tcp=True, seed=None):
        xyz = np.array(xyz, float)
        R = Rot.from_quat(quat).as_matrix()
        if at_tcp:
            xyz = xyz - TCP * R[:, 2]
        # machine fact (verified): the planner's model frame here is
        # `world`; an empty frame_id means world, "panda_link0" also works
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = "world"
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        (p.orientation.x, p.orientation.y,
         p.orientation.z, p.orientation.w) = map(float, quat)
        s = JointState()
        cur = seed or self.joints()
        for j in ARM:
            s.name.append(j)
            s.position.append(float(cur[j]))
        req.ik_request.robot_state.joint_state = s
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"IK failed: {None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def traj(self, points, seconds):
        """points: list of joint vectors; seconds: total time (spread evenly)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        n = len(points)
        for i, q in enumerate(points):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(x) for x in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf)
        code = rf.result().result.error_code
        cur = self.joints()
        err = max(abs(cur[j] - points[-1][i]) for i, j in enumerate(ARM))
        self.log(f"traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, xyz, quat, seconds=3.0, at_tcp=True, via=None):
        """IK to world pose (TCP by default) then one trajectory."""
        q = self.ik_world(xyz, quat, at_tcp)
        pts = ([via] if via is not None else []) + [q]
        code, err = self.traj(pts, seconds)
        for _ in range(3):
            if code == 0 and err < 0.02:
                break
            # machine fact: -5 on long goals is usually controller lag;
            # resending the same goal converges
            self.log("retrying goal")
            code, err = self.traj([q], max(seconds, 3.0))
        p, hq, tcp = self.hand_pose()
        self.log(f"tcp now {np.round(tcp, 4)} (target {np.round(xyz, 4)})")
        return code, err, tcp

    def line(self, a, b, quat, step=0.02, seconds=None, at_tcp=True):
        """Straight Cartesian TCP path a->b as one multi-point trajectory
        (IK per waypoint, each seeded with the previous solution)."""
        a, b = np.array(a, float), np.array(b, float)
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        seed = self.joints()
        pts = []
        for i in range(1, n + 1):
            q = self.ik_world(a + (b - a) * i / n, quat, at_tcp, seed=seed)
            seed = dict(zip(ARM, q))
            pts.append(q)
        seconds = seconds or max(2.0, n * 0.6)
        code, err = self.traj(pts, seconds)
        if code != 0 or err > 0.02:
            self.log("retrying final point")
            code, err = self.traj([pts[-1]], 2.0)
        p, hq, tcp = self.hand_pose()
        self.log(f"line done tcp {np.round(tcp, 4)} (target {np.round(b, 4)})")
        return code, err, tcp

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=120)
        r = rf.result().result
        self.spin(0.3)
        f = self.fingers()
        self.log(f"gripper({width}) reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def close(self):
        rclpy.shutdown()
