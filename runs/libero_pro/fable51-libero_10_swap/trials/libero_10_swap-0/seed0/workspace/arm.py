#!/usr/bin/env python3
"""Persistent helpers: joint state, FK, IK, trajectory, gripper. World<->base."""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = TRAJ["joints"]
BASE = np.zeros(3)  # verified: MoveIt model frame == world on this machine (FK of panda_link0 = (-0.51,0,0.42))
TCP = M["hand"]["tcp_offset_m"]
DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z pointing down, fingers along world y
DOWN_X = (0.7071068, 0.7071068, 0.0, 0.0)  # hand z down, yawed 90deg: fingers along world x


class Arm:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("arm_helper")
        self._js = None
        self.n.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.n, FollowJointTrajectory, TRAJ["port"])
        self.gr = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10) and self.gr.wait_for_server(10)
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.n, timeout_sec=t)

    def js(self):
        self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.js()
        return [j[k] for k in JOINTS]

    def fingers(self):
        j = self.js()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def hand_world(self):
        """FK: hand pose in world (pos, quat)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = self.arm_q()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def solve_ik(self, world_xyz, quat=DOWN, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        b = np.array(world_xyz) - BASE
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = JOINTS
        req.ik_request.robot_state.joint_state.position = seed or self.arm_q()
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK FAILED {None if r is None else r.error_code.val} for {world_xyz}", flush=True)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def traj(self, q, seconds=3.0, tol=0.02, tries=4):
        """Send the goal; the controller lags and reports -5 short of the
        target, so re-send (with the remaining distance) until converged."""
        for i in range(tries):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = JOINTS
            pt = JointTrajectoryPoint(positions=[float(x) for x in q])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.n, send, timeout_sec=60)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.n, res, timeout_sec=600)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"  traj[{i}] code={code} max_joint_err={err:.4f}", flush=True)
            if err < tol:
                break
            seconds = max(1.5, seconds * min(1.0, err / 0.5 + 0.3))
        return code, err

    def move_to(self, world_xyz, quat=DOWN, seconds=3.0):
        q = self.solve_ik(world_xyz, quat)
        if q is None:
            return False
        self.traj(q, seconds)
        pos, _ = self.hand_world()
        print(f"  hand now {pos.round(4)} target {np.array(world_xyz).round(4)}", flush=True)
        return True

    def grab(self, topic, msg_type, timeout=30.0):
        got = {}
        sub = self.n.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
        t0 = time.time()
        while "m" not in got and time.time() - t0 < timeout:
            self.spin(0.2)
        self.n.destroy_subscription(sub)
        return got.get("m")

    def cloud_world(self, cam):
        """Depth frame of <cam> -> Nx3 world points (uses live TF), plus color image."""
        from sensor_msgs.msg import Image, CameraInfo
        from tf2_ros import Buffer, TransformListener
        from rclpy.time import Time
        from cv_bridge import CvBridge
        if not hasattr(self, "tfb"):
            self.tfb = Buffer(); self.tfl = TransformListener(self.tfb, self.n)
        depth = self.grab(f"/{cam}/depth/image_raw", Image)
        color = self.grab(f"/{cam}/color/image_raw", Image)
        info = self.grab(f"/{cam}/color/camera_info", CameraInfo)
        frame = f"{cam}_optical_frame"
        t0 = time.time()
        while not self.tfb.can_transform("world", frame, Time()) and time.time() - t0 < 10:
            self.spin(0.2)
        t = self.tfb.lookup_transform("world", frame, Time()).transform
        q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        T = np.array([t.translation.x, t.translation.y, t.translation.z])
        D = CvBridge().imgmsg_to_cv2(depth, "passthrough").astype(np.float64)
        C = CvBridge().imgmsg_to_cv2(color, "bgr8")
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        vs, us = np.mgrid[0:D.shape[0], 0:D.shape[1]]
        P = np.stack([(us - cx) * D / fx, (vs - cy) * D / fy, D], -1)
        W = P @ R.T + T
        return W, C, D

    def locate(self, cam, guess, radius=0.06, zmin=0.445, zmax=0.60):
        """Centroid (world xy) and top z of the blob above the table near guess."""
        W, C, D = self.cloud_world(cam)
        ok = np.isfinite(D) & (D > 0.05)
        near = (np.hypot(W[..., 0] - guess[0], W[..., 1] - guess[1]) < radius)
        m = ok & near & (W[..., 2] > zmin) & (W[..., 2] < zmax)
        if m.sum() < 10:
            print("  locate: nothing found", flush=True); return None
        pts = W[m]
        # use the top slice (cap) to get an unbiased xy centroid of a cylinder
        ztop = np.percentile(pts[:, 2], 98)
        cap = pts[pts[:, 2] > ztop - 0.015]
        c = cap.mean(0)
        ext = (pts[:, 0].max() - pts[:, 0].min(), pts[:, 1].max() - pts[:, 1].min())
        print(f"  locate {cam}: n={m.sum()} center=({c[0]:.4f},{c[1]:.4f}) ztop={ztop:.4f} extent=({ext[0]:.3f},{ext[1]:.3f})", flush=True)
        return c[0], c[1], ztop

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}", flush=True)
        return f
