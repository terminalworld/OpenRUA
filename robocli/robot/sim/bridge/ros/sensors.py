"""Publish side of the bridge: the robot's standard observation surface.

Graph shape copies franka_ros2 / standard camera-driver naming (zero
invention): ``/clock``, ``/joint_states`` (effort from sim torques),
per-camera ``/<name>/color/image_raw`` (rgb8) + ``/<name>/depth/image_raw``
(32FC1, metric) + ``/<name>/color/camera_info`` (K via robosuite's own
intrinsics helper), TF (world -> base/hand/camera optical frames; the
eye-in-hand frame follows the arm because poses are read from the sim at
publish time), and the estimated external wrench (``WrenchStamped``).

Privileged object poses are NEVER published; the no-perception tier is
constructively deleted here (the env's ``*_pos``/``*_quat`` obs keys stay
inside this process).

Pull-style observation in a paused world: cameras render on their own
wall-clock timer (rendering is expensive, ~65 ms/frame under llvmpipe) and
NOTHING here ever steps the sim.
"""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import TransformStamped, WrenchStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import TransformBroadcaster

from .clock import sim_time_msg
from .joints import build_joint_map

# mujoco cameras look along -Z; ROS optical frames look along +Z
# (REP 103/104): rotate pi about X to convert.
_MJ2OPTICAL = np.diag([1.0, -1.0, -1.0])


def _mat_to_quat(m: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix -> (x, y, z, w)."""
    t = np.trace(m)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        return ((m[2, 1] - m[1, 2]) * s, (m[0, 2] - m[2, 0]) * s,
                (m[1, 0] - m[0, 1]) * s, 0.25 / s)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(max(1e-12, 1.0 + m[i, i] - m[j, j] - m[k, k]))
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[3] = (m[k, j] - m[j, k]) / s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return q[0], q[1], q[2], q[3]


class SensorPublishers:
    def __init__(self, node, env, cfg: dict, sim=None):
        self._node = node
        self._env = env
        self._runner = sim  # Worker: all sim access goes through it
        self._pending_state = False
        self._pending_cams = False
        self._sim = env.sim
        self._raw = env.env if hasattr(env, "env") else env  # robosuite env

        self._joint_map = build_joint_map(env, cfg)
        # With the MoveIt stack up (mainline), robot_state_publisher owns
        # the arm-chain TF from /joint_states + URDF; the bridge then only
        # publishes world->panda_link0 and camera frames (TF single-parent
        # rule). Set tf.hand: true to let the bridge publish the hand frame
        # in MoveIt-less runs.
        machine = cfg.get("machine", {})
        self._publish_hand_tf = bool(machine.get("tf", {}).get("hand", False))
        self._clock_pub = node.create_publisher(Clock, "/clock", 10)
        self._joint_pub = node.create_publisher(JointState, "/joint_states", 10)
        self._tf = TransformBroadcaster(node)
        # Per-arm view (run.normalize_arms materialized it): each arm has
        # its own TF root, hand frame, and wrench stream.
        from .arms import arm_specs
        self._arm_specs = arm_specs(machine)
        self._wrench_pubs = [
            (i, arm, node.create_publisher(
                WrenchStamped, arm["ports"]["wrench"], 10))
            for i, arm in enumerate(self._arm_specs)
            if arm.get("ports", {}).get("wrench")
        ]

        # Mobile base (robocasa leg): odometry from the base body's sim
        # pose; the paused-clock analog of wheel odometry (drift-free; a
        # disclosed simulation simplification, same tier as joint_states).
        odom_port = machine.get("ports", {}).get("odom")
        base_body = machine.get("base", {}).get("body")
        self._odom_pub = None
        if odom_port and base_body:
            from nav_msgs.msg import Odometry

            self._odom_body = base_body
            self._odom_frame = machine.get("base", {}).get(
                "frame", "base_footprint"
            )
            self._odom_pub = node.create_publisher(Odometry, odom_port, 10)

        # Cameras: copy the scene's own list unless the config narrows it.
        cam_cfg = machine.get("cameras", {})
        res = cam_cfg.get("resolution", [640, 480])
        self._cam_w, self._cam_h = int(res[0]), int(res[1])
        names = cam_cfg.get("list")
        if not isinstance(names, list):
            names = list(self._sim.model.camera_names)
        self._cams = {}
        for name in names:
            self._cams[name] = {
                "color": node.create_publisher(Image, f"/{name}/color/image_raw", 1),
                "depth": node.create_publisher(Image, f"/{name}/depth/image_raw", 1),
                "info": node.create_publisher(CameraInfo, f"/{name}/color/camera_info", 1),
            }
        # on_demand (default): render a camera only while someone is
        # subscribed to its color/depth stream. Agent-indistinguishable
        # from a always-streaming device; subscribing IS how anyone
        # looks, and the next timer tick (<= one period) delivers; real
        # cameras have the same connect latency. Saves llvmpipe whole-
        # room renders nobody is watching (2026-08-13, Zhaoyang).
        self._render_mode = str(cam_cfg.get("render_mode", "on_demand"))
        rate = float(cam_cfg.get("rate_hz", 2.0))
        if self._cams and rate > 0:
            node.create_timer(1.0 / rate, self.publish_cameras)

    # ------------------------------------------------------------------ state
    def publish(self) -> None:
        """Publish current cheap state (joints/clock/TF/wrench). Called after
        every step AND on the wall-clock republish timer; never steps.
        Cross-thread calls become sim-thread jobs (deduplicated)."""
        if self._pending_state:
            return
        self._pending_state = True
        self._runner.submit(self._publish_state_job, wait=False)

    def _publish_state_job(self) -> None:
        self._pending_state = False
        js = JointState()
        tfs = []

        sim = self._env.sim
        now = sim_time_msg(float(sim.data.time))
        qfrc = sim.data.qfrc_actuator
        for name, qadr, dadr in self._joint_map:
            js.name.append(name)
            js.position.append(float(sim.data.qpos[qadr]))
            js.velocity.append(float(sim.data.qvel[dadr]))
            js.effort.append(float(qfrc[dadr]))

        # Arm-root TF source per arm: "robot0_base" on fixed-base
        # assemblies; on mobile ones that body is a static dummy; the
        # true arm root (robot0_link0) rides the base (robocasa leg sets
        # tf.base_body, which normalize_arms folds into the arm spec).
        for arm in self._arm_specs:
            tfs.append(self._tf_from_body(
                now, arm.get("tf_base_body", "robot0_base"),
                arm.get("base_frame", "panda_link0")))
            if self._publish_hand_tf:
                tfs.append(self._tf_from_body(
                    now, arm.get("hand_body", "robot0_right_hand"),
                    arm.get("hand_frame", "panda_hand")))
        for cam in self._cams:
            tfs.append(self._tf_from_camera(now, cam))

        wrenches = []
        for i, arm, pub in self._wrench_pubs:
            robot = self._raw.robots[i]
            f = getattr(robot, "ee_force", np.zeros(3))
            t = getattr(robot, "ee_torque", np.zeros(3))
            if isinstance(f, dict):  # robosuite >= 1.5: keyed by arm name
                f = next(iter(f.values()))
            if isinstance(t, dict):
                t = next(iter(t.values()))
            wr = WrenchStamped()
            wr.wrench.force.x, wr.wrench.force.y, wr.wrench.force.z = \
                map(float, f)
            wr.wrench.torque.x, wr.wrench.torque.y, wr.wrench.torque.z = \
                map(float, t)
            wr.header.frame_id = arm.get("base_frame", "panda_link0")
            wrenches.append((wr, pub))

        if self._odom_pub is not None:
            from nav_msgs.msg import Odometry

            od = Odometry()
            od.header.stamp = now
            od.header.frame_id = "world"
            od.child_frame_id = self._odom_frame
            bid = sim.model.body_name2id(self._odom_body)
            px, py, pz = (float(v) for v in sim.data.body_xpos[bid])
            qw, qx, qy, qz = (float(v) for v in sim.data.body_xquat[bid])
            od.pose.pose.position.x = px
            od.pose.pose.position.y = py
            od.pose.pose.position.z = pz
            od.pose.pose.orientation.x = qx
            od.pose.pose.orientation.y = qy
            od.pose.pose.orientation.z = qz
            od.pose.pose.orientation.w = qw
            self._odom_pub.publish(od)
            tfs.append(self._tf_from_body(now, self._odom_body, self._odom_frame))

        clk = Clock()
        clk.clock = now
        self._clock_pub.publish(clk)
        js.header.stamp = now
        self._joint_pub.publish(js)
        self._tf.sendTransform([t for t in tfs if t is not None])
        for wr, pub in wrenches:
            wr.header.stamp = now
            pub.publish(wr)

    def _tf_from_body(self, now, body: str, child: str) -> TransformStamped | None:
        sim = self._env.sim
        try:
            bid = sim.model.body_name2id(body)
        except Exception:  # noqa: BLE001
            return None
        pos = sim.data.body_xpos[bid]
        mat = sim.data.body_xmat[bid].reshape(3, 3)
        return self._make_tf(now, "world", child, pos, mat)

    def _tf_from_camera(self, now, cam: str) -> TransformStamped | None:
        sim = self._env.sim
        try:
            cid = sim.model.camera_name2id(cam)
        except Exception:  # noqa: BLE001
            return None
        pos = sim.data.cam_xpos[cid]
        mat = sim.data.cam_xmat[cid].reshape(3, 3) @ _MJ2OPTICAL
        return self._make_tf(now, "world", f"{cam}_optical_frame", pos, mat)

    @staticmethod
    def _make_tf(now, parent, child, pos, mat) -> TransformStamped:
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = float(pos[0])
        tf.transform.translation.y = float(pos[1])
        tf.transform.translation.z = float(pos[2])
        x, y, z, w = _mat_to_quat(np.asarray(mat, dtype=float))
        tf.transform.rotation.x = float(x)
        tf.transform.rotation.y = float(y)
        tf.transform.rotation.z = float(z)
        tf.transform.rotation.w = float(w)
        return tf

    # ---------------------------------------------------------------- cameras
    def publish_cameras(self) -> None:
        """Render + publish frames (camera timer; never steps). Rendering
        MUST happen on the sim thread; the EGL context is thread-affine."""
        if self._pending_cams:
            return
        self._pending_cams = True
        self._runner.submit(self._publish_cameras_job, wait=False)

    def _publish_info_only(self, name: str, pubs: dict) -> None:
        """CameraInfo stays on air for unwatched cameras (tools read the
        intrinsics before deciding to subscribe); costs no render."""
        from robosuite.utils.camera_utils import get_camera_intrinsic_matrix

        sim = self._env.sim
        k = get_camera_intrinsic_matrix(sim, name, self._cam_h, self._cam_w)
        info = CameraInfo()
        info.header.stamp = sim_time_msg(float(sim.data.time))
        info.header.frame_id = f"{name}_optical_frame"
        info.height, info.width = self._cam_h, self._cam_w
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [float(v) for v in np.asarray(k).flatten()]
        info.p = [
            info.k[0], 0.0, info.k[2], 0.0,
            0.0, info.k[4], info.k[5], 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        pubs["info"].publish(info)

    def _publish_cameras_job(self) -> None:
        self._pending_cams = False
        # Commands outrank observation: on whole-room scenes a render
        # cycle costs ~0.2s x cameras (llvmpipe) and can starve command
        # consumption into backlog (2026-08-12 composite canaries). If
        # jobs are waiting, skip this cycle; the camera timer retries at
        # the next period; effective frame rate degrades under load
        # instead of the robot's responsiveness.
        if self._runner.backlog() > 0:
            return
        from robosuite.utils.camera_utils import (
            get_camera_intrinsic_matrix,
            get_real_depth_map,
        )

        for name, pubs in self._cams.items():
            if (self._render_mode == "on_demand"
                    and pubs["color"].get_subscription_count() == 0
                    and pubs["depth"].get_subscription_count() == 0):
                self._publish_info_only(name, pubs)
                continue
            sim = self._env.sim
            now = sim_time_msg(float(sim.data.time))
            rgb, depth = sim.render(
                width=self._cam_w,
                height=self._cam_h,
                camera_name=name,
                depth=True,
            )
            rgb = np.flipud(rgb).copy()
            depth = np.flipud(get_real_depth_map(sim, depth)).astype(np.float32)
            k = get_camera_intrinsic_matrix(sim, name, self._cam_h, self._cam_w)

            frame_id = f"{name}_optical_frame"
            img = Image()
            img.header.stamp = now
            img.header.frame_id = frame_id
            img.height, img.width = self._cam_h, self._cam_w
            img.encoding = "rgb8"
            img.step = self._cam_w * 3
            img.data = rgb.astype(np.uint8).tobytes()
            pubs["color"].publish(img)

            dimg = Image()
            dimg.header.stamp = now
            dimg.header.frame_id = frame_id
            dimg.height, dimg.width = self._cam_h, self._cam_w
            dimg.encoding = "32FC1"
            dimg.step = self._cam_w * 4
            dimg.data = depth.tobytes()
            pubs["depth"].publish(dimg)

            info = CameraInfo()
            info.header.stamp = now
            info.header.frame_id = frame_id
            info.height, info.width = self._cam_h, self._cam_w
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            info.k = [float(v) for v in np.asarray(k).flatten()]
            info.p = [
                info.k[0], 0.0, info.k[2], 0.0,
                0.0, info.k[4], info.k[5], 0.0,
                0.0, 0.0, 1.0, 0.0,
            ]
            pubs["info"].publish(info)
