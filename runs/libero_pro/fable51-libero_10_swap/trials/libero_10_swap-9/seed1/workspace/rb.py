#!/usr/bin/env python3
"""Reusable robot helper: one node, persistent clients.

import rb; R = rb.Robot()
R.joints() -> dict ; R.fk() -> (pos_world, quat) ; R.ik(pos_world, quat) -> [7]
R.move(positions, secs) ; R.move_path([[7]...], secs_each) ; R.grip(width)
R.snap(cam, out) ; R.cloud(cam) -> (H,W,3) world xyz
World<->base: base at (-0.66, 0, 0.912) with identity rotation.
"""
import struct
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE = np.zeros(3)  # FK/IK services already work in the world frame (verified)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()  # x,y,z,w


def frame_quat(hand_z, hand_y):
    """Quaternion for a hand frame whose z (approach) and y (finger) axes
    are the given world vectors."""
    z = np.array(hand_z, float); z /= np.linalg.norm(z)
    y = np.array(hand_y, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return R_to_quat(np.stack([x, y, z], 1))


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rb_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped,
                                                    "/servo_node/delta_twist_cmds", 10)
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(10); self.gr.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t = time.time()
        while "m" not in self._js and time.time() - t < 20:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """World position + quaternion of a link (default hand frame)."""
        q = self.arm_q() if q is None else list(q)
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
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y,
                              p.orientation.z, p.orientation.w])

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = quat_to_R(quat)
        return pos + R[:, 2] * M["hand"]["tcp_offset_m"], quat

    def ik(self, pos_world, quat, seed=None, tcp=True, attempts=3):
        """Joint solution for hand (or TCP if tcp=True) at a world pose."""
        pos = np.array(pos_world, float)
        if tcp:
            pos = pos - quat_to_R(quat)[:, 2] * M["hand"]["tcp_offset_m"]
        pos = pos - BASE
        # IK tip link is panda_link8 = hand rotated +45deg about hand z
        from scipy.spatial.transform import Rotation as Rot
        quat = (Rot.from_quat(quat) * Rot.from_rotvec([0, 0, np.pi / 4])).as_quat()
        seed0 = self.arm_q() if seed is None else list(seed)
        rng = np.random.default_rng(0)
        lim = np.array(FJT["limits_rad"])
        last = None
        for k in range(attempts):
            seed = seed0 if k == 0 else list(rng.uniform(lim[:, 0], lim[:, 1]))
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            last = res.error_code.val if res else None
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        raise RuntimeError(f"IK failed code={last} for {pos_world}")

    # ---------------- acting ----------------
    def move_path(self, qs, secs_each=2.0, first_secs=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        t = 0.0
        for i, q in enumerate(qs):
            t += (first_secs if (i == 0 and first_secs) else secs_each)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        q = np.array(self.arm_q()); err = np.abs(q - np.array(qs[-1])).max()
        print(f"move: code={code} max_joint_err={err:.4f}")
        return code, err

    def move(self, q, secs=3.0, tol=0.01, retries=2):
        code, err = self.move_path([q], secs)
        while err > tol and retries > 0:   # controller lag: resend converges
            retries -= 1
            code, err = self.move_path([q], max(2.0, secs / 2))
        return code, err

    def grip(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"grip({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def servo(self, lin, n=20, frame="panda_link0"):
        msg = TwistStamped(); msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        for _ in range(n):
            self.twist_pub.publish(msg); self.spin(0.05)

    # ---------------- cameras ----------------
    def _grab(self, topic, T, timeout=60):
        got = {}
        sub = self.node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
        t = time.time()
        while "m" not in got and time.time() - t < timeout:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        return got.get("m")

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        m = self._grab(f"/{cam}/color/image_raw", Image)
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(m, "bgr8"))
        return out

    def cloud(self, cam):
        d = self._grab(f"/{cam}/depth/image_raw", Image)
        info = self._grab(f"/{cam}/color/camera_info", CameraInfo)
        depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        frame = f"{cam}_optical_frame"
        t = time.time()
        while time.time() - t < 20 and not self.tfbuf.can_transform("world", frame, rclpy.time.Time()):
            self.spin(0.2)
        tr = self.tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = tr.transform.rotation
        R = quat_to_R([q.x, q.y, q.z, q.w])
        t3 = np.array([tr.transform.translation.x, tr.transform.translation.y,
                       tr.transform.translation.z])
        v, u = np.mgrid[0:d.height, 0:d.width]
        P = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1) @ R.T + t3
        return P


# ---------------- planning scene helpers ----------------
def _box(cid, center, size, yaw=0.0, frame="world"):
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    co = CollisionObject(); co.header.frame_id = frame; co.id = cid
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
    pose = Pose(); pose.position.x, pose.position.y, pose.position.z = map(float, center)
    pose.orientation.z = float(np.sin(yaw / 2)); pose.orientation.w = float(np.cos(yaw / 2))
    co.primitives.append(prim); co.primitive_poses.append(pose); co.operation = CollisionObject.ADD
    return co


def apply_scene(R, objs):
    from moveit_msgs.srv import ApplyPlanningScene
    cli = R.node.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(10)
    req = ApplyPlanningScene.Request(); req.scene.is_diff = True
    req.scene.world.collision_objects = objs
    fut = cli.call_async(req); rclpy.spin_until_future_complete(R.node, fut, timeout_sec=30)
    return fut.result().success


def ik_free(R, pos, quat, seed=None, tcp=True, attempts=3):
    """IK that must be collision-free against the planning scene; returns None if impossible."""
    from scipy.spatial.transform import Rotation as Rot
    p = np.array(pos, float)
    if tcp:
        p = p - quat_to_R(quat)[:, 2] * M["hand"]["tcp_offset_m"]
    q8 = (Rot.from_quat(quat) * Rot.from_rotvec([0, 0, np.pi / 4])).as_quat()
    seed0 = R.arm_q() if seed is None else list(seed)
    rng = np.random.default_rng(1); lim = np.array(FJT["limits_rad"]); last = None
    for k in range(attempts):
        sd = seed0 if k == 0 else list(rng.uniform(lim[:, 0], lim[:, 1]))
        req = GetPositionIK.Request(); req.ik_request.group_name = M["planning"]["group"]
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, p)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q8)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in sd]
        req.ik_request.avoid_collisions = True; req.ik_request.timeout.sec = 2
        fut = R.ik_cli.call_async(req); rclpy.spin_until_future_complete(R.node, fut, timeout_sec=60)
        res = fut.result(); last = res.error_code.val if res else None
        if res is not None and res.error_code.val == 1:
            sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            return [sol[j] for j in ARM]
    print("ik_free failed", last, np.round(pos, 3)); return None


def state_valid(R, q):
    """Collision check of an arm configuration via /check_state_validity if present."""
    from moveit_msgs.srv import GetStateValidity
    if not hasattr(R, "_sv"):
        R._sv = R.node.create_client(GetStateValidity, "/check_state_validity")
        if not R._sv.wait_for_service(5):
            return None
    req = GetStateValidity.Request(); req.group_name = M["planning"]["group"]
    req.robot_state.joint_state.name = list(ARM); req.robot_state.joint_state.position = [float(v) for v in q]
    fut = R._sv.call_async(req); rclpy.spin_until_future_complete(R.node, fut, timeout_sec=30)
    r = fut.result()
    return r.valid, [(c.contact_body_1, c.contact_body_2) for c in r.contacts]


def attach_box(R, cid, size, pose_in_hand, link="panda_hand", remove=False):
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    from moveit_msgs.srv import ApplyPlanningScene
    aco = AttachedCollisionObject(); aco.link_name = link
    aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger", "panda_link8", "panda_link7"]
    co = aco.object; co.header.frame_id = link; co.id = cid
    if remove:
        co.operation = CollisionObject.REMOVE
    else:
        co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size]))
        p = Pose(); p.position.x, p.position.y, p.position.z = map(float, pose_in_hand); p.orientation.w = 1.0
        co.primitive_poses.append(p); co.operation = CollisionObject.ADD
    cli = R.node.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(10)
    req = ApplyPlanningScene.Request(); req.scene.is_diff = True
    req.scene.robot_state.is_diff = True
    req.scene.robot_state.attached_collision_objects.append(aco)
    if remove:
        # also drop it from the world in case it was detached there
        w = CollisionObject(); w.id = cid; w.header.frame_id = "world"; w.operation = CollisionObject.REMOVE
        req.scene.world.collision_objects.append(w)
    fut = cli.call_async(req); rclpy.spin_until_future_complete(R.node, fut, timeout_sec=30)
    return fut.result().success


def plan_to_joints(R, q_goal, secs=10.0, attempts=5, vel=0.3):
    """MoveIt plan-only to a joint goal from the current state. Returns list of (t, positions)."""
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint
    if not hasattr(R, "_mg"):
        R._mg = ActionClient(R.node, MoveGroup, M["planning"]["move_action"]); R._mg.wait_for_server(10)
    g = MoveGroup.Goal(); r = g.request
    r.group_name = M["planning"]["group"]; r.allowed_planning_time = float(secs)
    r.num_planning_attempts = int(attempts); r.max_velocity_scaling_factor = float(vel)
    r.max_acceleration_scaling_factor = float(vel)
    r.start_state.is_diff = True
    c = Constraints()
    for n, v in zip(ARM, q_goal):
        c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v),
                                                   tolerance_above=0.005, tolerance_below=0.005, weight=1.0))
    r.goal_constraints.append(c)
    g.planning_options.plan_only = True
    fut = R._mg.send_goal_async(g); rclpy.spin_until_future_complete(R.node, fut, timeout_sec=60)
    rf = fut.result().get_result_async(); rclpy.spin_until_future_complete(R.node, rf, timeout_sec=120)
    res = rf.result().result
    if res.error_code.val != 1:
        print("plan failed", res.error_code.val); return None
    traj = res.planned_trajectory.joint_trajectory
    idx = [traj.joint_names.index(n) for n in ARM]
    return [(pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9, [pt.positions[i] for i in idx])
            for pt in traj.points]


def exec_plan(R, plan, tscale=1.5, min_dt=0.3):
    """Execute a planned trajectory (sparsified) with the FJT client, then converge on the final point."""
    pts = [p for p in plan]
    goal_qs = []; last_t = -1
    for t, q in pts:
        if t - last_t >= min_dt or (t, q) == pts[-1]:
            goal_qs.append((t, q)); last_t = t
    from control_msgs.action import FollowJointTrajectory
    gl = FollowJointTrajectory.Goal(); gl.trajectory.joint_names = list(ARM)
    for t, q in goal_qs:
        tt = max(0.5, t * tscale)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(tt), nanosec=int((tt % 1) * 1e9)); gl.trajectory.points.append(pt)
    send = R.fjt.send_goal_async(gl); rclpy.spin_until_future_complete(R.node, send, timeout_sec=60)
    res = send.result().get_result_async(); rclpy.spin_until_future_complete(R.node, res, timeout_sec=600)
    qf = np.array(goal_qs[-1][1]); err = np.abs(np.array(R.arm_q()) - qf).max()
    print(f"exec_plan: {len(goal_qs)} pts, code={res.result().result.error_code}, err={err:.4f}")
    if err > 0.01:
        R.move(list(qf), 3.0)
    return np.abs(np.array(R.arm_q()) - qf).max()


def cart_path(R, targets, quat, step=0.02, maxdq=0.5, seed=None, validate=True):
    """Straight-line TCP waypoints (world) -> continuous joint path, collision-validated. None on failure."""
    q = list(R.arm_q() if seed is None else seed)
    p0 = np.array(R.tcp(q)[0]); qs = []
    for tgt in targets:
        tgt = np.array(tgt, float); n = max(1, int(np.ceil(np.linalg.norm(tgt - p0) / step)))
        for i in range(1, n + 1):
            wp = p0 + (tgt - p0) * i / n
            try:
                s = R.ik(wp, quat, seed=q, attempts=1)
            except RuntimeError:
                print("cart_path: IK fail at", np.round(wp, 3)); return None
            dq = np.abs(np.array(s) - np.array(q)).max()
            if dq > maxdq:
                print("cart_path: branch jump", round(dq, 2), "at", np.round(wp, 3)); return None
            v = state_valid(R, s) if validate else None
            if v is not None and not v[0]:
                print("cart_path: collision at", np.round(wp, 3), v[1]); return None
            qs.append(s); q = s
        p0 = tgt
    return qs


def pose_path(R, poses, maxdq=0.5, seed=None, validate=True, verbose=False):
    """List of (pos, quat) TCP poses -> continuous joint path (seeded IK per pose). None on failure."""
    q = list(R.arm_q() if seed is None else seed); qs = []
    for i, (p, quat) in enumerate(poses):
        try:
            s = R.ik(np.array(p, float), quat, seed=q, attempts=1)
        except RuntimeError:
            print("pose_path: IK fail at", i, np.round(p, 3)); return None
        dq = np.abs(np.array(s) - np.array(q)).max()
        if dq > maxdq:
            print("pose_path: branch jump", round(dq, 2), "at", i, np.round(p, 3)); return None
        v = state_valid(R, s) if validate else None
        if v is not None and not v[0]:
            print("pose_path: collision at", i, np.round(p, 3), v[1]); return None
        if verbose: print(i, np.round(p, 3), "dq", round(dq, 3))
        qs.append(s); q = s
    return qs
