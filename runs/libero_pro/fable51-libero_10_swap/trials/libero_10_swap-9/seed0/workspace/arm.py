#!/usr/bin/env python3
"""Small arm helper: FK/IK, trajectories, gripper, camera grabs. Reuses clients."""
import sys, time, math
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState, Image, CameraInfo
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE = np.zeros(3)  # FK/IK services already return/accept WORLD-frame poses (verified)
TCP = M["hand"]["tcp_offset_m"]
MAX_RATE = 0.2  # rad/s the FJT controller actually achieves (measured)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return np.array([w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2,
                     w1*w2 - x1*x2 - y1*y2 - z1*z2])


ROTZ45 = np.array([0.0, 0.0, math.sin(math.pi/8), math.cos(math.pi/8)])


def R_to_quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()  # x,y,z,w


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.n = rclpy.create_node("arm_helper")
        self.js = None
        self.n.create_subscription(JointState, "/joint_states", self._js, 1)
        self.fjt = ActionClient(self.n, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.n)
        self.fjt.wait_for_server(10); self.gr.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        while self.js is None:
            rclpy.spin_once(self.n, timeout_sec=0.2)

    def _js(self, m):
        self.js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.n, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js = None
            while self.js is None:
                self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in JOINTS]

    def fingers(self, fresh=True):
        if fresh:
            self.js = None
            while self.js is None:
                self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def seed_state(self, q):
        s = JointState(); s.name = list(JOINTS); s.position = [float(v) for v in q]
        return s

    def fk_hand(self, q=None):
        """Return (pos_world, quat) of panda_hand for joint vector q (default current)."""
        q = self.joints() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.seed_state(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if r is None else r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik_hand(self, pos_world, quat, seed=None, at_tcp=False, tries=1):
        """IK for panda_hand at world pose. If at_tcp, pos is the fingertip point."""
        pos = np.array(pos_world, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        pos_base = pos - BASE
        seed = self.joints() if seed is None else seed
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos_base)
            # IK targets panda_link8 = panda_hand rotated +45deg about hand Z (verified via FK)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, qmul(quat, ROTZ45))
            req.ik_request.robot_state.joint_state = self.seed_state(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 1
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            seed = list(np.array(seed) + np.random.uniform(-0.3, 0.3, len(seed)))
        raise RuntimeError(f"IK failed at {pos_world} (code {None if r is None else r.error_code.val})")

    def move(self, waypoints, durations):
        """waypoints: list of joint vectors; durations: cumulative time_from_start per point."""
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(JOINTS)
        # controller tracks at ~0.2 rad/s: stretch timing so each segment is feasible
        prev = np.array(self.joints()); tprev = 0.0; fixed = []
        for q, t in zip(waypoints, durations):
            need = np.max(np.abs(np.array(q) - prev)) / MAX_RATE + 0.3
            t = max(t, tprev + need)
            fixed.append(t); prev = np.array(q); tprev = t
        durations = fixed
        for q, t in zip(waypoints, durations):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            g.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, fut)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res)
        code = res.result().result.error_code
        err = np.max(np.abs(np.array(self.joints()) - np.array(waypoints[-1])))
        print(f"move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"gripper -> pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        got = {}
        sub = self.n.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("m", m), 1)
        while "m" not in got:
            self.spin(0.2)
        self.n.destroy_subscription(sub)
        out = out or f"{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got["m"], "bgr8"))
        return out

    def xyz_map(self, cam):
        """World-frame XYZ per pixel from the camera's depth."""
        from cv_bridge import CvBridge
        got = {}
        s1 = self.n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
        s2 = self.n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
        fr = f"{cam}_optical_frame"
        while len(got) < 2 or not self.tfbuf.can_transform("world", fr, Time()):
            self.spin(0.2)
        self.n.destroy_subscription(s1); self.n.destroy_subscription(s2)
        d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
        k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        H, W = d.shape
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
        t = self.tfbuf.lookup_transform("world", fr, Time())
        q = t.transform.rotation
        R = quat_to_R([q.x, q.y, q.z, q.w])
        tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        return pc @ R.T + tr

    def hand_tf(self):
        while not self.tfbuf.can_transform("world", "panda_hand", Time()):
            self.spin(0.2)
        t = self.tfbuf.lookup_transform("world", "panda_hand", Time())
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), np.array([q.x, q.y, q.z, q.w])


# handy orientations (hand frame -> world): quaternion x,y,z,w
Q_DOWN = np.array([1.0, 0.0, 0.0, 0.0])          # Z down, fingers along world y, hand X = +x
Q_FWD = np.array([0.0, 0.7071068, 0.0, 0.7071068])  # Z along +x (toward microwave), fingers along y


# ---------------- MoveIt planning with a populated scene ----------------
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (CollisionObject, PlanningScene, Constraints, JointConstraint,
                             PositionConstraint, OrientationConstraint, MotionPlanRequest, PlanningOptions)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def _box(name, center, size, frame="world"):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    sp = SolidPrimitive(); sp.type = SolidPrimitive.BOX; sp.dimensions = [float(s) for s in size]
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center); p.orientation.w = 1.0
    co.primitives = [sp]; co.primitive_poses = [p]
    co.operation = CollisionObject.ADD
    return co


class Planner:
    def __init__(self, arm):
        self.a = arm
        self.n = arm.n
        self.mg = ActionClient(self.n, MoveGroup, M["planning"]["move_action"])
        self.aps = self.n.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.mg.wait_for_server(10); self.aps.wait_for_service(10)

    def set_scene(self, boxes, remove=()):
        ps = PlanningScene(); ps.is_diff = True
        for name, center, size in boxes:
            ps.world.collision_objects.append(_box(name, center, size))
        for name in remove:
            co = CollisionObject(); co.id = name; co.header.frame_id = "world"
            co.operation = CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        req = ApplyPlanningScene.Request(); req.scene = ps
        fut = self.aps.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        print("scene applied:", fut.result().success, flush=True)

    def go_joints(self, q, vel=0.3, plan_only=False):
        g = MoveGroup.Goal()
        r = g.request
        r.group_name = M["planning"]["group"]
        r.num_planning_attempts = 10
        r.allowed_planning_time = 10.0
        r.max_velocity_scaling_factor = vel
        r.max_acceleration_scaling_factor = 0.3
        r.start_state.joint_state = self.a.seed_state(self.a.joints())
        r.start_state.is_diff = True
        c = Constraints()
        for j, v in zip(JOINTS, q):
            jc = JointConstraint(joint_name=j, position=float(v), tolerance_above=0.005,
                                 tolerance_below=0.005, weight=1.0)
            c.joint_constraints.append(jc)
        r.goal_constraints = [c]
        g.planning_options.plan_only = plan_only
        g.planning_options.replan = False
        fut = self.mg.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        gh = fut.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.n, res)
        rr = res.result().result
        self._last_traj = rr.planned_trajectory
        err = np.max(np.abs(np.array(self.a.joints()) - np.array(q)))
        print(f"plan/exec code={rr.error_code.val} max_joint_err={err:.4f} "
              f"npts={len(rr.planned_trajectory.joint_trajectory.points)}", flush=True)
        return rr.error_code.val, err

    def plan_exec(self, q, total_time=None, speed=0.6):
        """Plan collision-free path with MoveIt, then execute it through the FJT action.
        speed: rad/s bound used to set total duration (if total_time not given)."""
        code, _ = self.go_joints(q, plan_only=True)
        if code != 1:
            raise RuntimeError(f"planning failed code={code}")
        pts = self._last_traj.joint_trajectory.points
        wps = [list(p.positions) for p in pts]
        if len(wps) < 2:
            wps = [self.a.joints(), list(q)]
        # path length in joint space -> duration
        L = sum(np.max(np.abs(np.array(wps[i+1]) - np.array(wps[i]))) for i in range(len(wps)-1))
        T = total_time or max(2.0, L / speed)
        cum = [0.0]
        for i in range(len(wps)-1):
            cum.append(cum[-1] + np.max(np.abs(np.array(wps[i+1]) - np.array(wps[i]))))
        times = [0.5 + T * c / max(cum[-1], 1e-6) for c in cum]
        wps[-1] = list(q)
        return self.a.move(wps[1:], times[1:])


def look_quat(eye, target, up=(0, 0, 1)):
    """Hand quaternion with Z_h pointing from eye to target and Y_h horizontal."""
    z = np.array(target, float) - np.array(eye, float); z /= np.linalg.norm(z)
    y = np.cross(z, np.array(up, float));
    if np.linalg.norm(y) < 1e-6: y = np.array([0, 1.0, 0])
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return R_to_quat(np.stack([x, y, z], 1))


def attach_cylinder(pl, name, xyz_hand, quat_hand, height, radius, link="panda_hand"):
    """Attach a cylinder to the hand in the planning scene (pose in panda_hand frame)."""
    from moveit_msgs.msg import AttachedCollisionObject
    aco = AttachedCollisionObject()
    aco.link_name = link
    aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger", "panda_link8", "panda_link7"]
    co = CollisionObject(); co.header.frame_id = link; co.id = name
    sp = SolidPrimitive(); sp.type = SolidPrimitive.CYLINDER; sp.dimensions = [float(height), float(radius)]
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, xyz_hand)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_hand)
    co.primitives = [sp]; co.primitive_poses = [p]; co.operation = CollisionObject.ADD
    aco.object = co
    ps = PlanningScene(); ps.is_diff = True; ps.robot_state.is_diff = True
    ps.robot_state.attached_collision_objects = [aco]
    req = ApplyPlanningScene.Request(); req.scene = ps
    fut = pl.aps.call_async(req)
    rclpy.spin_until_future_complete(pl.n, fut, timeout_sec=30)
    print("attach applied:", fut.result().success, flush=True)


def detach_all(pl, name, link="panda_hand"):
    from moveit_msgs.msg import AttachedCollisionObject
    aco = AttachedCollisionObject(); aco.link_name = link
    aco.object.id = name; aco.object.operation = CollisionObject.REMOVE
    ps = PlanningScene(); ps.is_diff = True; ps.robot_state.is_diff = True
    ps.robot_state.attached_collision_objects = [aco]
    # also drop it from the world in case it got re-added there
    co = CollisionObject(); co.id = name; co.header.frame_id = "world"; co.operation = CollisionObject.REMOVE
    ps.world.collision_objects.append(co)
    req = ApplyPlanningScene.Request(); req.scene = ps
    fut = pl.aps.call_async(req)
    rclpy.spin_until_future_complete(pl.n, fut, timeout_sec=30)
    print("detach applied:", fut.result().success, flush=True)


class Validity:
    """Wrapper for /check_state_validity (collision + limits) for arm joint vectors."""
    def __init__(self, arm):
        from moveit_msgs.srv import GetStateValidity
        self.a = arm; self.n = arm.n
        self.srv = GetStateValidity
        self.cl = self.n.create_client(GetStateValidity, "/check_state_validity")
        self.cl.wait_for_service(10)

    def check(self, q, verbose=True):
        req = self.srv.Request()
        req.group_name = M["planning"]["group"]
        req.robot_state.joint_state = self.a.seed_state(q)
        req.robot_state.is_diff = True
        fut = self.cl.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        r = fut.result()
        if r is None:
            print("validity: no response", flush=True); return None, []
        pairs = [(c.contact_body_1, c.contact_body_2, round(c.depth, 4)) for c in r.contacts]
        if verbose:
            print("valid:", r.valid, pairs, flush=True)
        return r.valid, pairs
