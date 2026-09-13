#!/usr/bin/env python3
"""Small helper lib for this Panda: IK/FK (world frame, panda_hand tip),
trajectory + gripper actions, joint-state reads. Import or run as CLI:

  python3 rob.py ik  x y z qx qy qz qw      # solve only, print joints
  python3 rob.py go  x y z qx qy qz qw [sec] # IK then move
  python3 rob.py js                          # print joint state
  python3 rob.py fk                          # hand pose (world)
  python3 rob.py grip open|close
  python3 rob.py joints j1,...,j7 [sec]
"""
import sys, time, math
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import TwistStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE = np.array([0.0, 0.0, 0.0])  # FK/IK service already speaks world coords (verified vs TF)


def quat_from_R(R):
    # returns x,y,z,w
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def R_from_axes(zaxis, finger_axis):
    """Hand orientation: hand z (approach) = zaxis, hand y (finger travel)
    = finger_axis (made orthogonal). Returns 3x3."""
    z = np.array(zaxis, float); z /= np.linalg.norm(z)
    y = np.array(finger_axis, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def quat_from_axes(zaxis, finger_axis):
    return quat_from_R(R_from_axes(zaxis, finger_axis))


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time() * 1000) % 100000))
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        end = time.time() + 10
        while self._js is None and time.time() < end:
            self.spin(0.1)
        if self._js is None:
            raise RuntimeError("no joint_states")
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    # ---- kinematics (world frame in/out) ----
    def ik(self, pos_w, quat, seed=None, timeout=30, avoid=True):
        if not self.ik_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.array(pos_w, float) - BASE
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = avoid
        req.ik_request.timeout.sec = 1
        seed = seed if seed is not None else self.arm_q()
        js = JointState(); js.name = list(ARM); js.position = [float(v) for v in seed]
        req.ik_request.robot_state.joint_state = js
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def fk(self, q=None, link="panda_hand"):
        if not self.fk_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        q = q if q is not None else self.arm_q()
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res}")
        ps = res.pose_stamped[0].pose
        pos = np.array([ps.position.x, ps.position.y, ps.position.z]) + BASE
        quat = np.array([ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w])
        return pos, quat

    # ---- motion ----
    def move_joints(self, q, seconds=3.0, waypoints=None):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        h = send.result()
        if h is None or not h.accepted:
            raise RuntimeError("FJT goal rejected")
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = r.result.error_code if r else None
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def go(self, pos_w, quat, seconds=3.0, seed=None):
        q = self.ik(pos_w, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED for {np.round(pos_w,3)}", flush=True)
            return None
        code, err = self.move_joints(q, seconds)
        p, _ = self.fk()
        print(f"  hand now at {np.round(p,3)} (target {np.round(pos_w,3)})", flush=True)
        return q

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def servo(self, lin, ang=(0, 0, 0), n=20, frame="panda_link0"):
        msg = TwistStamped(); msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            self.twist_pub.publish(msg); self.spin(0.05)


if __name__ == "__main__":
    r = Robot()
    cmd = sys.argv[1]
    if cmd == "js":
        print(r.joints())
    elif cmd == "fk":
        p, q = r.fk(); print("pos", np.round(p, 4), "quat", np.round(q, 4))
    elif cmd in ("ik", "go"):
        v = list(map(float, sys.argv[2:9]))
        sec = float(sys.argv[9]) if len(sys.argv) > 9 else 3.0
        if cmd == "ik":
            q = r.ik(v[:3], v[3:])
            print("IK:", None if q is None else np.round(q, 4))
            if q is not None:
                print("FK check:", np.round(r.fk(q)[0], 4))
        else:
            r.go(v[:3], v[3:], sec)
    elif cmd == "grip":
        r.gripper(GRIP["open_m"] if sys.argv[2] == "open" else GRIP["closed_m"])
    elif cmd == "joints":
        q = list(map(float, sys.argv[2].split(",")))
        r.move_joints(q, float(sys.argv[3]) if len(sys.argv) > 3 else 3.0)
    rclpy.shutdown()


# ---- MoveIt plan+execute (collision-aware) ----
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions


def _plan_client(self):
    if not hasattr(self, "_mg"):
        self._mg = ActionClient(self.node, MoveGroup, M["planning"]["move_action"])
        if not self._mg.wait_for_server(timeout_sec=10):
            raise RuntimeError("no move_action")
    return self._mg


def plan_to_joints(self, q, vel=0.3, acc=0.3, tries=3, planning_time=5.0):
    """Collision-aware plan + execute to joint target q (manifest order)."""
    cli = _plan_client(self)
    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = M["planning"]["group"]
    req.num_planning_attempts = 5
    req.allowed_planning_time = planning_time
    req.max_velocity_scaling_factor = vel
    req.max_acceleration_scaling_factor = acc
    c = Constraints()
    for j, v in zip(ARM, q):
        jc = JointConstraint(joint_name=j, position=float(v), tolerance_above=0.005,
                             tolerance_below=0.005, weight=1.0)
        c.joint_constraints.append(jc)
    req.goal_constraints = [c]
    goal.request = req
    goal.planning_options = PlanningOptions(plan_only=False, replan=False)
    for t in range(tries):
        send = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        h = send.result()
        if h is None or not h.accepted:
            print("  move_group goal rejected", flush=True); continue
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        res = rf.result()
        code = res.result.error_code.val if res else None
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  plan+exec try{t} code={code} max_joint_err={err:.4f}", flush=True)
        if code == 1 and err < 0.02:
            return True
    return False


def plan_go(self, pos_w, quat, seed=None, **kw):
    q = self.ik(pos_w, quat, seed=seed)
    if q is None:
        print(f"  IK FAILED (collision-aware) for {np.round(pos_w,3)}", flush=True)
        return None
    ok = plan_to_joints(self, q, **kw)
    p, _ = self.fk()
    print(f"  hand now at {np.round(p,3)} (target {np.round(pos_w,3)}) ok={ok}", flush=True)
    return q if ok else None


Robot.plan_to_joints = plan_to_joints
Robot.plan_go = plan_go


def cart_path(self, wps, seconds_per_m=8.0, min_step_t=0.8, max_jump=0.6, avoid=False):
    """wps: list of (pos_w, quat). IK each with previous seed, then one FJT pass.
    Aborts (returns None) if any IK fails or a joint jumps > max_jump rad."""
    seed = self.arm_q()
    qs, ts, t = [], [], 0.0
    prev_p = self.fk()[0]
    for pos, quat in wps:
        q = self.ik(pos, quat, seed=seed, avoid=avoid)
        if q is None:
            print(f"  cart_path IK FAILED at {np.round(pos,3)}", flush=True); return None
        jump = np.abs(np.array(q) - np.array(seed)).max()
        if jump > max_jump:
            print(f"  cart_path joint jump {jump:.2f} at {np.round(pos,3)}", flush=True); return None
        d = np.linalg.norm(np.array(pos) - prev_p)
        t += max(min_step_t, d * seconds_per_m); ts.append(t); qs.append(q)
        seed, prev_p = q, np.array(pos, float)
    wl = list(zip(qs[:-1], ts[:-1]))
    code, err = self.move_joints(qs[-1], ts[-1], waypoints=wl)
    p, _ = self.fk()
    print(f"  hand now at {np.round(p,3)} (target {np.round(wps[-1][0],3)})", flush=True)
    return qs


Robot.cart_path = cart_path


def plan_exec(self, q, vel=0.3, acc=0.3, planning_time=5.0, time_scale=2.0, tries=2):
    """Plan only with MoveIt (collision-aware), then execute the returned joint
    trajectory ourselves through FJT with stretched timing (more robust than
    move_group's own execution, which returns -4 partway)."""
    cli = _plan_client(self)
    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = M["planning"]["group"]
    req.num_planning_attempts = 5
    req.allowed_planning_time = planning_time
    req.max_velocity_scaling_factor = vel
    req.max_acceleration_scaling_factor = acc
    c = Constraints()
    for j, v in zip(ARM, q):
        c.joint_constraints.append(JointConstraint(joint_name=j, position=float(v),
                                   tolerance_above=0.005, tolerance_below=0.005, weight=1.0))
    req.goal_constraints = [c]
    goal.request = req
    goal.planning_options = PlanningOptions(plan_only=True, replan=False)
    for t in range(tries):
        send = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        h = send.result()
        if h is None or not h.accepted:
            print("  move_group goal rejected", flush=True); continue
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result()
        code = res.result.error_code.val if res else None
        if code != 1:
            print(f"  plan try{t} code={code}", flush=True); continue
        jt = res.result.planned_trajectory.joint_trajectory
        idx = [jt.joint_names.index(j) for j in ARM]
        wps = []
        for pt in jt.points:
            tt = (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9) * time_scale
            wps.append(([pt.positions[i] for i in idx], tt))
        wps = [w for w in wps if w[1] > 0.05]
        print(f"  planned {len(jt.points)} pts, {wps[-1][1]:.1f}s", flush=True)
        code2, err = self.move_joints(wps[-1][0], wps[-1][1], waypoints=wps[:-1])
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  exec code={code2} max_joint_err={err:.4f}", flush=True)
        if err < 0.02:
            return True
    return False


def plan_go2(self, pos_w, quat, seed=None, **kw):
    q = self.ik(pos_w, quat, seed=seed)
    if q is None:
        print(f"  IK FAILED (collision-aware) for {np.round(pos_w,3)}", flush=True)
        return None
    ok = plan_exec(self, q, **kw)
    p, _ = self.fk()
    print(f"  hand now at {np.round(p,3)} (target {np.round(pos_w,3)}) ok={ok}", flush=True)
    return q if ok else None


Robot.plan_exec = plan_exec
Robot.plan_go2 = plan_go2
