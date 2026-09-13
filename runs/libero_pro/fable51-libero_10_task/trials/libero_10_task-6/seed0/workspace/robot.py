"""Reusable control layer for this Panda (long-lived node, clients built once)."""
import time, math, yaml, numpy as np, rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, TwistStamped
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE = np.array([0.0, 0.0, 0.0])   # panda_link0 in world (from TF)
TCP = M["hand"]["tcp_offset_m"]

class Robot:
    def __init__(self, name="ctl"):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None; self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 10)
        self.node.create_subscription(WrenchStamped, M["sensors"][1]["port"], self._wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)
        self.spin(0.5)
        while self.js is None: self.spin(0.2)

    def _js(self, m): self.js = m
    def _wr(self, m): self.wr = m
    def spin(self, t):
        end = time.time() + t
        while time.time() < end: rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---- sensing
    def q(self):
        self.spin(0.1)
        d = dict(zip(self.js.name, self.js.position))
        return np.array([d[j] for j in ARM])
    def fingers(self):
        self.spin(0.1)
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]
    def wrench(self):
        self.spin(0.1)
        if self.wr is None: return None
        f = self.wr.wrench.force; t = self.wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])
    def fk(self, q=None):
        """hand pose in WORLD: (pos xyz, quat xyzw)"""
        if q is None: q = self.q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1: raise RuntimeError(f"FK failed {r}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = Rot.from_quat(quat).as_matrix()
        return pos + TCP * R[:, 2], quat

    # ---- planning
    def ik(self, pos_world, quat, seed=None, at_tcp=True, tries=3):
        pos = np.array(pos_world, float)
        R = Rot.from_quat(quat).as_matrix()
        if at_tcp: pos = pos - TCP * R[:, 2]
        pb = pos - BASE
        if seed is None: seed = self.q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pb)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return np.array([sol[j] for j in ARM])
        return None

    # ---- acting
    def move(self, q_target, seconds=3.0, via=(), retries=2):
        for i in range(retries + 1):
            code, err = self._move_once(q_target, seconds, via)
            if code == 0 and err < 0.01: return code, err
            print(f"  (retry {i+1}: code={code} err={err:.4f})"); via = (); seconds = max(seconds, 3.0)
        return code, err
    def _move_once(self, q_target, seconds, via):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = list(via) + [q_target]
        for i, q in enumerate(pts):
            t = seconds * (i + 1) / len(pts)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = self.settle(q_target)
        print(f"  move: code={code} max_joint_err={err:.4f}")
        return code, err
    def settle(self, q_target, tol=0.004, budget=40.0):
        """poll joint state until converged to q_target or stalled (no progress for ~2s)"""
        import time
        t0 = time.time(); last = None; still = 0
        while time.time() - t0 < budget:
            e = np.abs(self.q() - np.array(q_target)).max()
            if e < tol: return e
            if last is not None and abs(last - e) < 1e-4:
                still += 1
                if still >= 8: return e   # stalled
            else: still = 0
            last = e; self.spin(0.25)
        return e
    def goto(self, pos_world, quat, seconds=3.0, seed=None, at_tcp=True):
        q = self.ik(pos_world, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            print("  IK FAILED for", np.round(pos_world, 3)); return None
        self.move(q, seconds)
        tp, _ = self.tcp()
        print(f"  tcp now {np.round(tp,4)} (target {np.round(pos_world,4)})")
        return q
    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        self.spin(0.3)
        print(f"  gripper: reached={r.reached_goal} stalled={r.stalled} fingers={np.round(self.fingers(),4)}")
        return r
    def servo(self, v, n=20, frame=None):
        """stream twist (linear m/s in base frame) for n ticks"""
        msg = TwistStamped(); msg.header.frame_id = frame or TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            self.tw_pub.publish(msg); rclpy.spin_once(self.node, timeout_sec=0.05)

def down_quat(yaw_deg=0.0):
    """hand z pointing down (world -z); yaw rotates fingers' closing axis: 0 -> fingers close along world y"""
    return (Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])).as_quat()
