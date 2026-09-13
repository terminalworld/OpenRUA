"""Reusable robot helpers for this Panda workstation (clients built once per process)."""
import time, math, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import TwistStamped, WrenchStamped
from tf2_msgs.msg import TFMessage

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.zeros(3)   # verified: MoveIt FK/IK poses are already in world coords
TCP_OFF = float(M["hand"]["tcp_offset_m"])

def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])

def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)

# panda_hand = panda_link8 * Rz(-45deg); the IK service's tip is panda_link8,
# so a desired HAND orientation must be sent as q_hand * Rz(+45deg)
Q_HAND_TO_LINK8 = (0.0, 0.0, math.sin(math.pi/8), math.cos(math.pi/8))

def topdown_quat(yaw):
    """Hand z pointing down (world -z); hand x rotated by `yaw` about world z.
    yaw=0 -> fingers close along world y."""
    # R = Rz(yaw) * Rx(pi)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    # q_z(yaw) * q_x(pi) with q_x(pi) = (1,0,0,0)
    qz = (0, 0, sy, cy)
    # quaternion product qz * qx
    x1, y1, z1, w1 = qz; x2, y2, z2, w2 = (1, 0, 0, 0)
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)

class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.n = rclpy.create_node(name)
        self.js = {}
        self.n.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.fjt = ActionClient(self.n, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.n.create_publisher(TwistStamped, TW["port"], 10)
        self.wrench = {}
        self.n.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
                                   lambda m: self.wrench.update(m=m), 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.spin(0.5)

    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def joints(self):
        self.js = {}
        while len(self.js) < 7:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        return [self.js[j] for j in JOINTS]

    def fingers(self):
        self.joints()
        return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")

    def _arm_state(self):
        q = self.joints()
        s = JointState(); s.name = list(JOINTS); s.position = list(q)
        return s

    def hand_pose_world(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        s = self._arm_state()
        if q is not None:
            s.position = [float(v) for v in q]
        req.robot_state.joint_state = s
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), r.pose_stamped[0].header.frame_id

    def tcp_pose_world(self, joints=None):
        pos, q, f = self.hand_pose_world(joints)
        R = quat_to_R(*q)
        return pos + TCP_OFF * R[:, 2], q

    def ik_world(self, pos_world, quat, at_tcp=True, seed=None):
        pos = np.array(pos_world, float)
        if at_tcp:
            pos = pos - TCP_OFF * quat_to_R(*quat)[:, 2]
        pos = pos - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        q8 = qmul(quat, Q_HAND_TO_LINK8)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
        s = self._arm_state()
        if seed is not None:
            s.position = list(seed)
        req.ik_request.robot_state.joint_state = s
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move_joints(self, targets, seconds, waypoints=None):
        """targets: list of 7; waypoints: optional list of (positions, t)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        for pos, t in (waypoints or []) + [(targets, seconds)]:
            pt = JointTrajectoryPoint(positions=[float(x) for x in pos])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res, timeout_sec=600)
        code = res.result().result.error_code
        self.spin(0.3)
        q = self.joints()
        err = max(abs(a - b) for a, b in zip(q, targets))
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik_world(pos_world, quat, at_tcp=True, seed=seed)
        if q is None:
            print(f"  IK FAILED for tcp {np.round(pos_world,3)}")
            return None
        # sanity: joint limits
        for v, (lo, hi) in zip(q, FJT["limits_rad"]):
            if not lo <= v <= hi:
                print(f"  IK solution outside limits: {np.round(q,3)}")
                return None
        ptcp, pq = self.tcp_pose_world(q)
        dpos = np.linalg.norm(ptcp - np.array(pos_world))
        dq = min(np.linalg.norm(np.array(pq) - np.array(quat)), np.linalg.norm(np.array(pq) + np.array(quat)))
        if dpos > 0.01 or dq > 0.05:
            print(f"  IK solution FK mismatch: pos err {dpos:.4f} quat err {dq:.3f} -> abort")
            return None
        code, err = self.move_joints(q, seconds)
        tcp, _ = self.tcp_pose_world()
        print(f"  tcp now {np.round(tcp,4)} (target {np.round(pos_world,4)})")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res, timeout_sec=120)
        r = res.result().result
        self.spin(0.3)
        f = self.fingers()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def servo(self, vx=0, vy=0, vz=0, ticks=20):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(vx), float(vy), float(vz)
        for _ in range(ticks):
            msg.header.stamp = self.n.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.n, timeout_sec=0.05)
        stop = TwistStamped(); stop.header.frame_id = TW["frame"]
        for _ in range(3):
            self.twist_pub.publish(stop); rclpy.spin_once(self.n, timeout_sec=0.05)
        self.spin(0.3)

    def read_wrench(self):
        self.wrench = {}
        t0 = time.time()
        while "m" not in self.wrench and time.time() - t0 < 5:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        m = self.wrench.get("m")
        if m is None: return None
        f = m.wrench.force
        return (f.x, f.y, f.z)
