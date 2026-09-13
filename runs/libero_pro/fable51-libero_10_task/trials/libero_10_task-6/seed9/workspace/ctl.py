"""Reusable control helpers: IK (MoveIt), FK, trajectory, gripper, TF hand pose.
World<->base: base = panda_link0 at world (-0.51, 0, 0.42), identity rotation.
"""
import sys, time, numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = ARM["joints"]
TCP = M["hand"]["tcp_offset_m"]
BASE_W = np.zeros(3)  # planner frame is world here (verified via FK)
DOWN = (1.0, 0.0, 0.0, 0.0)  # hand pointing down, fingers along world y

class Ctl:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("ctl")
        self.js = {}
        self.n.create_subscription(JointState, "/joint_states", self._js, 10)
        self.ik = self.n.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.n, FollowJointTrajectory, ARM["port"])
        self.grip = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.tf = Buffer(); TransformListener(self.tf, self.n)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.spin(0.5)

    def _js(self, m): self.js = dict(zip(m.name, m.position))
    def spin(self, t):
        end = time.time() + t
        while time.time() < end: rclpy.spin_once(self.n, timeout_sec=0.05)
    def joints(self):
        self.js = {}
        while not self.js: rclpy.spin_once(self.n, timeout_sec=0.1)
        return self.js
    def arm_q(self): j = self.joints(); return [j[k] for k in JOINTS]
    def fingers(self): j = self.joints(); return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def hand_world(self):
        """world pose of panda_hand from TF (fresh)."""
        self.spin(0.3)
        t = self.tf.lookup_transform("world", "panda_hand", rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), (q.x, q.y, q.z, q.w)
    def tcp_world(self):
        p, q = self.hand_world(); R = quat_R(*q); return p + TCP * R[:, 2], q

    def solve_ik(self, xyz_world, quat=DOWN, at_tcp=True, seed=None):
        xyz = np.array(xyz_world, float)
        if at_tcp: xyz = xyz - TCP * quat_R(*quat)[:, 2]
        b = xyz - BASE_W
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = M["planning"]["group"]; r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = b
        # IK tip link is panda_link8; panda_hand = link8 * Rz(-45deg) (verified). request link8 = hand * Rz(+45deg)
        o = r.pose_stamped.pose.orientation; o.x, o.y, o.z, o.w = qmul(quat, (0.0, 0.0, np.sin(np.pi/8), np.cos(np.pi/8)))
        r.avoid_collisions = False
        r.timeout.sec = 2
        s = JointState(); s.name = list(JOINTS); s.position = list(seed if seed is not None else self.arm_q())
        r.robot_state.joint_state = s
        fut = self.ik.call_async(req); rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None, (None if res is None else res.error_code.val)
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS], 1

    def traj(self, q, secs, q_via=None):
        g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = list(JOINTS)
        pts = []
        if q_via is not None:
            for i, qv in enumerate(q_via):
                p = JointTrajectoryPoint(positions=list(qv)); t = secs * (i + 1) / (len(q_via) + 1)
                p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)); pts.append(p)
        p = JointTrajectoryPoint(positions=list(q)); p.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        pts.append(p); g.trajectory.points = pts
        f = self.fjt.send_goal_async(g); rclpy.spin_until_future_complete(self.n, f)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.n, rf)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        return code, err

    def move(self, xyz_world, quat=DOWN, secs=3.0, at_tcp=True, retries=2):
        q, code = self.solve_ik(xyz_world, quat, at_tcp)
        if q is None: print(f"IK FAILED code={code} for {xyz_world}"); return False
        for i in range(retries + 1):
            c, e = self.traj(q, secs)
            print(f"  traj code={c} maxerr={e:.4f}")
            if c == 0 and e < 0.02: break
        p, _ = self.tcp_world()
        print(f"  tcp now {p.round(4)} target {np.round(xyz_world,4)} d={np.linalg.norm(p-np.array(xyz_world)):.4f}")
        return True

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.n, f, timeout_sec=30)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.n, rf, timeout_sec=120)
        r = rf.result().result; self.spin(0.3)
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={np.round(self.fingers(),4)}")
        return r

def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

def qmul(a, b):
    """quaternion product a*b, (x,y,z,w)."""
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + w2*x1 + y1*z2 - z1*y2,
            w1*y2 + w2*y1 + z1*x2 - x1*z2,
            w1*z2 + w2*z1 + x1*y2 - y1*x2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)

def yaw_down(yaw):
    """hand quaternion: pointing down, finger axis rotated by yaw about world z from DOWN (fingers along y)."""
    return qmul((0.0, 0.0, np.sin(yaw/2), np.cos(yaw/2)), DOWN)
