"""Control helpers: persistent node, IK -> trajectory, gripper, FK, joint state."""
import time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])

def quat_R(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)

# IK/FK tip link is panda_link8; panda_hand = link8 rotated -45 deg about z
Q_L8_FROM_HAND = (0.0, 0.0, np.sin(np.pi/8), np.cos(np.pi/8))

def topdown_quat(yaw):
    """HAND pointing straight down; yaw=0 -> fingers along world y, yaw=pi/2 -> along x.
    Returned as the panda_link8 orientation the IK service expects."""
    q_hand = (np.cos(yaw/2), np.sin(yaw/2), 0.0, 0.0)
    return qmul(q_hand, Q_L8_FROM_HAND)

class Ctl:
    def __init__(self):
        rclpy.init(); self.node = rclpy.create_node("ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(15); self.grip.wait_for_server(15)
        self.ik.wait_for_service(15); self.fk.wait_for_service(15)
        self.spin(0.5)
        while "pos" not in self.js: self.spin(0.2)

    def _js_cb(self, m):
        self.js["pos"] = dict(zip(m.name, m.position)); self.js["vel"] = dict(zip(m.name, m.velocity))

    def spin(self, t):
        end = time.time() + t
        while time.time() < end: rclpy.spin_once(self.node, timeout_sec=0.05)

    def q(self):
        self.spin(0.3); return np.array([self.js["pos"][j] for j in JOINTS])

    def fingers(self):
        self.spin(0.3); p = self.js["pos"]; return p["panda_finger_joint1"], p["panda_finger_joint2"]

    def fk_pose(self, q=None, link="panda_hand"):
        q = self.q() if q is None else q
        req = GetPositionFK.Request(); req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        ps = fut.result().pose_stamped[0].pose
        p = np.array([ps.position.x, ps.position.y, ps.position.z])
        o = np.array([ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w])
        return p, o

    def tcp_pose(self, q=None):
        p, o = self.fk_pose(q); R = quat_R(*o); return p + TCP*R[:, 2], o

    def solve_ik(self, xyz, quat, at_tcp=True, seed=None, tries=5):
        xyz = np.array(xyz, float)
        if at_tcp: xyz = xyz - TCP*quat_R(*quat)[:, 2]
        seed = self.q() if seed is None else np.array(seed)
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, xyz)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = JOINTS
            s = seed if k == 0 else seed + np.random.uniform(-0.3, 0.3, 7)
            req.ik_request.robot_state.joint_state.position = [float(x) for x in s]
            req.ik_request.timeout.sec = 1
            fut = self.ik.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                qs = np.array([sol[j] for j in JOINTS])
                # verify by FK
                pt, _ = self.tcp_pose(qs) if at_tcp else self.fk_pose(qs)
                tgt = xyz + (TCP*quat_R(*quat)[:, 2] if at_tcp else 0)
                err = np.linalg.norm(pt - tgt)
                if err < 0.005: return qs
                print(f"IK fk-mismatch {err:.4f}, retry")
            else:
                print("IK failed", None if r is None else r.error_code.val, "retry", k)
        return None

    def move_q(self, qt, seconds, tol=0.01, retries=2):
        for k in range(retries+1):
            ok = self._move_q_once(qt, seconds, tol)
            if ok: return True
            print("  retrying trajectory (controller lag)")
        return False

    def _move_q_once(self, qt, seconds, tol):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(x) for x in qt])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1)*1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(self.q() - qt).max()
        print(f"  traj code={code} max joint err={err:.4f}")
        return code == 0 and err < tol

    def move_tcp(self, xyz, yaw=0.0, seconds=3.0, seed=None):
        quat = topdown_quat(yaw)
        qs = self.solve_ik(xyz, quat, seed=seed)
        if qs is None: print("  NO IK for", xyz); return False
        ok = self.move_q(qs, seconds)
        p, _ = self.tcp_pose()
        print(f"  tcp now {np.round(p,4)} target {np.round(xyz,4)} err={np.linalg.norm(p-np.array(xyz)):.4f}")
        return ok

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def close(self): rclpy.shutdown()
