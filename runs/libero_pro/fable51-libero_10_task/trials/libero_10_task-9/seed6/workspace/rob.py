"""Shared helpers: IK/FK clients, world<->base frames, pose building, trajectory + gripper."""
import time, numpy as np, rclpy, yaml
from scipy.spatial.transform import Rotation as Rot
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"]=="joint_trajectory")
JOINTS = ARM["joints"]
BASE_W = np.array([0.0, 0.0, 0.0])   # FK/IK model frame == world on this machine (verified via /compute_fk)
TCP = M["hand"]["tcp_offset_m"]

def w2b(p): return np.asarray(p, float) - BASE_W
def b2w(p): return np.asarray(p, float) + BASE_W

def R_from_axes(approach, finger_axis):
    z = np.asarray(approach, float); z /= np.linalg.norm(z)
    y = np.asarray(finger_axis, float); y -= y.dot(z)*z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])

def hand_pose_from_tcp(tcp_world, R):
    """panda_hand origin (world) given TCP world position and hand rotation matrix."""
    return np.asarray(tcp_world, float) - TCP * R[:, 2]

class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, ARM["port"])
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.spin(0.5)
    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))
    def spin(self, sec):
        end = time.time()+sec
        while time.time() < end: rclpy.spin_once(self.node, timeout_sec=0.05)
    def joints(self):
        self.js = {}
        while len(self.js) < 7: rclpy.spin_once(self.node, timeout_sec=0.1)
        return [self.js[j] for j in JOINTS]
    def fingers(self):
        self.joints(); return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")
    def _call(self, cli, req, timeout=60):
        cli.wait_for_service(timeout_sec=10)
        f = cli.call_async(req); rclpy.spin_until_future_complete(self.node, f, timeout_sec=timeout)
        return f.result()
    def solve_ik(self, hand_pos_world, R, seed=None, link="panda_hand", timeout=5.0):
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = M["planning"]["group"]; r.ik_link_name = link
        r.pose_stamped.header.frame_id = ""
        pb = w2b(hand_pos_world); q = Rot.from_matrix(R).as_quat()
        p = r.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        seed = seed if seed is not None else self.joints()
        r.robot_state.joint_state.name = list(JOINTS); r.robot_state.joint_state.position = [float(v) for v in seed]
        r.timeout.sec = int(timeout); r.timeout.nanosec = int((timeout%1)*1e9)
        r.avoid_collisions = False
        res = self._call(self.ik, req)
        if res is None or res.error_code.val != 1: return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]
    def solve_fk(self, q, link="panda_hand"):
        req = GetPositionFK.Request(); req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS); req.robot_state.joint_state.position = [float(v) for v in q]
        res = self._call(self.fk, req)
        p = res.pose_stamped[0].pose
        pos = b2w([p.position.x, p.position.y, p.position.z])
        R = Rot.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
        return pos, R
    def tcp(self):
        pos, R = self.solve_fk(self.joints()); return pos + TCP*R[:,2], R
    def move_joints(self, q, sec, via=None):
        """Send one FJT goal; via = list of (q, t) intermediate points."""
        self.fjt.wait_for_server(timeout_sec=10)
        # sim joints move at most ~0.2 rad/s: stretch the duration so the goal is reachable
        cur = np.array(self.joints()); need = np.abs(np.array(q) - cur).max() / 0.15
        if need > sec:
            scale = need / sec; sec = need
            via = [(qq, t*scale) for qq, t in (via or [])]
            print(f"  (duration stretched to {sec:.1f}s)")
        g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = list(JOINTS)
        pts = []
        for qq, t in (via or []) + [(q, sec)]:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t%1)*1e9)); pts.append(pt)
        g.trajectory.points = pts
        f = self.fjt.send_goal_async(g); rclpy.spin_until_future_complete(self.node, f)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.joints()) - np.array(q)).max()
        print(f"  move done code={code} max_joint_err={err:.4f}")
        return code, err
    def gripper(self, width):
        self.grip.wait_for_server(timeout_sec=10)
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
        f = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, f, timeout_sec=30)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}")
        return r
