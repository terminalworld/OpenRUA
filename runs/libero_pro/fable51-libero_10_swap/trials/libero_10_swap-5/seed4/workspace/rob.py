"""Shared robot helpers: joint state, FK/IK, trajectory, gripper."""
import time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
# verified by IK/FK roundtrip: compute_ik/compute_fk on this machine work in true
# WORLD coordinates (robot base offset already included), so no base shift is applied
BASE_W = np.array([0.0, 0.0, 0.0])
TCP = M["hand"]["tcp_offset_m"]

class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("pickplace")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, m): self._js = dict(zip(m.name, m.position))

    def joints(self, fresh=True):
        if fresh:
            self._js = {}
            while not self._js: rclpy.spin_once(self.node, timeout_sec=0.2)
        return self._js

    def arm_q(self): j = self.joints(); return np.array([j[n] for n in ARM])
    def fingers(self): j = self.joints(); return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def _seed(self, q):
        s = JointState(); s.name = list(ARM); s.position = [float(x) for x in q]; return s

    def fk_hand(self, q=None):
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result(); p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat, r.error_code.val, r.pose_stamped[0].header.frame_id

    def ik_world(self, pos_w, quat, seed=None, tcp=False):
        """IK for hand pose given in WORLD frame (quat xyzw). tcp=True: pos is the TCP."""
        pos_w = np.array(pos_w, float)
        if tcp:
            pos_w = pos_w - TCP * Rot.from_quat(quat).as_matrix()[:, 2]
        p_b = pos_w - BASE_W
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = M["planning"]["group"]; r.ik_link_name = "panda_hand"; r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = map(float, p_b)
        r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y, r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = map(float, quat)
        r.robot_state.joint_state = self._seed(self.arm_q() if seed is None else seed)
        r.avoid_collisions = False
        r.timeout.sec = 5
        fut = self.ik.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None, (None if res is None else res.error_code.val)
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        q = np.array([sol[j] for j in ARM])
        # IK can report success with a bogus solution: verify by FK
        fp, fq, _, _ = self.fk_hand(q)
        perr = np.linalg.norm(fp - pos_w)
        aerr = (Rot.from_quat(fq) * Rot.from_quat(quat).inv()).magnitude()
        if perr > 0.005 or aerr > 0.03:
            return None, f"fk-mismatch pos={perr:.4f} ang={aerr:.3f}"
        return q, 1

    def move(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        for i, w in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(x) for x in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)); pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(self.arm_q() - np.array(q)).max()
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        return r.reached_goal, r.stalled, self.fingers()

def topdown_quat(theta):
    """Hand z = world -z, hand y (finger closing axis) = (cos t, sin t, 0). Returns xyzw."""
    yh = np.array([np.cos(theta), np.sin(theta), 0.0]); zh = np.array([0, 0, -1.0]); xh = np.cross(yh, zh)
    return Rot.from_matrix(np.column_stack([xh, yh, zh])).as_quat()
