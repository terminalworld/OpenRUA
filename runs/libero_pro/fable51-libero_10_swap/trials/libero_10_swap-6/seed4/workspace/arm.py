"""Reusable arm helpers: joints, IK (prints solution), FJT, gripper, FK."""
import sys, time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
JOINTS = ARM["joints"]
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])
TCP_OFF = M["hand"]["tcp_offset_m"]

class Arm:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, ARM["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
    def spin(self, fut, timeout=600):
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout); return fut.result()
    def joints(self, fresh=True):
        if fresh: self._js.pop("m", None)
        end = time.time()+10
        while "m" not in self._js and time.time()<end: rclpy.spin_once(self.node, timeout_sec=0.1)
        m = self._js["m"]; d = dict(zip(m.name, m.position))
        return d
    def arm_q(self):
        d = self.joints(); return [d[j] for j in JOINTS]
    def fingers(self):
        d = self.joints(); return d["panda_finger_joint1"], d["panda_finger_joint2"]
    def fk_hand(self):
        """hand pose in planner frame via /compute_fk; returns (pos, quat xyzw)"""
        self.fk.wait_for_service(5)
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = self.arm_q()
        res = self.spin(self.fk.call_async(req), 60)
        p = res.pose_stamped[0].pose
        return res.pose_stamped[0].header.frame_id, np.array([p.position.x,p.position.y,p.position.z]), np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
    def solve_ik(self, pos, quat, seed=None, tries=3):
        self.ik.wait_for_service(5)
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = M["planning"]["group"]; r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = map(float,pos)
        r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y, r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = map(float,quat)
        r.robot_state.joint_state.name = JOINTS
        r.robot_state.joint_state.position = list(seed) if seed is not None else self.arm_q()
        r.timeout.sec = 2
        for _ in range(tries):
            res = self.spin(self.ik.call_async(req), 60)
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
        print("IK failed", None if res is None else res.error_code.val); return None
    def move_joints(self, q, seconds=3.0, retries=2):
        self.fjt.wait_for_server(5)
        for attempt in range(retries+1):
            goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
            pt = JointTrajectoryPoint(positions=[float(x) for x in q])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds%1)*1e9))
            goal.trajectory.points = [pt]
            gh = self.spin(self.fjt.send_goal_async(goal))
            res = self.spin(gh.get_result_async())
            code = res.result.error_code
            cur = np.array(self.arm_q()); err = np.abs(cur-np.array(q)).max()
            print(f"fjt code={code} max joint err={err:.4f}")
            if err < 0.02: return True
            seconds = max(seconds, 2.0)
        return err < 0.05
    def gripper(self, width):
        self.grip.wait_for_server(5)
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        gh = self.spin(self.grip.send_goal_async(g)); res = self.spin(gh.get_result_async(), 300)
        f = self.fingers(); print(f"gripper reached={res.result.reached_goal} stalled={res.result.stalled} fingers={f}")
        return f
def down_quat(yaw_deg=0.0):
    """hand pointing down (z down), fingers along world y when yaw=0; yaw rotates about world z."""
    return Rot.from_euler("z", yaw_deg, degrees=True).__mul__(Rot.from_quat([1,0,0,0])).as_quat()

def tcp_to_hand(tcp, yaw_deg=0.0):
    q = down_quat(yaw_deg); R = Rot.from_quat(q).as_matrix()
    return np.array(tcp) - TCP_OFF * R[:, 2], q

def move_tcp(a, tcp, yaw_deg=0.0, seconds=3.0, seed=None):
    pos, _ = tcp_to_hand(tcp, yaw_deg)
    # machine fact: this IK solver's hand yaw comes out +45 deg from the request
    _, q_req = tcp_to_hand(tcp, yaw_deg - 45.0)
    sol = a.solve_ik(pos, q_req, seed=seed)
    if sol is None: return False
    ok = a.move_joints(sol, seconds)
    fr, p, qq = a.fk_hand()
    R = Rot.from_quat(qq).as_matrix(); tcp_now = p + TCP_OFF * R[:, 2]
    print(f"TCP now {tcp_now.round(4)} target {np.round(tcp,4)}  hand y-axis {R[:,1].round(3)}")
    return ok
