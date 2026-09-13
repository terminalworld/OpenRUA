"""Reusable robot helpers: IK (world frame, link8 target), FJT, gripper, joint state."""
import math, time, numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
# link8 orientation for hand fingers along world y (hand q = (1,0,0,0))
Q_FINGERS_Y = (0.9239, -0.3827, 0.0, 0.0)
# link8 orientation for fingers along world x (hand q = Rx(180)*Rz(90) = (0.7071,0.7071,0,0))
Q_FINGERS_X = (1.0, 0.0, 0.0, 0.0)

class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("agent_lib")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.wr = {}
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: self.wr.__setitem__("m", m), 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, t=0.3):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self.js = {}
        while not self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return [self.js[j] for j in JOINTS]

    def fingers(self):
        self.joints()
        return self.js["panda_finger_joint1"], self.js["panda_finger_joint2"]

    def wrench(self):
        self.wr = {}
        while "m" not in self.wr:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        w = self.wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def fk_pose(self, q=None, link="panda_link8"):
        q = q if q is not None else self.joints()
        req = GetPositionFK.Request(); req.fk_link_names = [link]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(q)
        fut = self.fk.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z,
                         p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def solve_ik(self, pos, quat, seed=None, tries=1):
        seed = seed if seed is not None else self.joints()
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = JOINTS
            s = list(seed) if k == 0 else list(np.array(seed) + np.random.uniform(-0.3, 0.3, 7))
            req.ik_request.robot_state.joint_state.position = s
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=2)
            fut = self.ik.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [d[j] for j in JOINTS]
        return None

    def move_joints(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i + 1) / n
                pts.append(JointTrajectoryPoint(positions=list(map(float, v)),
                            time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9))))
        pts.append(JointTrajectoryPoint(positions=list(map(float, q)),
                    time_from_start=Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))))
        goal.trajectory.points = pts
        fut = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        rf = h.get_result_async(); rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        cur = np.array(self.joints())
        err = np.abs(cur - np.array(q)).max()
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, seed=None):
        q = self.solve_ik(pos, quat, seed=seed, tries=5)
        if q is None:
            return None, None, None
        code, err = self.move_joints(q, seconds)
        return q, code, err

    def gripper(self, width, timeout=120):
        g = GripperCommand.Goal(); g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def close(self):
        rclpy.shutdown()

# --- orientation helpers -------------------------------------------------
from scipy.spatial.transform import Rotation as Rot

def link8_quat_from_hand_axes(hand_x, hand_y, hand_z):
    """Return link8 quaternion (x,y,z,w) given desired world-frame hand axes."""
    Rh = np.column_stack([hand_x, hand_y, hand_z])
    R8 = Rot.from_matrix(Rh) * Rot.from_euler("z", 45, degrees=True)   # hand = link8 * Rz(-45)
    return R8.as_quat()

# hand pointing down (z=-Z), fingers along world y  -> should equal Q_FINGERS_Y
Q_DOWN_FY = link8_quat_from_hand_axes([1,0,0],[0,-1,0],[0,0,-1])
# hand pointing down, fingers along world x
Q_DOWN_FX = link8_quat_from_hand_axes([0,1,0],[1,0,0],[0,0,-1])
# hand pointing toward -x (horizontal), fingers along world z
Q_BACK_FZ = link8_quat_from_hand_axes([0,-1,0],[0,0,1],[-1,0,0])
# hand pointing down, fingers along x, hand x = -Y (matches Ry(-90) applied to Q_BACK_FZ)
Q_DOWN_FX2 = link8_quat_from_hand_axes([0,-1,0],[-1,0,0],[0,0,-1])
