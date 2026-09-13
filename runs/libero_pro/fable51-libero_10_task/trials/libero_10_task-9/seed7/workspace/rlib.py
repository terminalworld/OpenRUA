"""Robot helper library: persistent node, FK/IK, trajectory, gripper, TF."""
import time, math, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE = np.zeros(3)   # verified: FK/IK model frame == world here

class Robot:
    def __init__(self, name="rlib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.spin(0.5)

    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, t):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self.js = {}
        while not self.js:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return [self.js[j] for j in JOINTS]

    def fingers(self):
        self.joints()
        return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")

    # ---- kinematics (poses in WORLD frame; converted to base for MoveIt) ----
    def fk_world(self, q=None, link="panda_hand"):
        q = q if q is not None else self.joints()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r.error_code.val if r else None}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def ik_world(self, pos, quat, seed=None, attempts=3, timeout=5.0):
        """pos: world xyz of panda_hand frame; quat xyzw. Returns joint list or None."""
        seed = seed if seed is not None else self.joints()
        pb = np.array(pos) - BASE
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pb)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.timeout.sec = int(timeout)
            req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
            req.ik_request.avoid_collisions = False
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
        return None

    # ---- motion ----
    def move_joints(self, points, seconds, wait=True):
        """points: list of joint lists (waypoints), evenly timed to `seconds` total."""
        if not isinstance(points[0], (list, tuple, np.ndarray)):
            points = [points]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(points)
        for i, q in enumerate(points):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        if not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q = self.joints()
        err = max(abs(a - b) for a, b in zip(q, points[-1]))
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        send = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def tf(self, target, source):
        end = time.time() + 5
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.tfbuf.can_transform(target, source, rclpy.time.Time()):
                break
        t = self.tfbuf.lookup_transform(target, source, rclpy.time.Time()).transform
        return (np.array([t.translation.x, t.translation.y, t.translation.z]),
                np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]))

def quat_from_R(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()  # xyzw

def R_from_axes(xh, yh, zh):
    return np.column_stack([xh, yh, zh])

def hand_pose_from_tcp(tcp, R, tcp_offset=M["hand"]["tcp_offset_m"]):
    """world position of panda_hand frame such that the TCP (fingertip centre) lands on `tcp`."""
    return np.array(tcp) - tcp_offset * R[:, 2]
