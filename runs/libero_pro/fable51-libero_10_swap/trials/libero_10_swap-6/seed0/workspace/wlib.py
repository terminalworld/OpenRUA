"""Shared helpers: TF, depth->world cloud, FK/IK, trajectory, gripper."""
import struct, sys, time
import numpy as np
import rclpy, yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")["joints"]
BASE_OFF = np.zeros(3)  # IK/FK on this machine already work in WORLD coords (verified vs TF and manual FK)

def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s, 0.25*s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1 + R[0,0] - R[1,1] - R[2,2]) * 2
        return np.array([0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s])
    if i == 1:
        s = np.sqrt(1 + R[1,1] - R[0,0] - R[2,2]) * 2
        return np.array([(R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s])
    s = np.sqrt(1 + R[2,2] - R[0,0] - R[1,1]) * 2
    return np.array([(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s])

def topdown_quat(yaw):
    """Hand pointing straight down (hand Z = -world Z), rotated by yaw about world Z.
    yaw=0 -> hand X = world X, fingers close along world Y."""
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    R0 = np.diag([1.0, -1.0, -1.0])
    return R_to_quat(Rz @ R0)

class W(Node):
    def __init__(self):
        super().__init__("wlib")
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self)
        self.bridge = CvBridge()
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._js, 1)
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self, GripperCommand, "/franka_gripper/gripper_action")

    def _js(self, m): self.js = m

    def spin(self, t=0.2): rclpy.spin_once(self, timeout_sec=t)

    def joints(self):
        self.js = None
        while self.js is None: self.spin()
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints(); return [d[j] for j in ARM]

    def grab(self, topic, typ, timeout=30.0):
        got = {}
        sub = self.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
        t0 = time.time()
        while "m" not in got and time.time() - t0 < timeout: self.spin()
        self.destroy_subscription(sub)
        if "m" not in got: raise RuntimeError(f"no msg on {topic}")
        return got["m"]

    def tf(self, target, source):
        t0 = time.time()
        while time.time() - t0 < 10:
            self.spin()
            if self.tfbuf.can_transform(target, source, rclpy.time.Time()): break
        t = self.tfbuf.lookup_transform(target, source, rclpy.time.Time())
        q = t.transform.rotation; tr = t.transform.translation
        T = np.eye(4); T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w); T[:3, 3] = [tr.x, tr.y, tr.z]
        return T

    def cloud(self, cam):
        """World-frame point cloud (H,W,3) + color image from a camera."""
        depth_msg = self.grab(f"/{cam}/depth/image_raw", Image)
        color_msg = self.grab(f"/{cam}/color/image_raw", Image)
        info = self.grab(f"/{cam}/color/camera_info", CameraInfo)
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
        color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        H, Wd = depth.shape
        u, v = np.meshgrid(np.arange(Wd), np.arange(H))
        pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth, np.ones_like(depth)], -1)
        T = self.tf("world", f"{cam}_optical_frame")
        pw = pc @ T.T
        return pw[..., :3], color

    def fk_pose(self, q=None):
        """Hand pose in WORLD frame (T 4x4) for arm joints q (default current)."""
        if q is None: q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(x) for x in q]
        self.fk_cli.wait_for_service(timeout_sec=10); fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1: raise RuntimeError(f"FK failed {r}")
        p = r.pose_stamped[0].pose
        T = np.eye(4); T[:3, :3] = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        T[:3, 3] = [p.position.x, p.position.y, p.position.z]
        T[:3, 3] += BASE_OFF  # planner frame is panda_link0
        return T

    def ik(self, pos_world, quat, seed=None, tries=1):
        """IK for hand at world pos/quat. Returns joint list or None."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos_world) - BASE_OFF
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(x) for x in (seed or self.arm_q())]
        req.ik_request.timeout.sec = 2
        for _ in range(tries):
            self.ik_cli.wait_for_service(timeout_sec=10); fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def move(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, vq in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in vq])
                t = seconds * (i + 1) / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)); pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9)); pts.append(pt)
        goal.trajectory.points = pts
        self.fjt.wait_for_server(timeout_sec=10)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"move: error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width); goal.command.max_effort = 30.0
        self.grip.wait_for_server(timeout_sec=10)
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=120)
        r = res.result().result
        d = self.joints()
        print(f"gripper: reached={r.reached_goal} stalled={r.stalled} fingers={d['panda_finger_joint1']:.4f},{d['panda_finger_joint2']:.4f}", flush=True)
        return d['panda_finger_joint1']

    def move_pose(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed, tries=3)
        if q is None:
            print(f"IK FAILED for {pos}", flush=True); return None
        return self.move(q, seconds)

def start():
    rclpy.init(); return W()
