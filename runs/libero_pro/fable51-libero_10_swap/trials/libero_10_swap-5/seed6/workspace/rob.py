#!/usr/bin/env python3
"""Small helper library: FK/IK, trajectory, gripper, joint state, in one node.

Coordinates for ik()/fk() are in panda_link0 (planning frame); use w2b()/b2w()
to convert to/from world (panda_link0 sits at world (-0.75, 0, 0.912)).
"""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import WrenchStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_W = np.array([-0.75, 0.0, 0.912])
TCP = M["hand"]["tcp_offset_m"]


def w2b(p):
    return np.asarray(p, float) - BASE_W


def b2w(p):
    return np.asarray(p, float) + BASE_W


def quat_topdown(yaw_x):
    """Hand z down; hand x-axis at angle yaw_x (rad) in the world xy plane.
    Fingers close along hand y = (sin a, -cos a, 0)."""
    a = yaw_x
    R = np.array([[np.cos(a), np.sin(a), 0.0],
                  [np.sin(a), -np.cos(a), 0.0],
                  [0.0, 0.0, -1.0]]).T  # columns = hand axes
    return R_to_quat(R)


def R_to_quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()  # x y z w


def quat_to_R(q):
    from scipy.spatial.transform import Rotation
    return Rotation.from_quat(q).as_matrix()


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr.pop("m", None)
        while "m" not in self._wr:
            self.spin(0.2)
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z,
                         w.torque.x, w.torque.y, w.torque.z])

    def _seed(self, arm=None):
        js = JointState()
        arm = arm if arm is not None else self.arm()
        js.name = list(JOINTS)
        js.position = [float(v) for v in arm]
        return js

    def fk(self, arm=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(arm)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w]))

    def ik(self, pos_b, quat, seed=None, at_tcp=False, timeout=30):
        """pos_b in panda_link0 frame; if at_tcp, pos is the TCP target."""
        pos_b = np.asarray(pos_b, float)
        if at_tcp:
            R = quat_to_R(quat)
            pos_b = pos_b - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None:
            raise RuntimeError("IK timeout")
        if r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move(self, positions, seconds=3.0, via=None):
        """Send one trajectory; via = optional list of (positions, t)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        for q, t in (via or []) + [(positions, seconds)]:
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        now = self.arm()
        err = np.abs(np.array(now) - np.array(positions)).max()
        print(f"move: error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"gripper: reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def grab_img(self, topic, timeout=30):
        got = {}
        sub = self.node.create_subscription(Image, topic, lambda m: got.setdefault("m", m), 1)
        t0 = time.time()
        while "m" not in got and time.time() - t0 < timeout:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        return got.get("m")

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        m = self.grab_img(f"/{cam}/color/image_raw")
        img = CvBridge().imgmsg_to_cv2(m, "bgr8")
        cv2.imwrite(out or f"/workspace/{cam}.png", img)
        return img

    def depth(self, cam):
        from cv_bridge import CvBridge
        m = self.grab_img(f"/{cam}/depth/image_raw")
        return CvBridge().imgmsg_to_cv2(m, "passthrough")
