#!/usr/bin/env python3
"""Robot helper: FK/IK (world frame), trajectory, gripper, wrench, snapshots.

Import and use inside a script (rclpy node built once per process).
World frame = panda_link0 + BASE offset.
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped, TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

BASE = np.array([0.0, 0.0, 0.0])  # MoveIt model frame == world (verified by FK)
M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")


class Robot:
    def __init__(self, name="rb"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._wr = None
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------- sensing ----------
    def js(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.js()
        return np.array([j[n] for n in ARM])

    def fingers(self):
        j = self.js()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._wr is None:
            return None
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def snap(self, cam, out=None):
        from cv_bridge import CvBridge
        import cv2
        topic = f"/{cam}/color/image_raw"
        got = []
        sub = self.node.create_subscription(Image, topic, lambda m: got.append(m), 1)
        while not got:
            rclpy.spin_once(self.node, timeout_sec=0.5)
        self.node.destroy_subscription(sub)
        out = out or f"{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got[0], desired_encoding="bgr8"))
        return out

    # ---------- kinematics ----------
    def fk(self, q=None, link="panda_hand"):
        """Return (pos_world, R_world) of link for arm config q (default current)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        req.header.frame_id = ""
        self.fk_cli.wait_for_service(timeout_sec=10)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        R = Rot.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
        return pos, R

    def ik(self, pos_world, R, seed=None, attempts=3):
        """IK for panda_hand at world pos with rotation matrix R. Returns q or None."""
        if seed is None:
            seed = self.arm_q()
        pb = np.asarray(pos_world) - BASE
        qt = Rot.from_matrix(R).as_quat()
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pb)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, qt)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=0, nanosec=200_000_000)
            self.ik_cli.wait_for_service(timeout_sec=10)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                q = np.array([sol[n] for n in ARM])
                # verify
                fp, fR = self.fk(q)
                err = np.linalg.norm(fp - pos_world)
                if err < 0.005:
                    return q
            seed = np.array(seed) + np.random.uniform(-0.2, 0.2, len(seed))
        return None

    # ---------- action ----------
    def move(self, q, sec=3.0, extra_points=None):
        """Send trajectory to q (list of arm positions). extra_points: list of (q, t) before final."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if extra_points:
            for qq, t in extra_points:
                pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        self.fjt.wait_for_server(timeout_sec=10)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        qa = self.arm_q()
        err = np.abs(qa - np.array(q)).max()
        return code, err

    def move_pose(self, pos_world, R, sec=3.0, seed=None):
        q = self.ik(pos_world, R, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {np.round(pos_world,3)}")
        code, err = self.move(q, sec)
        p, _ = self.fk()
        return code, err, p

    def grip(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        self.gr.wait_for_server(timeout_sec=10)
        fut = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def servo(self, lin, ang=(0, 0, 0), n=20, frame="panda_link0"):
        msg = TwistStamped()
        msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)


def R_from(approach, closing):
    """Rotation matrix for panda_hand: z=approach, y=closing (both world vectors)."""
    z = np.asarray(approach, float); z /= np.linalg.norm(z)
    y = np.asarray(closing, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])
