#!/usr/bin/env python3
"""Small persistent helper around IK + FollowJointTrajectory + gripper.

World frame in, base-frame IK under the hood. Import and use:
    from arm import Arm; a = Arm(); a.move_hand(pos, R, secs)
"""
import sys, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
# MoveIt's model frame here IS world (FK/IK verified against TF), so no offset
BASE = np.zeros(3)
BASE_POS = np.array([-0.660, 0.0, 0.912])   # world position of panda_link0 (TF)
TCP = M["hand"]["tcp_offset_m"]


def R_from_axes(z, y=None, x=None):
    """Rotation whose columns are hand x,y,z axes expressed in world."""
    z = np.asarray(z, float); z /= np.linalg.norm(z)
    if y is not None:
        y = np.asarray(y, float); y -= z * (y @ z); y /= np.linalg.norm(y)
        x = np.cross(y, z)
    else:
        x = np.asarray(x, float); x -= z * (x @ z); x /= np.linalg.norm(x)
        y = np.cross(z, x)
    return np.c_[x, y, z]


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, M["sensors"][1]["port"], self._on_wr, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.spin_until(lambda: self.js is not None)

    def _on_js(self, m): self.js = m
    def _on_wr(self, m): self.wr = m

    def spin_until(self, pred, timeout=30):
        t0 = time.time()
        while not pred() and time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return pred()

    def q(self):
        """Current arm joint positions in manifest order."""
        self.js = None; self.spin_until(lambda: self.js is not None)
        d = dict(zip(self.js.name, self.js.position))
        return np.array([d[j] for j in JOINTS])

    def fingers(self):
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self.wr = None; self.spin_until(lambda: self.wr is not None)
        f = self.wr.wrench.force; t = self.wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def hand_pose(self, q=None):
        """FK: world position + rotation of panda_hand."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(self.q() if q is None else q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        R = Rot.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
        return pos, R

    def solve_ik(self, pos, R, seed=None, at_tcp=False):
        """World pose of hand (or tcp point) -> joint vector or None."""
        pos = np.asarray(pos, float)
        if at_tcp:
            pos = pos - TCP * R[:, 2]
        p_base = pos - BASE
        # IK's tip link is panda_link8; panda_hand = link8 * Rz(-45deg)
        R8 = np.asarray(R) @ Rot.from_euler("z", np.pi / 4).as_matrix()
        qx, qy, qz, qw = Rot.from_matrix(R8).as_quat()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = p_base
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = list(self.q() if seed is None else seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"  IK failed: {None if r is None else r.error_code.val}")
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    def traj(self, points, secs, verbose=True):
        """points: list of joint vectors; secs: total duration (or list of times)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(points)
        times = secs if isinstance(secs, (list, tuple)) else [secs * (i + 1) / n for i in range(n)]
        for p, t in zip(points, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in p])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(self.q() - np.asarray(points[-1])).max()
        if verbose:
            print(f"  traj done code={code} max joint err={err:.4f}")
        return code, err

    def move_hand(self, pos, R, secs=3.0, at_tcp=False, max_jump=2.5):
        q0 = self.q()
        sol = self.solve_ik(pos, R, seed=q0, at_tcp=at_tcp)
        if sol is None:
            return False
        jump = np.abs(sol - q0).max()
        if jump > max_jump:
            print(f"  IK solution jumps {jump:.2f} rad; refusing")
            return False
        code, err = self.traj([sol], secs)
        for _ in range(2):
            if err < 0.02:
                break
            # controller lag on long goals: resend the same target
            code, err = self.traj([sol], max(2.0, secs / 2))
        return err < 0.02

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        self.q()
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def snap(self, cam, out):
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        import cv2
        got = []
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw", got.append, 1)
        self.spin_until(lambda: got)
        self.node.destroy_subscription(sub)
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got[0], "bgr8"))
        return out


def look_pose(cam, target, y_axis=(1, 0, 0)):
    """Hand pose that puts the eye-in-hand camera at `cam` looking at `target`.
    Camera sits at +0.05 along hand x; image right = hand y."""
    cam = np.asarray(cam, float); z = np.asarray(target, float) - cam
    R = R_from_axes(z=z, y=y_axis)
    return cam - 0.05 * R[:, 0], R
