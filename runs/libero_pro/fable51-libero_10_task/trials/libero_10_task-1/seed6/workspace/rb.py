#!/usr/bin/env python3
"""Small robot helper library for this Panda workstation (see machine.yaml)."""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import WrenchStamped, TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.510, 0.000, 0.420])  # from tf world->panda_link0
TCP_OFF = float(M["hand"]["tcp_offset_m"])
TOPDOWN = (1.0, 0.0, 0.0, 0.0)  # hand z down, fingers open along base y


# NOTE (measured): MoveIt's model frame on this machine is WORLD, not the
# arm base (FK of panda_hand at home = (-0.053, 0, 0.778) = tf world->hand).
# IK tip link is panda_link8; panda_hand is yawed -45 deg from it.
def w2b(p):
    return np.asarray(p, float)


def b2w(p):
    return np.asarray(p, float)


def grasp_quat(finger_yaw=0.0):
    """link8 quaternion for a top-down grasp whose finger-opening axis is
    world +y rotated by finger_yaw about world z (0 -> fingers along y,
    pi/2 -> fingers along x)."""
    return yaw_quat(finger_yaw - np.pi / 4)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_quat(yaw):
    """Top-down grasp rotated by yaw about world z (fingers along y at yaw=0)."""
    # q = Rz(yaw) * (1,0,0,0)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # Rz = (0,0,s,c); (1,0,0,0) ; product (w1w2 - v1.v2, w1v2 + w2v1 + v1xv2)
    # q1 = (x=0,y=0,z=s,w=c), q2 = (1,0,0,0)
    x = c * 1 + 0
    y = s * 1  # z1*x2 -> v1 x v2 = (0,0,s)x(1,0,0) = (0, s, 0)
    z = 0.0
    w = 0.0
    return (x, y, z, w)


class Robot:
    def __init__(self, name="rb"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
        end = time.time() + 15
        while "m" not in self.js and time.time() < end:
            self.spin(0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def wrench(self):
        got = {}
        sub = self.node.create_subscription(WrenchStamped, M["sensors"][1]["port"],
                                            lambda m: got.setdefault("m", m), 1)
        end = time.time() + 10
        while "m" not in got and time.time() < end:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        if "m" not in got:
            return None
        f = got["m"].wrench.force; t = got["m"].wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def fk(self, q=None, link="panda_hand"):
        """Hand pose in base frame -> (pos[3], quat[4]) ; also returns world pos."""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp_world(self, q=None):
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        tcp_b = pos + TCP_OFF * R[:, 2]
        return b2w(tcp_b), quat

    # ---------- IK ----------
    def ik(self, pos_base, quat, seed=None, at_tcp=True, timeout=60):
        pos_base = np.asarray(pos_base, float)
        if at_tcp:
            R = quat_R(*quat)
            pos_base = pos_base - TCP_OFF * R[:, 2]
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(x) for x in seed]
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            raise RuntimeError(f"IK failed code={res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_world(self, pos_world, quat=TOPDOWN, **kw):
        return self.ik(w2b(pos_world), quat, **kw)

    # ---------- acting ----------
    def move(self, q, seconds=3.0, wait=True):
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("FJT goal rejected")
        if not wait:
            return gh
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = r.result.error_code if r else None
        now = self.arm_q()
        err = max(abs(a - b) for a, b in zip(now, q))
        print(f"move: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_world(self, pos_world, quat=TOPDOWN, seconds=3.0, seed=None):
        q = self.ik_world(pos_world, quat, seed=seed)
        r = self.move(q, seconds)
        tcp, _ = self.tcp_world()
        print(f"  tcp now world={np.round(tcp,4)} target={np.round(pos_world,4)}")
        return r

    def gripper(self, width, wait=True):
        self.grip.wait_for_server(timeout_sec=10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result().result
        f = self.fingers()
        print(f"gripper: reached={res.reached_goal} stalled={res.stalled} fingers={f}")
        return res, f

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw",
                                            lambda m: got.setdefault("m", m), 1)
        end = time.time() + 15
        while "m" not in got and time.time() < end:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        img = CvBridge().imgmsg_to_cv2(got["m"], "bgr8")
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, img)
        return out
