"""Reusable robot helpers: one node, clients built once.

World frame -> planner frame (panda_link0): base = world - (-0.51, 0, 0.42)
(read from TF world->panda_link0 at session start).
"""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])
TCP_OFF = 0.1034
TABLE_Z = 0.425  # world

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def down_quat(yaw_deg=0.0):
    """Hand pointing straight down (hand z = world -z), fingers separated
    along world y when yaw=0; yaw rotates the finger axis about world z."""
    # q = Rz(yaw) * Rx(pi)
    h = math.radians(yaw_deg) / 2
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sin h, cos h)
    # product (Rz * Rx): w = -0, x = cos h, y = sin h, z = 0
    return (math.cos(h), math.sin(h), 0.0, 0.0)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK,
                                          M["planning"]["ik_service"])
        assert self.fjt.wait_for_server(timeout_sec=20)
        assert self.grip.wait_for_server(timeout_sec=20)
        assert self.ik.wait_for_service(timeout_sec=20)
        self.spin(0.5)

    def _on_js(self, msg):
        self.js = dict(zip(msg.name, msg.position))

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---- sensing
    def joints(self):
        self.js = {}
        while not self.js:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return dict(self.js)

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def hand_world(self):
        """(pos, quat xyzw) of panda_hand in world via TF (fresh spin)."""
        self.spin(0.3)
        t = self.tfbuf.lookup_transform("world", "panda_hand",
                                        rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), (q.x, q.y, q.z, q.w)

    def tcp_world(self):
        p, q = self.hand_world()
        R = quat_to_R(*q)
        return p + TCP_OFF * R[:, 2], q

    # ---- acting
    def ik_world_tcp(self, xyz, quat, seed=None):
        """IK for a TCP pose given in WORLD coords; returns arm joints."""
        R = quat_to_R(*quat)
        hand = np.array(xyz) - TCP_OFF * R[:, 2]
        # empirically (compute_fk) the planner frame is WORLD here; the IK
        # tip link is panda_link8 = hand rotated by +45 deg about hand z
        base = hand
        quat = quat_mul(quat, (0.0, 0.0, math.sin(math.pi / 8),
                               math.cos(math.pi / 8)))
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, base)
        (p.orientation.x, p.orientation.y,
         p.orientation.z, p.orientation.w) = map(float, quat)
        s = JointState()
        s.name = list(ARM)
        s.position = list(seed) if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state = s
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"IK failed code={code} for tcp={xyz}")
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[n] for n in ARM]

    def move_q(self, q, sec=3.0, tol=0.02):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(sec),
                                      nanosec=int((sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        self.spin(0.3)
        err = max(abs(a - b) for a, b in zip(self.arm_q(), q))
        print(f"  move: code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, xyz, quat, sec=3.0):
        q = self.ik_world_tcp(xyz, quat)
        code, err = self.move_q(q, sec)
        tcp, _ = self.tcp_world()
        d = np.array(xyz) - tcp
        print(f"  tcp now {tcp.round(4)} target {np.round(xyz,4)} "
              f"delta {d.round(4)}", flush=True)
        return tcp

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        self.spin(0.3)
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} "
              f"stalled={r.stalled} pos={r.position:.4f} fingers={f}",
              flush=True)
        return f
