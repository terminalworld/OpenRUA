"""Panda kinematics + motion helpers for this workstation.

FK/IK done locally (numpy/scipy) because /compute_ik ignores orientation
on this machine. World frame: base at (-0.66, 0, 0.912), identity rotation.
"""
import time
import numpy as np
import rclpy
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
LIM = np.array(FJT["limits_rad"])
BASE = np.array([-0.66, 0.0, 0.912])
TCP = float(M["hand"]["tcp_offset_m"])

# (xyz, rpy) origin of each joint relative to parent, then rotate about z by q
_ORIG = [
    ((0, 0, 0.333), (0, 0, 0)),
    ((0, 0, 0), (-np.pi / 2, 0, 0)),
    ((0, -0.316, 0), (np.pi / 2, 0, 0)),
    ((0.0825, 0, 0), (np.pi / 2, 0, 0)),
    ((-0.0825, 0.384, 0), (-np.pi / 2, 0, 0)),
    ((0, 0, 0), (np.pi / 2, 0, 0)),
    ((0.088, 0, 0), (np.pi / 2, 0, 0)),
]


def _T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = Rot.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = xyz
    return T


_FIXED = [_T(o[0], o[1]) for o in _ORIG]
_HAND = _T((0, 0, 0.107), (0, 0, 0)) @ _T((0, 0, 0), (0, 0, -np.pi / 4))


def fk(q, tcp=False):
    """4x4 pose of panda_hand (or TCP point) in world."""
    T = np.eye(4)
    T[:3, 3] = BASE
    for i in range(7):
        T = T @ _FIXED[i] @ _T((0, 0, 0), (0, 0, q[i]))
    T = T @ _HAND
    if tcp:
        T = T @ _T((0, 0, TCP), (0, 0, 0))
    return T


def pose_T(pos, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


def topdown_R(yaw):
    """Hand rotation with z pointing down (-world z) and hand x axis at `yaw`
    about world z (yaw=0: hand x = world +x, hand y = world -y).
    Finger closing axis is hand y."""
    return Rot.from_euler("xyz", [np.pi, 0, yaw]).as_matrix()


def ik(target, seed, tcp=False, w_rot=0.3):
    """Solve q for target 4x4 (hand or TCP pose). Returns (q, pos_err, rot_err)."""
    seed = np.asarray(seed, float)

    def resid(q):
        T = fk(q, tcp)
        dp = T[:3, 3] - target[:3, 3]
        dR = Rot.from_matrix(T[:3, :3] @ target[:3, :3].T).as_rotvec()
        return np.concatenate([dp * 10, dR * w_rot * 10, (q - seed) * 0.01])

    best = None
    for k in range(6):
        s = seed if k == 0 else np.clip(seed + np.random.uniform(-0.6, 0.6, 7), LIM[:, 0], LIM[:, 1])
        r = least_squares(resid, s, bounds=(LIM[:, 0] + 0.02, LIM[:, 1] - 0.02), xtol=1e-10, ftol=1e-10)
        T = fk(r.x, tcp)
        pe = np.linalg.norm(T[:3, 3] - target[:3, 3])
        re = np.linalg.norm(Rot.from_matrix(T[:3, :3] @ target[:3, :3].T).as_rotvec())
        if best is None or pe + 0.05 * re < best[1] + 0.05 * best[2]:
            best = (r.x, pe, re)
        if pe < 1e-3 and re < 5e-3:
            break
    return best


class Robot:
    def __init__(self, name="pk"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                self.spin()
        d = dict(zip(self._js.name, self._js.position))
        return np.array([d[j] for j in ARM])

    def fingers(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                self.spin()
        d = dict(zip(self._js.name, self._js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin()
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def move_joints(self, waypoints, seconds, timeout=600):
        """waypoints: list of 7-vectors, evenly timed up to `seconds`."""
        waypoints = [np.asarray(w, float) for w in waypoints]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        n = len(waypoints)
        for i, w in enumerate(waypoints):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=list(map(float, w)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=timeout)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        code = res.result().result.error_code
        q = self.joints()
        err = np.abs(q - waypoints[-1]).max()
        print(f"  move done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_pose(self, pos, R, seconds=3.0, tcp=True, via=None, seed=None):
        """IK to pose then trajectory. Returns (q, pos_err, rot_err)."""
        seed = self.joints() if seed is None else seed
        q, pe, re = ik(pose_T(pos, R), seed, tcp=tcp)
        print(f"  ik pos_err={pe*1000:.1f}mm rot_err={np.degrees(re):.2f}deg q={np.round(q,3)}")
        if pe > 0.005 or re > 0.05:
            raise RuntimeError("IK failed")
        wps = ([] if via is None else list(via)) + [q]
        self.move_joints(wps, seconds)
        T = fk(self.joints(), tcp)
        print(f"  reached tcp={np.round(T[:3,3],4)}")
        return q, pe, re

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={np.round(f,4)}")
        return f
