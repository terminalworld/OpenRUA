#!/usr/bin/env python3
"""Small helper library for this Panda: joint state, FK, IK, trajectory,
gripper. Clients are built once per Robot instance.

World <-> base: panda_link0 sits at world (-0.51, 0, 0.42), identity rotation.
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])
TCP_OFF = 0.1034


def w2b(p):
    return np.asarray(p, float) - BASE_IN_WORLD


def b2w(p):
    return np.asarray(p, float) + BASE_IN_WORLD


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        M = yaml.safe_load(open("/workspace/machine.yaml"))
        self.fjt_entry = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
        self.grip_entry = next(a for a in M["actuators"] if a["kind"] == "gripper")
        self.joints = self.fjt_entry["joints"]
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, self.fjt_entry["port"])
        self.grip = ActionClient(self.node, GripperCommand, self.grip_entry["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no fjt server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no ik"
        assert self.fk.wait_for_service(10), "no fk"

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joint_state(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        js = self.joint_state()
        return [js[j] for j in self.joints]

    def fingers(self):
        js = self.joint_state()
        return js["panda_finger_joint1"], js["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin(0.2)
        f = self._wr.wrench.force
        return np.array([f.x, f.y, f.z])

    def _seed(self, q=None):
        q = q if q is not None else self.arm_q()
        s = JointState()
        s.name = list(self.joints)
        s.position = [float(v) for v in q]
        return s

    def fk_hand(self, q=None):
        """Hand pose in WORLD frame (measured: FK output frame is world): (pos[3], quat[4])."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        assert r is not None and r.error_code.val == 1, f"FK failed {r}"
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_world(self, q=None):
        p, quat = self.fk_hand(q)
        R = quat_R(*quat)
        return p + TCP_OFF * R[:, 2]

    def ik_tcp_world(self, pw, quat, seed=None, tries=5):
        """IK for TCP at world pos pw with hand orientation quat. Returns joint list or None."""
        # machine fact (measured): FK/IK model frame is WORLD here, and the
        # default IK tip is panda_link8, so name the hand link explicitly.
        R = quat_R(*quat)
        pb = np.asarray(pw, float) - TCP_OFF * R[:, 2]
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            po = req.ik_request.pose_stamped.pose
            po.position.x, po.position.y, po.position.z = map(float, pb)
            po.orientation.x, po.orientation.y, po.orientation.z, po.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state = self._seed(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 1
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in self.joints]
            print(f"  ik try {k} failed: {None if r is None else r.error_code.val}")
        return None

    def move_joints(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joints)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i + 1) / n
                p = JointTrajectoryPoint(positions=[float(x) for x in v])
                p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(p)
        p = JointTrajectoryPoint(positions=[float(x) for x in q])
        p.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(p)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pw, quat, seconds=3.0, seed=None):
        q = self.ik_tcp_world(pw, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED for {pw}")
            return None
        self.move_joints(q, seconds)
        got = self.tcp_world()
        print(f"  tcp now world={np.round(got, 4)} target={np.round(pw, 4)} err={np.linalg.norm(got - pw):.4f}")
        return q

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(self.grip_entry.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f


# Hand orientations (quaternions x,y,z,w) in base frame, hand Z pointing down.
Q_DOWN_Y = np.array([1.0, 0.0, 0.0, 0.0])          # fingers open along base Y
Q_DOWN_X = np.array([0.7071068, 0.7071068, 0.0, 0.0])  # fingers open along base X
