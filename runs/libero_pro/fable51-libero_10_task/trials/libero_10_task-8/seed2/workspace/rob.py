#!/usr/bin/env python3
"""Robot helper: joint state, FK/IK (MoveIt), trajectories, gripper.

World <-> base: base = world - BASE_IN_WORLD.  IK/FK work in the base frame.
"""
import math, sys, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # MoveIt model frame is world here (verified via FK of panda_link0)
TCP = M["hand"]["tcp_offset_m"]


def R_to_quat(R):
    m = R
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s; x = (m[2, 1] - m[1, 2]) / s; y = (m[0, 2] - m[2, 0]) / s; z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s; x = 0.25 * s; y = (m[0, 1] + m[1, 0]) / s; z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s; x = (m[0, 1] + m[1, 0]) / s; y = 0.25 * s; z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s; x = (m[0, 2] + m[2, 0]) / s; y = (m[1, 2] + m[2, 1]) / s; z = 0.25 * s
    return np.array([x, y, z, w])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def frame_from_axes(Yh, Zh):
    """Rotation whose hand-Y (finger axis) and hand-Z (approach) are given."""
    Yh = np.asarray(Yh, float); Yh /= np.linalg.norm(Yh)
    Zh = np.asarray(Zh, float); Zh /= np.linalg.norm(Zh)
    Xh = np.cross(Yh, Zh)
    return np.column_stack([Xh, Yh, Zh])


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.update(zip(m.name, m.position)), 1)
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.joints()

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        self._js.clear()
        while not all(j in self._js for j in ARM):
            self.spin(0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return np.array([js[j] for j in ARM])

    def finger_gap(self):
        js = self.joints()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    # ---------- kinematics (base frame) ----------
    def fk(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = list(map(float, self.arm_q() if q is None else q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def fk_world(self, q=None):
        p, quat = self.fk(q)
        return p + BASE_IN_WORLD, quat

    def ik(self, p_base, quat, seed=None, timeout=5.0, attempts=1):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"   # group tip is panda_link8 (45 deg off from the hand)
        req.ik_request.pose_stamped.header.frame_id = ""
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        seed = self.arm_q() if seed is None else np.asarray(seed, float)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = list(map(float, seed))
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    def ik_world(self, p_world, quat, **kw):
        return self.ik(np.asarray(p_world) - BASE_IN_WORLD, quat, **kw)

    # ---------- motion ----------
    def move_joints(self, q_list, times):
        """Send one trajectory through q_list (list of 7-vectors) at cumulative times."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(q_list, times):
            pt = JointTrajectoryPoint(positions=list(map(float, q)))
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
        q_now = self.arm_q()
        err = np.abs(q_now - np.asarray(q_list[-1])).max()
        print(f"  traj done code={code} max|dq|={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        gap = self.finger_gap()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap


if __name__ == "__main__":
    r = Robot()
    q = r.arm_q()
    print("q", np.round(q, 4))
    p, quat = r.fk_world()
    print("hand world", np.round(p, 4), "quat", np.round(quat, 4))
    print("R\n", np.round(quat_to_R(quat), 3))
    print("finger gap", r.finger_gap())
