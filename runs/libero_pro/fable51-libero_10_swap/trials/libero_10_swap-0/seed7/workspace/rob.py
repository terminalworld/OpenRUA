"""Shared robot helper: one node, reusable IK/FK/FJT/gripper clients.

Coordinates given to move_* are WORLD frame; converted to the arm base
(planning) frame using the static world->panda_link0 transform.
"""
import time
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
BASE_IN_WORLD = np.zeros(3)  # MoveIt model frame == world (verified by FK vs camera TF)
TCP = float(M["hand"]["tcp_offset_m"])
# hand pointing straight down, fingers closing along world y
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


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


def q_down_yaw(yaw):
    """hand z down, then rotated about world z by yaw (rad)."""
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    return quat_mul((0.0, 0.0, s, c), Q_DOWN)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"
        assert self.fjt.wait_for_server(10), "no FJT"
        assert self.grip.wait_for_server(10), "no gripper"

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 20:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def hand_pose(self, q=None):
        """world-frame (xyz, quat) of panda_hand for arm config q (default current)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_pose(self):
        xyz, q = self.hand_pose()
        return xyz + TCP * quat_to_R(*q)[:, 2], q

    # ---------- IK ----------
    def solve_ik(self, xyz_world, quat, at_tcp=True, seed=None, timeout=60):
        xyz = np.array(xyz_world, float)
        if at_tcp:
            xyz = xyz - TCP * quat_to_R(*quat)[:, 2]
        xyz = xyz - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            raise RuntimeError(f"IK failed code={res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, waypoints=None, retries=4, tol=0.01):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if waypoints:
            for wq, wt in waypoints:
                pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
                pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        cur = np.array(self.arm_q())
        err = np.abs(cur - np.array(q)).max()
        print(f"  fjt code={code} max_joint_err={err:.4f}")
        if err > tol and retries > 0:
            # joint7 on this machine is slow; re-send with time for the residual
            return self.move_q(q, max(2.0, err / 0.15), retries=retries - 1, tol=tol)
        return code, err

    def move_tcp(self, xyz, quat=Q_DOWN, seconds=3.0, seed=None):
        q = self.solve_ik(xyz, quat, at_tcp=True, seed=seed)
        code, err = self.move_q(q, seconds)
        p, _ = self.tcp_pose()
        print(f"  tcp now {p.round(4)} target {np.array(xyz).round(4)} d={np.linalg.norm(p-xyz):.4f}")
        return p

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        g = self.finger_gap()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={g}")
        return g

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
