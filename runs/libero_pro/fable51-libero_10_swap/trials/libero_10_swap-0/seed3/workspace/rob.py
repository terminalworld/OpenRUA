"""Small helper library: one node, reusable clients. World-frame API.

Usage from a script:
    from rob import Robot
    r = Robot()
    r.gripper(0.04)
    r.move_tcp(x, y, z, yaw=0.0, secs=3)   # top-down grasp orientation
    print(r.hand_pose(), r.fingers())
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

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (from /tf)


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def topdown_quat(yaw):
    """Hand z pointing down (-world z), fingers closing along world y
    rotated by yaw about z. q = Rz(yaw) * (1,0,0,0)."""
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    # Rz(yaw) = (0,0,s,c); (1,0,0,0) ; product (w1w2 - v1.v2, w1 v2 + w2 v1 + v1 x v2)
    # q1=(0,0,s,c) q2=(1,0,0,0)
    w = c * 0 - (0 * 1 + 0 * 0 + s * 0)
    v = c * np.array([1, 0, 0]) + 0 * np.array([0, 0, s]) + np.cross([0, 0, s], [1, 0, 0])
    return (v[0], v[1], v[2], w)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        assert self.fjt.wait_for_server(timeout_sec=20), "no FJT server"
        assert self.grip.wait_for_server(timeout_sec=20), "no gripper server"
        assert self.ik.wait_for_service(timeout_sec=20), "no IK service"
        self.spin(0.5)
        log("robot ready")

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---- sensing ----
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
            while "m" not in self._js:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def hand_pose(self):
        """world -> panda_hand (t, q) from TF; TCP point also returned."""
        for _ in range(50):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.tfbuf.can_transform("world", "panda_hand", rclpy.time.Time()):
                break
        t = self.tfbuf.lookup_transform("world", "panda_hand", rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        p = np.array([tr.x, tr.y, tr.z])
        R = quat_to_R(q.x, q.y, q.z, q.w)
        tcp = p + TCP * R[:, 2]
        return p, (q.x, q.y, q.z, q.w), tcp

    # ---- IK ----
    def solve_ik(self, x, y, z, quat, at_tcp=True, seed=None, timeout=60):
        """x,y,z in WORLD; returns joint list (manifest order) or None."""
        p = np.array([x, y, z], float)
        if at_tcp:
            R = quat_to_R(*quat)
            p = p - TCP * R[:, 2]
        # machine facts (measured): IK poses are in WORLD on this machine,
        # and the group's tip link is panda_link8 = hand rotated +45deg
        # about z (link8->hand static TF is Rz(-45deg)).
        q8 = qmul(quat, (0.0, 0.0, 0.38268343, 0.92387953))
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = "world"
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q8)
        js = JointState()
        seed = seed if seed is not None else self.arm_q()
        js.name = list(ARM)
        js.position = [float(v) for v in seed]
        req.ik_request.robot_state.joint_state = js
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            log("IK: no answer")
            return None
        if res.error_code.val != 1:
            log(f"IK failed code={res.error_code.val} for world {x:.3f},{y:.3f},{z:.3f}")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in ARM]

    # ---- motion ----
    def move_joints(self, q, secs=3.0, retries=1):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        for attempt in range(retries + 1):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            cur = self.arm_q()
            err = max(abs(a - b) for a, b in zip(cur, q))
            log(f"traj code={code} max_joint_err={err:.4f}")
            if code == 0 and err < 0.02:
                return True
            if attempt < retries:
                log("retrying trajectory")
        return err < 0.05

    def move_tcp(self, x, y, z, yaw=0.0, secs=3.0, quat=None):
        quat = quat or topdown_quat(yaw)
        q = self.solve_ik(x, y, z, quat)
        if q is None:
            return False
        ok = self.move_joints(q, secs)
        _, _, tcp = self.hand_pose()
        log(f"tcp now {tcp.round(4)} target {np.array([x, y, z]).round(4)} "
            f"err={np.linalg.norm(tcp - [x, y, z]):.4f}")
        return ok

    def move_tcp_line(self, x, y, z, yaw=0.0, secs=3.0, n=4, quat=None):
        """Straight-line TCP motion from the current TCP to (x,y,z): IK on
        n waypoints (each seeded with the previous) in one trajectory."""
        quat = quat or topdown_quat(yaw)
        _, _, tcp0 = self.hand_pose()
        target = np.array([x, y, z], float)
        seed = self.arm_q()
        qs = []
        for i in range(1, n + 1):
            wp = tcp0 + (target - tcp0) * i / n
            q = self.solve_ik(*wp, quat, seed=seed)
            if q is None:
                return False
            qs.append(q)
            seed = q
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for i, q in enumerate(qs, 1):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            t = secs * i / n
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        cur = self.arm_q()
        err = max(abs(a - b) for a, b in zip(cur, qs[-1]))
        _, _, tcp = self.hand_pose()
        log(f"line traj code={code} max_joint_err={err:.4f} tcp now {tcp.round(4)} "
            f"target {target.round(4)} err={np.linalg.norm(tcp - target):.4f}")
        return code == 0 and err < 0.05

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        self.spin(0.3)
        f = self.fingers()
        log(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f


def quat_from_axes(zaxis, yaxis):
    """Quaternion (x,y,z,w) of a hand frame with the given world-frame
    approach axis (hand z) and finger-closing axis (hand y)."""
    z = np.asarray(zaxis, float); z /= np.linalg.norm(z)
    y = np.asarray(yaxis, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    R = np.column_stack([x, y, z])
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def move_tcp_iter(r, x, y, z, quat, secs=3.0, tries=4, tol=0.005):
    """Repeat IK->trajectory with error compensation until TCP within tol."""
    target = np.array([x, y, z], float)
    goal = target.copy()
    for i in range(tries):
        q = r.solve_ik(*goal, quat)
        if q is None:
            log("iter: IK failed"); return False
        r.move_joints(q, secs, retries=0)
        cur = np.array(r.arm_q())
        _, _, tcp = r.hand_pose()
        err = tcp - target
        log(f"iter {i}: tcp {tcp.round(4)} err {err.round(4)} |err|={np.linalg.norm(err):.4f} "
            f"joint err {np.round(cur - np.array(q), 3)}")
        if np.linalg.norm(err) < tol:
            return True
        goal = goal - err  # compensate the residual
    return False
