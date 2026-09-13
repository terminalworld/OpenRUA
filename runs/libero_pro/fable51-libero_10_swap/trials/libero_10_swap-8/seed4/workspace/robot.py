"""Reusable helpers: joint state, FK, IK, trajectory, gripper, TF.
All poses are WORLD frame unless noted; the planner works in panda_link0
which sits at BASE = (-0.66, 0, 0.912) with identity rotation."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open('/workspace/machine.yaml'))
FJT = next(a for a in M['actuators'] if a['kind'] == 'joint_trajectory')
GRIP = next(a for a in M['actuators'] if a['kind'] == 'gripper')
ARM = FJT['joints']
# Verified: FK/IK poses on this machine are already WORLD-frame (TF
# panda_link0->panda_hand == FK - (-0.66,0,0.912)); no offset to apply.
BASE = np.zeros(3)
LINK0 = np.array([-0.66, 0.0, 0.912])
TCP = M['hand']['tcp_offset_m']


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_quat(R):
    # from rotation matrix to xyzw
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def grasp_R(yaw=0.0, pitch=0.0):
    """Hand rotation: z down; fingers open along world y rotated by yaw
    about z; then pitched by `pitch` about the world y axis (positive =
    hand leans toward -x / the robot)."""
    R = rot_x(np.pi)          # z down, hand x = world x, hand y = -world y
    R = rot_z(yaw) @ R
    R = rot_y(pitch) @ R
    return R


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('robot_helper')
        self._js = {}
        self.node.create_subscription(JointState, '/joint_states',
                                      lambda m: self._js.__setitem__('m', m), 1)
        self.ik_cli = self.node.create_client(GetPositionIK, '/compute_ik')
        self.fk_cli = self.node.create_client(GetPositionFK, '/compute_fk')
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT['port'])
        self.grip = ActionClient(self.node, GripperCommand, GRIP['port'])
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop('m', None)
        t0 = time.time()
        while 'm' not in self._js and time.time() - t0 < 10:
            self.spin(0.2)
        m = self._js['m']
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return np.array([j[n] for n in ARM])

    def finger_gap(self):
        j = self.joints()
        return j['panda_finger_joint1'] - j['panda_finger_joint2']

    def _seed(self, q):
        js = JointState(); js.name = list(ARM); js.position = [float(v) for v in q]
        rs = RobotState(); rs.joint_state = js
        return rs

    def fk(self, q, link='panda_hand'):
        """World-frame (pos, R) of link for arm config q."""
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f'FK failed: {res and res.error_code.val}')
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q_ = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        return pos, quat_R(q_)

    def ik(self, pos, R, seed=None, timeout=5.0, attempts=1):
        """IK for panda_hand at world pos with rotation R. Returns q or None."""
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M['planning']['group']
        req.ik_request.pose_stamped.header.frame_id = ''
        req.ik_request.ik_link_name = 'panda_hand'
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos, float) - BASE
        p.position.x, p.position.y, p.position.z = map(float, pb)
        qx, qy, qz, qw = R_quat(R)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, (qx, qy, qz, qw))
        req.ik_request.robot_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    def ik_tcp(self, tcp_pos, R, **kw):
        """IK with the fingertip centre (TCP) at tcp_pos."""
        hand = np.asarray(tcp_pos, float) - TCP * R[:, 2]
        return self.ik(hand, R, **kw)

    def ik_best(self, tcp_pos, R, prev=None, n_random=60, w_cont=1.0,
                min_margin=0.15, rng=np.random.default_rng(0)):
        """Multi-start IK: many seeds, keep the solution with good joint
        limit margin and (if prev given) small joint-space distance."""
        lim = np.array(FJT['limits_rad'])
        seeds = []
        if prev is not None:
            seeds.append(np.asarray(prev))
        seeds.append(self.arm_q())
        for _ in range(n_random):
            seeds.append(rng.uniform(lim[:, 0] + 0.2, lim[:, 1] - 0.2))
        best, best_cost = None, None
        for sd in seeds:
            s = self.ik_tcp(tcp_pos, R, seed=sd, timeout=0.05)
            if s is None:
                continue
            margin = np.minimum(s - lim[:, 0], lim[:, 1] - s).min()
            cost = max(0.0, min_margin - margin) * 20.0
            if prev is not None:
                cost += w_cont * np.abs(s - prev).max()
            if best is None or cost < best_cost:
                best, best_cost = s, cost
        return best

    def move(self, q_list, durations):
        """Send a multi-point trajectory; returns error_code."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(q_list, durations):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        return rf.result().result.error_code

    def move_to(self, q, t=3.0):
        code = self.move([q], [t])
        err = np.abs(self.arm_q() - q).max()
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP['max_effort'])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        return r.reached_goal, r.stalled, r.position
