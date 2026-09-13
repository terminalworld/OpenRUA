"""Reusable controller: IK/FK, trajectory, gripper, joint state. World-frame API."""
import sys, time
import numpy as np, rclpy, yaml
from scipy.spatial.transform import Rotation as R
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open('/workspace/machine.yaml'))
FJT = next(a for a in M['actuators'] if a['kind'] == 'joint_trajectory')
GRIP = next(a for a in M['actuators'] if a['kind'] == 'gripper')
ARM = FJT['joints']
BASE_IN_WORLD = np.zeros(3)  # verified: /compute_fk and /compute_ik poses are WORLD frame here
TCP = float(M['hand']['tcp_offset_m'])
# link8 orientation for a top-down grasp with fingers closing along world y
Q_DOWN_Y = (0.92387953, -0.38268343, 0.0, 0.0)
# fingers closing along world x (yaw 90 deg)
Q_DOWN_X = (0.38268343, -0.92387953, 0.0, 0.0)


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('ctl')
        self._js = {}
        self.node.create_subscription(JointState, '/joint_states',
                                      lambda m: self._js.__setitem__('m', m), 10)
        self.ik = self.node.create_client(GetPositionIK, '/compute_ik')
        self.fk = self.node.create_client(GetPositionFK, '/compute_fk')
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT['port'])
        self.grip = ActionClient(self.node, GripperCommand, GRIP['port'])
        self.ik.wait_for_service(20); self.fk.wait_for_service(20)
        self.traj.wait_for_server(20); self.grip.wait_for_server(20)

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop('m', None)
        t0 = time.time()
        while 'm' not in self._js and time.time() - t0 < 30:
            self.spin(0.2)
        m = self._js['m']
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j['panda_finger_joint1'], j['panda_finger_joint2']

    def _seed(self, q=None):
        q = q or self.arm_q()
        js = JointState(); js.name = list(ARM); js.position = [float(v) for v in q]
        return js

    def solve_ik(self, xyz_world, quat, at_tcp=True, seed=None):
        p = np.array(xyz_world, float)
        if at_tcp:
            from scipy.spatial.transform import Rotation as R
            p = p - TCP * R.from_quat(quat).as_matrix()[:, 2]
        p = p - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M['planning']['group']
        req.ik_request.pose_stamped.header.frame_id = ''
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print('IK failed', None if res is None else res.error_code.val)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def fk_world(self, q=None, link='panda_link8'):
        req = GetPositionFK.Request()
        req.header.frame_id = ''
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        ps = res.pose_stamped[0].pose
        p = np.array([ps.position.x, ps.position.y, ps.position.z]) + BASE_IN_WORLD
        q = (ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w)
        return p, q

    def tcp_world(self):
        from scipy.spatial.transform import Rotation as R
        p, q = self.fk_world()
        return p + TCP * R.from_quat(q).as_matrix()[:, 2]

    def move(self, q, seconds=3.0, retries=2, tol=0.02):
        """Send a single-point trajectory; verify; resend on tolerance violation."""
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            f = self.traj.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, f, timeout_sec=120)
            rf = f.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
            code = rf.result().result.error_code
            cur = self.arm_q()
            err = max(abs(a - b) for a, b in zip(cur, q))
            print(f'  move: code={code} max_err={err:.4f}')
            if err < tol:
                return True
        return False

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP['max_effort'])
        f = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        fg = self.fingers()
        print(f'  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={fg[0]:.4f},{fg[1]:.4f}')
        return fg

    def goto(self, xyz, quat=Q_DOWN_Y, seconds=3.0, at_tcp=True):
        q = self.solve_ik(xyz, quat, at_tcp)
        if q is None:
            return False
        ok = self.move(q, seconds)
        tcp = self.tcp_world()
        print(f'  tcp now {tcp.round(4)} target {np.round(xyz,4)}')
        return ok
