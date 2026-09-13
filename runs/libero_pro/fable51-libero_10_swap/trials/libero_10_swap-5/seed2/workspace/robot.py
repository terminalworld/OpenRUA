"""Small helper layer over the manifest ports: joints, FK, IK, trajectory, gripper."""
import math
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down_quat(theta):
    """Hand pointing straight down (hand Z = world -Z), yawed by theta about world Z.
    theta=0: hand X = world X, hand Y = world -Y."""
    return (math.cos(theta / 2), math.sin(theta / 2), 0.0, 0.0)


class Robot:
    def __init__(self, name="robot_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.n = rclpy.create_node(name)
        self._js = {}
        self.n.create_subscription(JointState, "/joint_states",
                                   self._on_js, 1)
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.traj = ActionClient(self.n, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.fk.wait_for_service(10); self.ik.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, m):
        self._js["m"] = m

    def joints(self, fresh=True):
        if fresh:
            self._js.clear()
        while "m" not in self._js:
            rclpy.spin_once(self.n, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_state(self):
        js = self.joints()
        s = JointState()
        for j in ARM:
            s.name.append(j); s.position.append(js[j])
        return s

    def fk_pose(self, positions=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        if positions is None:
            req.robot_state.joint_state = self.arm_state()
        else:
            req.robot_state.joint_state.name = list(ARM)
            req.robot_state.joint_state.position = [float(p) for p in positions]
        f = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=30)
        r = f.result()
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def tcp_from_hand(self, pos, quat):
        R = quat_R(*quat)
        return np.asarray(pos) + TCP * R[:, 2]

    def ik_solve(self, pos, quat, at_tcp=True, seed=None):
        """Return arm joint positions (manifest order) or None."""
        pos = np.asarray(pos, float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = seed or self.arm_state()
        req.ik_request.avoid_collisions = False
        f = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=60)
        r = f.result()
        if r is None or r.error_code.val != 1:
            print("IK failed:", None if r is None else r.error_code.val)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, positions, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [positions]
        for i, wp in enumerate(wps, 1):
            t = seconds * i / len(wps)
            pt = JointTrajectoryPoint(positions=[float(x) for x in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        f = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, f)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf)
        code = rf.result().result.error_code
        js = self.joints()
        err = max(abs(js[j] - p) for j, p in zip(ARM, positions))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, at_tcp=True):
        sol = self.ik_solve(pos, quat, at_tcp=at_tcp)
        if sol is None:
            return None
        code, err = self.move_joints(sol, seconds)
        hp, hq = self.fk_pose()
        tcp = self.tcp_from_hand(hp, hq)
        print("hand", hp.round(4), "tcp", tcp.round(4), "quat", np.round(hq, 4))
        return tcp

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=30)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=120)
        r = rf.result().result
        js = self.joints()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={js.get('panda_finger_joint1'):.4f},{js.get('panda_finger_joint2'):.4f}")
        return js
