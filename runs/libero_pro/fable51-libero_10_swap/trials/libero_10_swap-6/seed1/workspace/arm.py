"""Small helper library: joint state, FK/IK (world frame on this machine), trajectories, gripper."""
import time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped

ARM = [f"panda_joint{i}" for i in range(1, 8)]
Q_DOWN = (1.0, 0.0, 0.0, 0.0)          # hand z down, fingers along world y
Q_DOWN_X = (0.7071, 0.7071, 0.0, 0.0)  # hand z down, fingers along world x

def R_of(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

class Arm:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.traj = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js = dict(zip(m.name, m.position))
        self._js_stamp = time.time()

    def joints(self, fresh=True):
        t0 = time.time()
        if fresh:
            self._js = {}
        while len(self._js) < 9 and time.time() - t0 < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request(); req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z]), (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik(self, xyz, quat=Q_DOWN, seed=None, tcp=False):
        """World-frame hand pose -> joint list, or None. tcp=True: xyz is the fingertip centre."""
        xyz = np.array(xyz, dtype=float)
        if tcp:
            xyz = xyz - 0.1034 * R_of(quat)[:, 2]
        seed = seed if seed is not None else self.arm_q()
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = "panda_arm"; r.ik_link_name = "panda_hand"; r.avoid_collisions = False
        r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = map(float, xyz)
        (r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y,
         r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w) = map(float, quat)
        r.robot_state.joint_state.name = ARM
        r.robot_state.joint_state.position = [float(v) for v in seed]
        r.timeout = Duration(sec=2)
        fut = self.ik_cli.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK failed for {np.round(xyz,3)}: {None if res is None else res.error_code.val}")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, waypoints, seconds, verbose=True):
        """waypoints: list of 7-lists (or one), seconds: total (evenly spaced)."""
        if not isinstance(waypoints[0], (list, tuple, np.ndarray)):
            waypoints = [waypoints]
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = ARM
        n = len(waypoints)
        for i, wp in enumerate(waypoints):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.traj.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q = self.arm_q(); err = np.abs(np.array(q) - np.array(waypoints[-1])).max()
        if verbose: print(f"traj done code={code} max joint err={err:.4f}")
        return code, err

    def move_to(self, xyz, quat=Q_DOWN, seconds=3.0, tcp=False, via=None):
        """IK + trajectory. via: optional list of intermediate xyz (same quat)."""
        pts = list(via or []) + [xyz]
        seed = self.arm_q(); wps = []
        for p in pts:
            q = self.ik(p, quat, seed=seed, tcp=tcp)
            if q is None: return None
            wps.append(q); seed = q
        code, err = self.move_joints(wps, seconds)
        pos, _ = self.fk()
        print(f"hand now at {np.round(pos,4)}")
        return pos

    def gripper(self, width):
        goal = GripperCommand.Goal(); goal.command.position = float(width); goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def servo(self, lin, ticks, ang=(0, 0, 0)):
        msg = TwistStamped(); msg.header.frame_id = "panda_link0"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg); rclpy.spin_once(self.node, timeout_sec=0.05)
