#!/usr/bin/env python3
"""Small helper library for this session: IK moves, servo bursts, gripper,
sensing. Base frame = panda_link0 (world = base + (-0.75, 0, 0.912))."""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import TwistStamped, WrenchStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
W2B = np.array([0.75, 0.0, -0.912])  # world -> base translation


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rb")
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._js_cb, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._wr_cb, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _js_cb(self, m): self.js = m
    def _wr_cb(self, m): self.wr = m

    def spin(self, n=3):
        for _ in range(n): rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self.spin(); self.js = None
        while self.js is None: rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints(); return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self.wr = None
        while self.wr is None: rclpy.spin_once(self.node, timeout_sec=0.2)
        f = self.wr.wrench.force; t = self.wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def hand_pose(self):
        """hand pose in base frame via FK: (xyz, quat xyzw)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        q = self.arm_q()
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = q
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z,
                          p.orientation.w]))

    def solve_ik(self, xyz, quat, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        (p.orientation.x, p.orientation.y, p.orientation.z,
         p.orientation.w) = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = list(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name,
                       r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def traj(self, points, seconds):
        """points: list of joint vectors; seconds: list of cumulative times."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(points, seconds):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q = np.array(self.arm_q()); err = np.abs(q - np.array(points[-1])).max()
        print(f"traj code={code} max_joint_err={err:.4f}")
        return code, err

    def move_to(self, xyz, quat, seconds=3.0, via=None):
        """IK for the pose, trajectory there. via: optional list of
        intermediate (xyz) poses with same quat for a multi-point pass."""
        pts, ts, seed = [], [], self.arm_q()
        targets = (via or []) + [xyz]
        for i, t in enumerate(targets):
            q = self.solve_ik(t, quat, seed)
            if q is None:
                print(f"IK FAILED for {t}"); return False
            pts.append(q); seed = q
            ts.append(seconds * (i + 1) / len(targets))
        code, err = self.traj(pts, ts)
        tries = 0
        while err > 0.02 and tries < 3:  # controller lag: resend final point
            tries += 1
            code, err = self.traj([pts[-1]], [max(2.0, seconds / 2)])
        return err < 0.02

    def servo(self, v, n, dt=0.05):
        """stream twist v=(vx,vy,vz) m/s in base frame for n ticks."""
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=dt)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={self.fingers()}")
        return r

    def close(self):
        self.node.destroy_node(); rclpy.shutdown()


def w2b(p):
    return np.array(p) + W2B


def b2w(p):
    return np.array(p) - W2B


def qmul(a, b):
    """quaternion product a⊗b, xyzw."""
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw*bx + bx*0 + ax*bw + (ay*bz - az*by),
                     aw*by + ay*bw + (az*bx - ax*bz),
                     aw*bz + az*bw + (ax*by - ay*bx),
                     aw*bw - (ax*bx + ay*by + az*bz)])


RZ45 = np.array([0, 0, np.sin(np.pi/8), np.cos(np.pi/8)])


def hand2link8(q_hand):
    """IK tip link is panda_link8 = hand ⊗ Rz(+45°)."""
    return qmul(np.array(q_hand, float), RZ45)


def q_down(yaw_deg=0.0):
    """hand pointing straight down, fingers closing along world y rotated
    by yaw about z (yaw=0: fingers along y; yaw=90: along x)."""
    h = np.radians(yaw_deg) / 2
    return qmul(np.array([0, 0, np.sin(h), np.cos(h)]), np.array([1., 0, 0, 0]))


def q_axis(axis, deg):
    """quaternion (xyzw) for rotation of deg about world axis."""
    ax = np.array(axis, float); ax /= np.linalg.norm(ax)
    h = np.radians(deg) / 2
    return np.concatenate([ax * np.sin(h), [np.cos(h)]])


def q2R(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
