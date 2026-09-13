#!/usr/bin/env python3
"""Small control library for this Panda: FK, IK, trajectory, gripper.
World<->base conversion uses machine.yaml/TF fact: panda_link0 at
world (-0.51, 0, 0.42), no rotation.
"""
import sys, time
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
# verified: /compute_fk and /compute_ik on this machine use WORLD coords
# (FK of the current state matched the TF chain world->panda_hand)
BASE = np.array([0.0, 0.0, 0.0])
TCP = 0.1034


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("ctl")
        self._js = {}
        self.n.create_subscription(JointState, "/joint_states",
                                   lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.n.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.n, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.n, GripperCommand,
                                 "/franka_gripper/gripper_action")
        self.twist = self.n.create_publisher(TwistStamped,
                                             "/servo_node/delta_twist_cmds", 10)
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            rclpy.spin_once(self.n, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[a] for a in ARM]

    def finger_gap(self):
        j = self.joints()
        return abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"])

    def _seed(self):
        js = JointState()
        for a, p in zip(ARM, self.arm_q()):
            js.name.append(a); js.position.append(p)
        return js

    def hand_pose(self):
        """Hand frame pose in WORLD: (xyz, quat xyzw)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        r = fut.result()
        assert r is not None and r.error_code.val == 1, f"FK failed {r}"
        p = r.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z,
                      p.orientation.w])
        return xyz, q

    def tcp_pose(self):
        xyz, q = self.hand_pose()
        R = quat_R(*q)
        return xyz + TCP * R[:, 2], q

    def solve_ik(self, tcp_xyz_world, q, seed=None):
        """IK for hand so that TCP is at tcp_xyz_world with quaternion q."""
        R = quat_R(*q)
        hand = np.array(tcp_xyz_world) - TCP * R[:, 2] - BASE
        # verified: /compute_ik targets panda_link8, which is rotated -45deg
        # about z w.r.t. panda_hand (same origin). Convert hand quat -> link8.
        q = quat_mul(q, Q_HAND_TO_L8)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = seed or self._seed()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            log("IK failed", None if r is None else r.error_code.val)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[a] for a in ARM]

    def move_q(self, q, seconds=3.0, tol=0.02, retries=2):
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ARM
            pt = JointTrajectoryPoint(positions=[float(x) for x in q])
            pt.time_from_start = Duration(sec=int(seconds),
                                          nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            fut = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
            res = fut.result().get_result_async()
            rclpy.spin_until_future_complete(self.n, res, timeout_sec=600)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            log(f"move_q code={code} max_err={err:.4f}")
            if err < tol:
                return True
        return False

    def move_tcp(self, xyz, q, seconds=3.0):
        sol = self.solve_ik(xyz, q)
        if sol is None:
            return False
        ok = self.move_q(sol, seconds)
        p, _ = self.tcp_pose()
        log(f"tcp now {np.round(p, 4)} target {np.round(xyz, 4)} ok={ok}")
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res, timeout_sec=300)
        r = res.result().result
        log(f"gripper({width}) reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}")
        return r

    def servo(self, dx=0, dy=0, dz=0, ticks=20):
        msg = TwistStamped(); msg.header.frame_id = "panda_link0"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(dx), float(dy), float(dz)
        for _ in range(ticks):
            msg.header.stamp = self.n.get_clock().now().to_msg()
            self.twist.publish(msg); rclpy.spin_once(self.n, timeout_sec=0.05)
        p, _ = self.tcp_pose()
        log(f"servo done tcp {np.round(p, 4)}")

    def close(self):
        self.n.destroy_node(); rclpy.shutdown()


def quat_mul(a, b):
    """Hamilton product, xyzw convention: result = a * b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


# TF panda_link8 -> panda_hand is (0,0,-0.383,0.924); inverse is +45deg about z
Q_HAND_TO_L8 = np.array([0.0, 0.0, 0.3826834, 0.9238795])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


if __name__ == "__main__":
    c = Ctl()
    xyz, q = c.hand_pose()
    t, _ = c.tcp_pose()
    print("hand world", np.round(xyz, 4), "quat", np.round(q, 4))
    print("tcp world", np.round(t, 4))
    print("R", np.round(quat_R(*q), 3))
    print("joints", c.joints())
    c.close()
