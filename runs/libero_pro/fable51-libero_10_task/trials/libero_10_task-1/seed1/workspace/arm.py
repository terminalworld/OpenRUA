#!/usr/bin/env python3
"""Small arm helper for this Panda (verified conventions):
 - /compute_ik: frame_id "" == WORLD frame; tip link is panda_link8
   (hand quat * Rz(+45deg)).
 - TCP = hand origin + 0.1034 along hand +Z (down for top-down grasps).

CLI:
  arm.py tcp X Y Z [yaw_deg] [secs]    move fingertip point to world XYZ, top-down,
                                       fingers separating along world y rotated by yaw
  arm.py grip W                        gripper per-finger width (0.04 open, 0.0 close)
  arm.py js                            print joints / finger gap / hand pose
"""
import math, sys, time
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
import rclpy.time

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034


def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2, w1*w2 - x1*x2 - y1*y2 - z1*z2)


def topdown_hand_q(yaw_deg):
    h = math.radians(yaw_deg) / 2
    return (math.cos(h), math.sin(h), 0.0, 0.0)   # Rz(yaw) * Rx(180)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grp = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.buf = Buffer(); TransformListener(self.buf, self.node)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
        while "m" not in self.js:
            self.spin(0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def hand_pose(self):
        for _ in range(50):
            self.spin(0.1)
            if self.buf.can_transform("world", "panda_hand", rclpy.time.Time()):
                break
        t = self.buf.lookup_transform("world", "panda_hand", rclpy.time.Time()).transform
        return ((t.translation.x, t.translation.y, t.translation.z),
                (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))

    def solve_ik(self, hand_pos_world, hand_q, seed=None):
        self.ik.wait_for_service(10)
        q8 = qmul(hand_q, (0, 0, 0.3826834, 0.9238795))
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = "panda_arm"; r.pose_stamped.header.frame_id = ""
        p = r.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = [float(v) for v in hand_pos_world]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = [float(v) for v in q8]
        cur = seed or self.joints(fresh=False)
        r.robot_state.joint_state.name = ARM
        r.robot_state.joint_state.position = [float(cur[j]) for j in ARM]
        r.timeout.sec = 1; r.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"IK failed ({code}) for {hand_pos_world}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, positions, secs):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        cur = self.joints()
        err = max(abs(cur[j] - p) for j, p in zip(ARM, positions))
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def tcp(self, x, y, z, yaw=0.0, secs=3.0):
        hq = topdown_hand_q(yaw)
        # hand +Z (approach) points down for top-down: hand origin is TCP above by TCP
        hand = (x, y, z + TCP)
        sol = self.solve_ik(hand, hq)
        code, err = self.move_joints(sol, secs)
        pos, q = self.hand_pose()
        print(f"  hand now world=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}) "
              f"tcp z={pos[2]-TCP:.4f}")
        return pos

    def grip(self, width):
        self.grp.wait_for_server(10)
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
        fut = self.grp.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        j = self.joints()
        gap = j["panda_finger_joint1"] - j["panda_finger_joint2"]
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def status(self):
        j = self.joints()
        print("joints:", ",".join(f"{j[n]:.4f}" for n in ARM))
        print(f"finger gap: {j['panda_finger_joint1'] - j['panda_finger_joint2']:.4f}")
        pos, q = self.hand_pose()
        print(f"hand world=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}) q=({q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f}) tcp z={pos[2]-TCP:.4f}")


if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "tcp":
        v = [float(s) for s in sys.argv[2:]]
        a.tcp(*v)
    elif cmd == "grip":
        a.grip(float(sys.argv[2]))
    elif cmd == "js":
        a.status()
    rclpy.shutdown()
