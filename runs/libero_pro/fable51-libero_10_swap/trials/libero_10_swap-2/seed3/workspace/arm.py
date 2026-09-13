#!/usr/bin/env python3
"""World-frame arm helper for this Panda (base at world (-0.66, 0, 0.912)).

  arm.py fk                                  print TCP pose in world
  arm.py goto x y z qx qy qz qw [secs] [--tcp]   IK -> trajectory -> FK check
  arm.py joints j1,...,j7 secs
  arm.py rot7 delta secs                     rotate joint7 by delta (rad)
  arm.py servo dx dy dz [secs]               cartesian twist burst (world m)
"""
import sys
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
JOINTS = FJT["joints"]
# verified: /compute_fk and /compute_ik on this machine use the WORLD frame
# (hand FK == TF world->panda_hand), so no base offset is needed.
BASE = np.array([0.0, 0.0, 0.0])
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    """Hamilton product a*b, quaternions as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fkc = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.twist = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    def _js(self, m):
        self.js = m

    def joint_state(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return self.js

    def arm_positions(self):
        js = self.joint_state()
        d = dict(zip(js.name, js.position))
        return [d[j] for j in JOINTS], d

    def seed(self):
        q, _ = self.arm_positions()
        s = JointState(); s.name = list(JOINTS); s.position = list(q)
        return s

    def fk(self):
        self.fkc.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.seed()
        fut = self.fkc.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        R = quat_R(*q)
        tcp = pos + TCP * R[:, 2]
        return pos, tcp, q

    def solve_ik(self, world_xyz, quat, at_tcp=True):
        self.ik.wait_for_service(10)
        R = quat_R(*quat)
        p = np.array(world_xyz, float)
        if at_tcp:
            p = p - TCP * R[:, 2]
        p = p - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.pose_stamped.pose.position.x = float(p[0])
        req.ik_request.pose_stamped.pose.position.y = float(p[1])
        req.ik_request.pose_stamped.pose.position.z = float(p[2])
        # the IK tip is panda_link8, which sits 45 deg (about z) from
        # panda_hand: request link8 = hand * Rz(+45deg)
        o = req.ik_request.pose_stamped.pose.orientation
        o.x, o.y, o.z, o.w = map(float, quat_mul(quat, (0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8))))
        req.ik_request.robot_state.joint_state = self.seed()
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"IK failed: {None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move_joints(self, q, secs):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        cur, _ = self.arm_positions()
        err = np.abs(np.array(cur) - np.array(q)).max()
        print(f"trajectory error_code={code} max_joint_err={err:.4f}")
        return code

    def goto(self, xyz, quat, secs=4.0, at_tcp=True):
        q = self.solve_ik(xyz, quat, at_tcp)
        print("IK:", np.round(q, 4).tolist())
        self.move_joints(q, secs)
        self.report()

    def servo(self, dxyz, secs=1.0, rate=20):
        msg = TwistStamped(); msg.header.frame_id = "panda_link0"
        v = np.array(dxyz, float) / secs
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(int(secs * rate)):
            self.twist.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=1.0 / rate)
        self.report()

    def report(self):
        pos, tcp, q = self.fk()
        _, d = self.arm_positions()
        print(f"hand world={np.round(pos,4).tolist()} tcp world={np.round(tcp,4).tolist()} quat={np.round(q,4).tolist()}")
        print(f"fingers={d['panda_finger_joint1']:.4f},{d['panda_finger_joint2']:.4f}")


def main():
    a = sys.argv[1:]
    arm = Arm()
    cmd = a[0]
    if cmd == "fk":
        arm.report()
    elif cmd == "goto":
        nums = [float(x) for x in a[1:] if not x.startswith("--")]
        secs = nums[7] if len(nums) > 7 else 4.0
        arm.goto(nums[:3], nums[3:7], secs, at_tcp="--hand" not in a)
    elif cmd == "joints":
        q = [float(x) for x in a[1].split(",")]
        arm.move_joints(q, float(a[2])); arm.report()
    elif cmd == "rot7":
        q, _ = arm.arm_positions()
        q[6] += float(a[1])
        arm.move_joints(q, float(a[2])); arm.report()
    elif cmd == "servo":
        d = [float(x) for x in a[1:4]]
        arm.servo(d, float(a[4]) if len(a) > 4 else 1.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
