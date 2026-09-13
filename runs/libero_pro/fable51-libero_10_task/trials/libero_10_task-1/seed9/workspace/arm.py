#!/usr/bin/env python3
"""Reusable arm helper: one node, IK/FJT/gripper clients built once.

Usage as CLI:
  python3 arm.py js                         # print joint state
  python3 arm.py ik x y z qx qy qz qw       # solve only, print joints
  python3 arm.py tcp x y z [yaw_deg] [secs] # move TCP (top-down) to world xyz
  python3 arm.py grip open|close
"""
import sys, time, math
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def topdown_quat(yaw_deg=0.0):
    """Hand z pointing down (world -z); yaw rotates finger axis about world z.
    yaw=0 -> fingers open along world y."""
    # q = rotz(yaw) * rotx(180deg)
    h = math.radians(yaw_deg) / 2
    qz = np.array([0, 0, math.sin(h), math.cos(h)])  # x,y,z,w
    qx = np.array([1, 0, 0, 0])
    # quaternion multiply qz * qx
    x1, y1, z1, w1 = qz; x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.update(dict(zip(m.name, m.position))), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def joints(self):
        self._js.clear()
        t = time.time()
        while len(self._js) < 9 and time.time() - t < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def solve_ik(self, x, y, z, q, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = M["frames"]["hand"]  # group tip is link8 (45deg off)
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        js = self.joints() if seed is None else seed
        s = JointState(); s.name = list(JOINTS); s.position = [float(js[j]) for j in JOINTS]
        req.ik_request.robot_state.joint_state = s
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        f = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        r = f.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"IK failed: {None if r is None else r.error_code.val}")
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move_joints(self, positions, seconds=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        f = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        js = self.joints()
        err = max(abs(js[j] - p) for j, p in zip(JOINTS, positions))
        print(f"  fjt error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, x, y, z, yaw_deg=0.0, seconds=3.0, seed=None):
        q = topdown_quat(yaw_deg)
        R = quat_R(*q)
        hx, hy, hz = np.array([x, y, z]) - TCP * R[:, 2]
        sol = self.solve_ik(hx, hy, hz, q, seed)
        print(f"  tcp->({x:.3f},{y:.3f},{z:.3f}) yaw={yaw_deg} joints={np.round(sol,3).tolist()}")
        return self.move_joints(sol, seconds)

    def gripper(self, open_):
        goal = GripperCommand.Goal()
        goal.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        goal.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        js = self.joints()
        print(f"  gripper {'open' if open_ else 'close'}: reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={js.get('panda_finger_joint1'):.4f},{js.get('panda_finger_joint2'):.4f}")
        return js


if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "js":
        print(a.joints())
    elif cmd == "ik":
        x, y, z, qx, qy, qz, qw = map(float, sys.argv[2:9])
        print(a.solve_ik(x, y, z, (qx, qy, qz, qw)))
    elif cmd == "tcp":
        x, y, z = map(float, sys.argv[2:5])
        yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
        secs = float(sys.argv[6]) if len(sys.argv) > 6 else 3.0
        a.move_tcp(x, y, z, yaw, secs)
    elif cmd == "grip":
        a.gripper(sys.argv[2] == "open")
    rclpy.shutdown()
