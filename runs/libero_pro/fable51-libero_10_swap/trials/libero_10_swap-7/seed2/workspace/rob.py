#!/usr/bin/env python3
"""Small helper around this machine's ports (see machine.yaml).

  python3 rob.py state                         joints + hand/tcp pose (world)
  python3 rob.py goto X Y Z [SECS] [YAW_DEG]   tcp -> world point, hand Z down
  python3 rob.py joints p1,...,p7 SECS         raw joint trajectory
  python3 rob.py grip WIDTH                    per-finger width (0.04 open, 0 closed)
"""
import sys, time
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
# Verified empirically: /compute_fk and /compute_ik on this machine take/give
# poses that match TF `world` (FK of panda_hand == TF world->panda_hand), so
# no base offset is applied.
BASE_W = np.array([0.0, 0.0, 0.0])
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def topdown_quat(yaw_deg=0.0):
    """Hand Z down; yaw rotates the finger axis about world Z (0 -> fingers along Y)."""
    # q = Rz(yaw) * Rx(pi)
    h = np.radians(yaw_deg) / 2
    qz = np.array([0, 0, np.sin(h), np.cos(h)])
    qx = np.array([1.0, 0, 0, 0])
    x1, y1, z1, w1 = qz; x2, y2, z2, w2 = qx
    return np.array([w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2,
                     w1*w2 - x1*x2 - y1*y2 - z1*z2])


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, n=1, t=0.1):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin()
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_positions(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def wrench(self):
        self._wr.pop("m", None)
        end = time.time() + 3
        while "m" not in self._wr and time.time() < end:
            self.spin()
        if "m" not in self._wr:
            return None
        f = self._wr["m"].wrench.force
        return (f.x, f.y, f.z)

    def hand_pose(self):
        """FK of panda_hand in world (base offset added)."""
        self.fk.wait_for_service(5)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        js = JointState()
        j = self.joints()
        for n in ARM:
            js.name.append(n); js.position.append(j[n])
        req.robot_state.joint_state = js
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP * quat_to_R(*q)[:, 2]
        return pos, q, tcp

    def solve_ik(self, tcp_world, quat, seed=None):
        """Joint solution putting the TCP at tcp_world with hand orientation quat."""
        self.ik.wait_for_service(5)
        R = quat_to_R(*quat)
        hand_w = np.array(tcp_world) - TCP * R[:, 2]
        hand_b = hand_w - BASE_W
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"   # group tip is link8 (45deg yaw off)
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        js = JointState()
        seed = seed if seed is not None else self.arm_positions()
        for n, v in zip(ARM, seed):
            js.name.append(n); js.position.append(float(v))
        req.ik_request.robot_state.joint_state = js
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            raise RuntimeError("IK no answer")
        if r.error_code.val != 1:
            raise RuntimeError(f"IK failed code={r.error_code.val}")
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[n] for n in ARM]

    def move_joints(self, positions, seconds, via=None):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
            for i, (pos, t) in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in pos])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        cur = self.arm_positions()
        err = max(abs(a - b) for a, b in zip(cur, positions))
        print(f"traj done error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def goto(self, tcp_world, seconds=4.0, yaw_deg=0.0, seed=None):
        q = topdown_quat(yaw_deg)
        sol = self.solve_ik(tcp_world, q, seed)
        print("ik sol", np.round(sol, 4).tolist(), flush=True)
        code, err = self.move_joints(sol, seconds)
        pose = self.hand_pose()
        if pose:
            print(f"tcp now {np.round(pose[2], 4).tolist()} target {list(tcp_world)}", flush=True)
        return code, err

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        print(f"gripper reached_goal={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}", flush=True)
        return r

    def report(self):
        j = self.joints()
        print("arm", {n: round(j[n], 4) for n in ARM})
        print("fingers", self.fingers())
        pose = self.hand_pose()
        if pose:
            print("hand", np.round(pose[0], 4).tolist(), "q", np.round(pose[1], 4).tolist(),
                  "tcp", np.round(pose[2], 4).tolist())
        print("wrench", self.wrench())

    def close(self):
        self.node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    a = sys.argv[1:]
    r = Robot()
    try:
        if a[0] == "state":
            r.report()
        elif a[0] == "goto":
            xyz = [float(v) for v in a[1:4]]
            secs = float(a[4]) if len(a) > 4 else 4.0
            yaw = float(a[5]) if len(a) > 5 else 0.0
            r.goto(xyz, secs, yaw)
        elif a[0] == "joints":
            r.move_joints([float(v) for v in a[1].split(",")], float(a[2]))
        elif a[0] == "grip":
            r.gripper(float(a[1]))
        else:
            print(__doc__)
    finally:
        r.close()
