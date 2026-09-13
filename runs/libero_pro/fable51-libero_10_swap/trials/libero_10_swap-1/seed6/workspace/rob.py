#!/usr/bin/env python3
"""Small control helper: one node, clients built once, act -> verify.

Usage as a library (see run_*.py) or:
  python3 rob.py fk                      # print hand + tcp pose in base frame
  python3 rob.py grip <w>                # gripper per-finger width
  python3 rob.py tcp <x> <y> <z> [sec]   # IK (top-down, fingers along y) + FJT
"""
import sys, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_W = np.array([-0.51, 0.0, 0.42])   # panda_link0 in world (from /tf)
# top-down grasp orientation: hand z = -world z, hand x = world x
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_yaw_down(yaw):
    """Top-down orientation with the hand x axis rotated by yaw about world z."""
    # R = Rz(yaw) @ Rx(pi)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # q = qz * qx  with qx=(1,0,0,0), qz=(0,0,s,c)
    # (w1 w2 - v1.v2, w1 v2 + w2 v1 + v1 x v2)
    w = c * 0 - (0 * 1 + 0 * 0 + s * 0)
    v = c * np.array([1, 0, 0]) + 0 * np.array([0, 0, s]) + np.cross([0, 0, s], [1, 0, 0])
    return (v[0], v[1], v[2], w)


class Rob:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no fjt server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no ik"
        assert self.fk.wait_for_service(10), "no fk"

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
        while "m" not in self.js:
            self.spin(0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def _seed(self):
        s = JointState()
        j = self.joints()
        for n in ARM:
            s.name.append(n); s.position.append(j[n])
        return s

    def fk_pose(self):
        """hand pose in base frame -> (pos, quat), plus tcp position."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        assert r is not None and r.error_code.val == 1, f"fk failed {r}"
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP * quat_R(*q)[:, 2]
        return pos, q, tcp

    def ik_tcp(self, x, y, z, q=Q_DOWN):
        """IK for a TCP position (base frame) with orientation q. Returns joints or None."""
        R = quat_R(*q)
        hx, hy, hz = np.array([x, y, z]) - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"  # group tip is link8 (45deg off)
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = float(hx), float(hy), float(hz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = self._seed()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK failed: {None if r is None else r.error_code.val}")
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[n] for n in ARM]

    def move_q(self, q, sec=3.0, retries=1):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        for attempt in range(retries + 1):
            t0 = time.time()
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"  fjt code={code} max_joint_err={err:.4f} ({time.time()-t0:.0f}s)")
            if code == 0 and err < 0.02:
                return True
        return err < 0.05

    def move_tcp(self, x, y, z, q=Q_DOWN, sec=3.0):
        sol = self.ik_tcp(x, y, z, q)
        if sol is None:
            return False
        ok = self.move_q(sol, sec)
        pos, _, tcp = self.fk_pose()
        print(f"  tcp now {tcp.round(4)} target {np.array([x,y,z]).round(4)}")
        return ok

    def gripper(self, w):
        g = GripperCommand.Goal()
        g.command.position = float(w)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=180)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper -> {w}: reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def shutdown(self):
        rclpy.shutdown()


if __name__ == "__main__":
    r = Rob()
    cmd = sys.argv[1]
    if cmd == "fk":
        pos, q, tcp = r.fk_pose()
        print("hand", pos.round(4), "q", np.round(q, 4), "tcp", tcp.round(4))
        print("joints", np.round(r.arm_q(), 4), "fingers", r.fingers())
    elif cmd == "grip":
        r.gripper(float(sys.argv[2]))
    elif cmd == "tcp":
        x, y, z = map(float, sys.argv[2:5])
        sec = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
        r.move_tcp(x, y, z, sec=sec)
    r.shutdown()
