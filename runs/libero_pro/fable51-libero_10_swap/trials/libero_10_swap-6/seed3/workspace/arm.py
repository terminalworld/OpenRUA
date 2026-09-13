#!/usr/bin/env python3
"""Arm helper: IK (with frame test), trajectory execution, gripper, state.

  python3 arm.py iktest
  python3 arm.py goto <x> <y> <z> [seconds] [--hand]   # world TCP pose, hand down, fingers along world y
  python3 arm.py grip <per_finger_m>
  python3 arm.py state
"""
import sys, time
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # from TF world->panda_link0
TCP = M["hand"]["tcp_offset_m"]
# hand pointing straight down, fingers (hand Y) along world Y: 180 deg about X
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grp = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")

    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
        end = time.time() + 15
        while "m" not in self.js and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_joints(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def solve_ik(self, pos, quat, seed=None, frame_pos_is_world=True):
        """pos: TCP position (world if frame_pos_is_world). Returns joint list or None."""
        p = np.array(pos, float)
        R = quat_R(*quat)
        hand = p - TCP * R[:, 2]          # hand frame = TCP back along hand +Z
        if not frame_pos_is_world:
            hand = hand + BASE_IN_WORLD   # IK (like FK) works in world on this machine
        # IK solves for panda_link8 = hand rotated +45deg about Z (TF link8->hand is -45deg)
        q8 = quat_mul(quat, (0.0, 0.0, 0.3826834, 0.9238795))
        self.ik.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, hand)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q8)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        s = JointState(); s.name = list(ARM)
        s.position = list(seed) if seed is not None else self.arm_joints()
        req.ik_request.robot_state.joint_state = s
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print("IK failed", None if res is None else res.error_code.val)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in ARM]

    def fk_hand(self, q=None):
        self.fk.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = list(q) if q is not None else self.arm_joints()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        ps = fut.result().pose_stamped[0].pose
        return (np.array([ps.position.x, ps.position.y, ps.position.z]),
                (ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w))

    def tcp_world(self):
        p, q = self.fk_hand()
        return p + TCP * quat_R(*q)[:, 2], q

    def move_joints(self, q, seconds=3.0, waypoints=None):
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = list(waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(x) for x in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        cur = self.arm_joints()
        err = max(abs(a - b) for a, b in zip(cur, q))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, pos, seconds=3.0, quat=Q_DOWN, seed=None):
        q = self.solve_ik(pos, quat, seed=seed)
        if q is None:
            return None
        code, err = self.move_joints(q, seconds)
        p, _ = self.tcp_world()
        print(f"TCP now world=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f}) target={tuple(round(x,4) for x in pos)}")
        return q

    def grip(self, width):
        self.grp.wait_for_server(timeout_sec=10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grp.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"grip reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f


def quat_mul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "iktest":
        cur = a.arm_joints()
        p, q = a.fk_hand()
        print("FK hand (as returned):", p, q)
        tcp = p + TCP * quat_R(*q)[:, 2]
        for label, world in (("as-world", True), ("as-base", False)):
            sol = a.solve_ik(tcp, q, frame_pos_is_world=world)
            if sol:
                print(label, "max dev from current:", max(abs(x - y) for x, y in zip(sol, cur)))
    elif cmd == "goto":
        pos = [float(x) for x in sys.argv[2:5]]
        sec = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
        a.goto(pos, sec)
    elif cmd == "line":
        # straight TCP line from current TCP to target via n IK waypoints
        pos = np.array([float(x) for x in sys.argv[2:5]])
        n = int(sys.argv[5]) if len(sys.argv) > 5 else 4
        sec = float(sys.argv[6]) if len(sys.argv) > 6 else 3.0
        start, _ = a.tcp_world()
        seed = a.arm_joints(); wps = []
        for i in range(1, n + 1):
            p = start + (pos - start) * i / n
            q = a.solve_ik(p, Q_DOWN, seed=seed)
            if q is None:
                raise SystemExit(f"no IK at waypoint {p}")
            seed = q
            wps.append((q, sec * i / n))
        a.move_joints(wps[-1][0], sec, waypoints=wps[:-1])
        p, _ = a.tcp_world()
        print(f"TCP now world=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f}) target={pos}")
    elif cmd == "grip":
        a.grip(float(sys.argv[2]))
    elif cmd == "state":
        p, q = a.tcp_world()
        print("TCP world:", p, "hand quat:", q)
        print("fingers:", a.fingers())
        print("arm:", [round(x, 4) for x in a.arm_joints()])
    rclpy.shutdown()
