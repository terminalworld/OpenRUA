#!/usr/bin/env python3
"""Arm controller for this Panda: TCP-in-world IK moves, gripper, state.

Usage:
  python3 arm.py state                      # hand/TCP world pose + joints
  python3 arm.py ikcheck                    # IK on current pose: frame sanity check
  python3 arm.py move X Y Z [SECS] [--q qx qy qz qw]   # TCP world pose (default hand pointing down, fingers along world y)
  python3 arm.py grip open|close
"""
import sys, time
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
DOWN_Q = (1.0, 0.0, 0.0, 0.0)  # hand z down, fingers along world y


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_mul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self.buf = Buffer(); TransformListener(self.buf, self.node)
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.spin(1.0)
        while not self.buf.can_transform("world", "panda_link0", Time()) or "m" not in self.js:
            self.spin(0.2)
        t = self.buf.lookup_transform("world", "panda_link0", Time()).transform.translation
        self.base = np.array([t.x, t.y, t.z])

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def hand_world(self):
        # force a fresh TF sample after motion
        self.spin(0.3)
        end = time.time() + 15
        while not self.buf.can_transform("world", "panda_hand", Time()) and time.time() < end:
            self.spin(0.2)
        t = self.buf.lookup_transform("world", "panda_hand", Time())
        p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        q = t.transform.rotation
        R = quat_R(q.x, q.y, q.z, q.w)
        return p, (q.x, q.y, q.z, q.w), p + TCP * R[:, 2]

    def state(self):
        p, q, tcp = self.hand_world()
        j = self.joints()
        print(f"hand world=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f}) q=({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})")
        print(f"TCP  world=({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f})")
        print("arm:", [round(j[n], 4) for n in JOINTS])
        print("fingers:", round(j.get("panda_finger_joint1", 0), 4), round(j.get("panda_finger_joint2", 0), 4))
        return p, q, tcp, j

    def solve_ik(self, hand_world_p, q, seed_pos=None):
        if not self.ik.wait_for_service(timeout_sec=10):
            raise SystemExit("IK unavailable")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        # machine fact (verified): IK tip link is panda_link8, poses are in
        # WORLD coordinates. link8 = hand * Rz(+45deg) (same origin).
        pb = np.asarray(hand_world_p)
        q8 = quat_mul(q, (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)))
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, pb)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q8)
        seed = JointState()
        if seed_pos is None:
            j = self.joints(); seed_pos = [j[n] for n in JOINTS]
        for n, p_ in zip(JOINTS, seed_pos):
            seed.name.append(n); seed.position.append(float(p_))
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise SystemExit("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in JOINTS]

    def traj_multi(self, waypoints, total_secs):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        n = len(waypoints)
        for i, wp in enumerate(waypoints):
            t = total_secs * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(p) for p in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        j = self.joints()
        err = max(abs(j[n] - p) for n, p in zip(JOINTS, waypoints[-1]))
        print(f"traj_multi done error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp_lin(self, xyz, q=DOWN_Q, step=0.02, speed=0.05, max_jump=0.35):
        """Straight-line TCP move in small steps; IK chained from the previous
        waypoint so the arm stays on one configuration branch."""
        _, _, tcp0 = self.hand_world()
        target = np.asarray(xyz, float)
        dist = np.linalg.norm(target - tcp0)
        n = max(2, int(np.ceil(dist / step)))
        R = quat_R(*q)
        j = self.joints(); seed = [j[k] for k in JOINTS]
        wps = []
        for i in range(1, n + 1):
            p = tcp0 + (target - tcp0) * i / n
            hand = p - TCP * R[:, 2]
            sol = self.solve_ik(hand, q, seed)
            if sol is None:
                print(f"IK FAILED at waypoint {i}/{n} {p}"); return False
            jump = max(abs(a - b) for a, b in zip(sol, seed))
            if jump > max_jump:
                print(f"branch jump {jump:.2f} at waypoint {i}/{n}; aborting"); return False
            wps.append(sol); seed = sol
        secs = max(1.0, dist / speed)
        code, err = self.traj_multi(wps, secs)
        if err > 0.02:
            print("large tracking error, resending last point")
            code, err = self.traj(wps[-1], 1.5)
        _, _, tcp = self.hand_world()
        d = np.linalg.norm(tcp - target)
        print(f"TCP now ({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}) target {tuple(round(v,4) for v in xyz)} dist={d:.4f}")
        return d < 0.01

    def traj(self, positions, secs):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        j = self.joints()
        err = max(abs(j[n] - p) for n, p in zip(JOINTS, positions))
        print(f"traj done error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, xyz, secs=3.0, q=DOWN_Q):
        R = quat_R(*q)
        hand = np.asarray(xyz, float) - TCP * R[:, 2]
        sol = self.solve_ik(hand, q)
        if sol is None:
            print(f"IK FAILED for TCP {xyz}")
            return False
        lim = FJT["limits_rad"]
        for n, p, (lo, hi) in zip(JOINTS, sol, lim):
            if not lo <= p <= hi:
                print(f"IK solution violates limit {n}={p:.3f} not in [{lo},{hi}]")
                return False
        code, err = self.traj(sol, secs)
        if err > 0.02:
            print("large tracking error, resending")
            code, err = self.traj(sol, secs)
        p, _, tcp = self.hand_world()
        d = np.linalg.norm(tcp - np.asarray(xyz))
        print(f"TCP now ({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}) target {tuple(round(v,4) for v in xyz)} dist={d:.4f}")
        return d < 0.01

    def gripper(self, open_):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise SystemExit("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        j = self.joints()
        f1, f2 = j["panda_finger_joint1"], j["panda_finger_joint2"]
        print(f"gripper {'open' if open_ else 'close'}: reached={r.reached_goal} stalled={r.stalled} fingers={f1:.4f},{f2:.4f} gap={abs(f1)+abs(f2):.4f}")
        return f1, f2


if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "state":
        a.state()
    elif cmd == "ikcheck":
        p, q, tcp, j = a.state()
        sol = a.solve_ik(p, q)
        print("IK of current pose:", None if sol is None else [round(v, 4) for v in sol])
    elif cmd == "move":
        args = [x for x in sys.argv[2:] if not x.startswith("--")]
        xyz = list(map(float, args[:3])); secs = float(args[3]) if len(args) > 3 else 3.0
        q = DOWN_Q
        if "--q" in sys.argv:
            i = sys.argv.index("--q"); q = tuple(map(float, sys.argv[i+1:i+5]))
        ok = a.move_tcp(xyz, secs, q)
        print("OK" if ok else "NOT AT TARGET")
    elif cmd == "grip":
        a.gripper(sys.argv[2] == "open")
    rclpy.shutdown()


def plan_only(a, xyz, q=DOWN_Q):
    R = quat_R(*q)
    hand = np.asarray(xyz, float) - TCP * R[:, 2]
    sol = a.solve_ik(hand, q)
    j = a.joints()
    cur = [j[n] for n in JOINTS]
    if sol is None:
        print("IK FAILED", xyz); return None
    print("sol ", [round(v, 3) for v in sol])
    print("cur ", [round(v, 3) for v in cur])
    print("dq  ", [round(s - c, 3) for s, c in zip(sol, cur)])
    return sol
