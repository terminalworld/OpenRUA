#!/usr/bin/env python3
"""Arm control helpers: FK, IK, straight-line cartesian trajectories, gripper.
CLI:
  arm.py fk                       -> current TCP pose (world)
  arm.py open | close             -> gripper
  arm.py goto x y z [yaw_deg] [T] -> TCP to world pose, hand down, via IK+FJT (single point)
  arm.py line x y z [yaw_deg] [T] [pitch_deg] -> straight-line TCP move (multi-waypoint IK)
  arm.py joints p1,..,p7 T
"""
import sys, time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from scipy.spatial.transform import Rotation as Rot, Slerp

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
# verified: /compute_fk and /compute_ik on this machine both use WORLD coordinates
# (panda_link0 sits at world (-0.51, 0, 0.42)); no base offset needed.
BASE_T = np.eye(4)

class Arm:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        while self.js is None: rclpy.spin_once(self.node, timeout_sec=0.2)

    def _js(self, m): self.js = m

    def spin(self, n=3):
        for _ in range(n): rclpy.spin_once(self.node, timeout_sec=0.1)

    def q(self):
        self.spin()
        d = dict(zip(self.js.name, self.js.position))
        return np.array([d[j] for j in JOINTS])

    def fingers(self):
        self.spin()
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk_pose(self, q=None):
        """hand pose in WORLD: (pos, quat xyzw)"""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(self.q() if q is None else q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = BASE_T[:3, :3] @ np.array([p.position.x, p.position.y, p.position.z]) + BASE_T[:3, 3]
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat, r.error_code.val

    def tcp_pose(self, q=None):
        pos, quat, _ = self.fk_pose(q)
        R = Rot.from_quat(quat).as_matrix()
        return pos + TCP_OFF * R[:, 2], quat

    def solve_ik(self, tcp_pos_world, quat, seed=None, attempts=3):
        """IK for a TCP pose (world). Returns joint array or None."""
        R = Rot.from_quat(quat).as_matrix()
        hand_w = np.asarray(tcp_pos_world) - TCP_OFF * R[:, 2]
        hand_b = hand_w - BASE_T[:3, 3]           # base has identity rotation
        seed = self.q() if seed is None else seed
        for k in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            req.ik_request.ik_link_name = "panda_hand"
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, hand_b)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = JOINTS
            s = np.array(seed) + (np.random.randn(7) * 0.1 if k else 0)
            req.ik_request.robot_state.joint_state.position = list(map(float, s))
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                sol = np.array([d[j] for j in JOINTS])
                # wrap-safety: pick the branch nearest the seed
                if np.abs(sol - seed).max() < 1.5 or k == attempts - 1:
                    return sol
                seed_alt = sol
            else:
                print(f"  IK attempt {k} failed code={None if r is None else r.error_code.val}")
        return None

    def send_traj(self, points, times, wait=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for p, t in zip(points, times):
            pt = JointTrajectoryPoint(positions=list(map(float, p)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("  goal rejected"); return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        res = rf.result()
        code = res.result.error_code if res else None
        err = np.abs(self.q() - np.array(points[-1])).max()
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code

    def move_joints(self, qt, T=3.0):
        return self.send_traj([qt], [T])

    def goto(self, tcp, quat, T=3.0):
        sol = self.solve_ik(tcp, quat)
        if sol is None: print("  IK FAILED, no motion"); return None
        return self.move_joints(sol, T)

    def line(self, tcp_to, quat, T=None, step=0.02):
        """straight-line TCP move from current pose (slerp orientation); multi-waypoint IK."""
        p0, q0 = self.tcp_pose()
        p1 = np.asarray(tcp_to, float)
        ang = (Rot.from_quat(q0).inv() * Rot.from_quat(quat)).magnitude()
        n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / step)) + 1, int(np.ceil(ang / 0.1)) + 1)
        slerp = Slerp([0, 1], Rot.from_quat([q0, quat]))
        seed = self.q(); pts = []
        for i in range(1, n):
            f = i / (n - 1)
            p = p0 + (p1 - p0) * f
            sol = self.solve_ik(p, slerp(f).as_quat(), seed=seed)
            if sol is None: print(f"  IK FAILED at waypoint {i}/{n-1} {p}"); return None
            pts.append(sol); seed = sol
        T = T or max(1.0, np.linalg.norm(p1 - p0) / 0.05)
        times = [T * (i + 1) / len(pts) for i in range(len(pts))]
        return self.send_traj(pts, times)

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        # settle ticks
        self.spin(10)
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}")
        return r

def down_quat(yaw_deg=0.0, pitch_deg=0.0):
    """hand pointing down (hand z = -world z); yaw about world z; pitch>0 tilts the
    approach toward -x (hand body sits on the +x side). yaw=0: fingers open along world y."""
    return (Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_euler("y", pitch_deg, degrees=True)
            * Rot.from_euler("x", 180, degrees=True)).as_quat()

if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "fk":
        p, qt = a.tcp_pose(); print("TCP", p.round(4), "quat", qt.round(4), "joints", a.q().round(3), "fingers", a.fingers())
    elif cmd == "open": a.gripper(GRIP["open_m"])
    elif cmd == "close": a.gripper(GRIP["closed_m"])
    elif cmd in ("goto", "line"):
        x, y, z = map(float, sys.argv[2:5])
        yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
        T = float(sys.argv[6]) if len(sys.argv) > 6 else None
        pitch = float(sys.argv[7]) if len(sys.argv) > 7 else 0.0
        (a.goto if cmd == "goto" else a.line)([x, y, z], down_quat(yaw, pitch), T or (3.0 if cmd == "goto" else None))
        p, qt = a.tcp_pose(); print("TCP now", p.round(4), "quat", qt.round(4))
    elif cmd == "joints":
        a.move_joints([float(v) for v in sys.argv[2].split(",")], float(sys.argv[3]))
    rclpy.shutdown()
