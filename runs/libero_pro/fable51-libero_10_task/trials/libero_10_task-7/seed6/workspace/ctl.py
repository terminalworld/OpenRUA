#!/usr/bin/env python3
"""Pick/place controller for this Panda.

  ctl.py state                         joints, fingers, hand pose (FK), wrench
  ctl.py move X Y Z MODE [secs]        TCP (fingertip midpoint) to world XYZ,
                                       hand pointing down; MODE y = fingers
                                       close along world y, x = along world x
  ctl.py grip open|close
  ctl.py joints p1,...,p7 [secs]       raw joint target
All poses are WORLD frame (verified: /compute_ik & /compute_fk work in world).
"""
import sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034
FJT = "/panda_arm_controller/follow_joint_trajectory"
GRIP = "/franka_gripper/gripper_action"
# hand pointing down; hand x -> world x (fingers open along world y)
Q_Y = Rot.from_quat([1, 0, 0, 0])
# same, yawed 90 deg: fingers open along world x
Q_X = Rot.from_euler("z", 90, degrees=True) * Q_Y


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._wr, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT)
        self.grip = ActionClient(self.node, GripperCommand, GRIP)

    def _js(self, m):
        self.js = m

    def _wr(self, m):
        self.wr = m

    def spin(self, t):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self, fresh=True):
        if fresh:
            self.js = None
        end = time.time() + 15
        while self.js is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in ARM], (d["panda_finger_joint1"], d["panda_finger_joint2"])

    def hand_pose(self, q):
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        ps = fut.result().pose_stamped[0].pose
        p = np.array([ps.position.x, ps.position.y, ps.position.z])
        R = Rot.from_quat([ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w])
        return p, R

    def solve_ik(self, hand_p, R, seed):
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_p)
        qx, qy, qz, qw = R.as_quat()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            return None, "timeout"
        if r.error_code.val != 1:
            return None, r.error_code.val
        d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [d[j] for j in ARM], 1

    def run_traj(self, target, secs):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(v) for v in target])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        t0 = time.time()
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            print("FJT goal rejected"); return None
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=900)
        if res.result() is None:
            print("FJT result timeout (client side)")
            code = None
        else:
            code = res.result().result.error_code
        q, _ = self.joints()
        err = np.abs(np.array(q) - np.array(target))
        print(f"traj done in {time.time()-t0:.1f}s wall, error_code={code}, "
              f"max joint err={err.max():.4f} rad")
        return code

    def move_tcp(self, tcp, mode, secs=3.0):
        if mode.startswith("yaw:"):  # fingers close along world (90+deg) degrees
            R = Rot.from_euler("z", float(mode[4:]), degrees=True) * Q_Y
        else:
            R = Q_X if mode == "x" else Q_Y
        hand_p = np.array(tcp, float) - TCP * R.apply([0, 0, 1])
        seed, _ = self.joints()
        sol, code = self.solve_ik(hand_p, R, seed)
        if sol is None:
            print(f"IK FAILED ({code}) for tcp={tcp} mode={mode}; no motion")
            return False
        dq = np.abs(np.array(sol) - np.array(seed))
        print("IK ok; joint delta max %.3f; target %s" % (dq.max(), ", ".join(f"{v:.3f}" for v in sol)))
        rc = self.run_traj(sol, secs)
        q, _ = self.joints()
        p, Ract = self.hand_pose(q)
        tcp_act = p + TCP * Ract.apply([0, 0, 1])
        print(f"TCP now {tcp_act.round(4)} (wanted {np.array(tcp).round(4)}), "
              f"pos err {np.linalg.norm(tcp_act-np.array(tcp))*1000:.1f} mm")
        return rc == 0

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        send = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result if res.result() else None
        _, f = self.joints()
        print(f"gripper -> {width}: reached={getattr(r,'reached_goal',None)} "
              f"stalled={getattr(r,'stalled',None)} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def state(self):
        q, f = self.joints()
        p, R = self.hand_pose(q)
        tcp = p + TCP * R.apply([0, 0, 1])
        print("joints:", ", ".join(f"{v:.4f}" for v in q))
        print(f"fingers: {f[0]:.4f} {f[1]:.4f}")
        print(f"hand: {p.round(4)} quat {R.as_quat().round(4)}  TCP: {tcp.round(4)}")
        self.spin(0.5)
        if self.wr:
            w = self.wr.wrench
            print(f"wrench F=({w.force.x:.2f},{w.force.y:.2f},{w.force.z:.2f}) "
                  f"T=({w.torque.x:.2f},{w.torque.y:.2f},{w.torque.z:.2f})")


def main():
    if sys.argv[1:2] == ["iktest"]:
        return
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    c = Ctl()
    if a[0] == "state":
        c.state()
    elif a[0] == "move":
        x, y, z = map(float, a[1:4]); mode = a[4]
        secs = float(a[5]) if len(a) > 5 else 3.0
        ok = c.move_tcp((x, y, z), mode, secs)
        print("MOVE", "OK" if ok else "PROBLEM")
    elif a[0] == "grip":
        c.gripper(0.04 if a[1] == "open" else 0.0)
    elif a[0] == "joints":
        tgt = [float(v) for v in a[1].split(",")]
        secs = float(a[2]) if len(a) > 2 else 3.0
        c.run_traj(tgt, secs)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


def iktest():
    """ctl.py iktest X Y Z: try several orientations, report feasibility (no motion)."""
    c = Ctl()
    x, y, z = map(float, sys.argv[2:5])
    seed, _ = c.joints()
    cands = {"down_y": Q_Y, "down_x": Q_X}
    # tilt the approach away from the base (lean the hand outward) by 15/30 deg
    for deg in (15, 30, 45):
        ang = np.arctan2(y, x + 0.51)  # direction base->target in world
        axis = np.array([-np.sin(ang), np.cos(ang), 0])  # horizontal, perpendicular
        tilt = Rot.from_rotvec(-np.deg2rad(deg) * axis)
        cands[f"tilt{deg}_y"] = tilt * Q_Y
        cands[f"tilt{deg}_x"] = tilt * Q_X
    for name, R in cands.items():
        hand_p = np.array([x, y, z]) - TCP * R.apply([0, 0, 1])
        sol, code = c.solve_ik(hand_p, R, seed)
        print(f"{name:10s} hand_p={hand_p.round(3)} code={code} "
              + ("sol=" + ", ".join(f"{v:.3f}" for v in sol) if sol else ""))
    rclpy.shutdown()


if __name__ == "__main__" and sys.argv[1:2] == ["iktest"]:
    iktest()
