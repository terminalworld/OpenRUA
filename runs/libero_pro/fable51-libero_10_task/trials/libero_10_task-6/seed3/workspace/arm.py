#!/usr/bin/env python3
"""Reusable arm helper: joint state, FK, IK, trajectory, gripper. Import or run.

python3 arm.py ik x y z qx qy qz qw          -> print IK solution (no motion)
python3 arm.py goto x y z qx qy qz qw [sec]  -> IK + trajectory for TCP pose (world)
python3 arm.py joints p1,...,p7 [sec]        -> trajectory
python3 arm.py grip open|close
python3 arm.py state                         -> joints + hand/tcp world pose
"""
import sys, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli.wait_for_service(timeout_sec=20)
        self.ik_cli.wait_for_service(timeout_sec=20)
        self.traj.wait_for_server(timeout_sec=20)
        self.grip.wait_for_server(timeout_sec=20)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        d = dict(zip(m.name, m.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def _seed(self, q):
        s = JointState(); s.name = list(ARM); s.position = [float(v) for v in q]
        return s

    def fk(self, q=None):
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        p = fut.result().pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        R = quat_to_R(*quat)
        return pos, quat, pos + TCP * R[:, 2]

    def ik(self, pos, quat, at_tcp=True, seed=None):
        pos = np.array(pos, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(*quat)[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed if seed is not None else self.arm_q())
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 5
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"IK failed code={code}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        for i, wp in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def ik_near(self, pos, quat, at_tcp=True, max_jump=0.8, tries=8):
        """IK solution closest (in joint space) to the current configuration.
        Retries the (randomised) solver; raises if every solution jumps branches."""
        cur = np.array(self.arm_q())
        best, best_d = None, np.inf
        for _ in range(tries):
            try:
                q = np.array(self.ik(pos, quat, at_tcp=at_tcp, seed=cur))
            except RuntimeError:
                continue
            d = np.abs(q - cur).max()
            if d < best_d:
                best, best_d = q, d
            if d < max_jump:
                break
        if best is None:
            raise RuntimeError("IK failed")
        if best_d >= max_jump:
            raise RuntimeError(f"IK only found far solutions (max joint jump {best_d:.2f} rad)")
        return best.tolist(), best_d

    def goto(self, pos, quat, seconds=3.0, at_tcp=True, max_jump=0.8, tol=0.01):
        q, jump = self.ik_near(pos, quat, at_tcp=at_tcp, max_jump=max_jump)
        seconds = max(seconds, jump / 0.4)  # never faster than ~0.4 rad/s on the biggest joint
        for attempt in range(3):
            code, err = self.move_joints(q, seconds)
            if err < tol:
                break
            print(f"  not converged (err {err:.3f}); re-sending")
            seconds = max(2.0, seconds * 0.7)
        _, _, tcp = self.fk()
        print("tcp now", tcp.round(4), "target", np.array(pos).round(4), f"jump={jump:.2f}")
        if err >= tol:
            raise RuntimeError(f"move did not converge (joint err {err:.3f})")
        return tcp

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f


# IK poses are for panda_link8 (= panda_hand rotated +45deg about z), in WORLD frame.
# Both point z straight down; they differ in which world axis the fingers close along.
DOWN_Y = np.array([0.92388, -0.38268, 0.0, 0.0])  # fingers close along world y
DOWN_X = np.array([0.38268, -0.92388, 0.0, 0.0])  # fingers close along world x


def qmul(a, b):
    """Quaternion product a*b (xyzw): apply b first, then a (both extrinsic/world)."""
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def quat_axis(axis, deg):
    """Quaternion (xyzw) for a rotation of deg about a world axis."""
    ax = np.array(axis, float); ax = ax / np.linalg.norm(ax)
    h = np.radians(deg) / 2
    return np.array([*(ax * np.sin(h)), np.cos(h)])


def down_closing_along(yaw_deg):
    """Hand pointing straight down, fingers closing along the world direction yaw_deg (from +x)."""
    return qmul(quat_axis([0, 0, 1], yaw_deg), DOWN_X)


def main():
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "state":
        q = a.arm_q(); pos, quat, tcp = a.fk(q)
        print("q", np.round(q, 4).tolist()); print("hand", pos.round(4), quat.round(4)); print("tcp", tcp.round(4)); print("fingers", a.fingers())
    elif cmd == "ik":
        v = list(map(float, sys.argv[2:9]))
        print(np.round(a.ik(v[:3], v[3:]), 4).tolist())
    elif cmd == "goto":
        v = list(map(float, sys.argv[2:9])); sec = float(sys.argv[9]) if len(sys.argv) > 9 else 3.0
        a.goto(v[:3], v[3:], sec)
    elif cmd == "joints":
        q = list(map(float, sys.argv[2].split(","))); sec = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
        a.move_joints(q, sec)
    elif cmd == "grip":
        a.gripper(GRIP["open_m"] if sys.argv[2] == "open" else GRIP["closed_m"])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
