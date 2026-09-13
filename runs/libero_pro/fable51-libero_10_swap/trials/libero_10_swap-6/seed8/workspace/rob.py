#!/usr/bin/env python3
"""Robot helper: FK / IK / trajectory / gripper against machine.yaml ports.

Positions given to the CLI are in the WORLD frame; converted to the arm
base (planning frame) internally using the world->panda_link0 offset.

  python3 rob.py fk                         # hand + tcp pose (world)
  python3 rob.py js                         # joint dict
  python3 rob.py goto X Y Z [yaw_deg] [secs] # TCP to world pose, top-down
  python3 rob.py joints p1,...,p7 [secs]
  python3 rob.py grip open|close
"""
import sys
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = M["hand"]["tcp_offset_m"]
# Verified: FK(panda_link0) with empty frame_id returns (-0.51, 0, 0.42),
# i.e. the planner's model frame IS world on this machine -> no offset.
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def topdown_R(yaw_deg):
    """Hand Z pointing down, hand X rotated by yaw about world Z."""
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # flip: Z down
    return Rz @ Rx


class Rob:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.update(zip(m.name, m.position)), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js.clear()
        end = time.time() + 10
        while not all(j in self.js for j in JOINTS) and time.time() < end:
            self.spin()
        return [self.js[j] for j in JOINTS]

    def fingers(self):
        self.joints()
        return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")

    def _seed(self, q=None):
        q = q if q is not None else self.joints()
        s = JointState()
        s.name = list(JOINTS)
        s.position = [float(v) for v in q]
        return s

    def fk(self, q=None, link="panda_hand"):
        self.fk_cli.wait_for_service(10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        R = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, R

    def fk_world(self, q=None):
        pos, R = self.fk(q)
        tcp = pos + TCP * R[:, 2]
        return pos + BASE_IN_WORLD, tcp + BASE_IN_WORLD, R

    def ik(self, tcp_world, R, seed=None, tries=1):
        """IK for TCP at world position with hand rotation R. Returns joints or None."""
        pos = np.asarray(tcp_world) - BASE_IN_WORLD - TCP * R[:, 2]
        q = R_to_quat(R)
        self.ik_cli.wait_for_service(10)
        seed_q = seed if seed is not None else self.joints()
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            # the group's default tip is panda_link8, which is yawed 45 deg
            # from panda_hand; solve for the hand frame explicitly
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state = self._seed(seed_q)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 1
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                out = [sol[j] for j in JOINTS]
                if all(lo <= v <= hi for v, (lo, hi) in zip(out, LIMITS)):
                    return out
            seed_q = [np.clip(v + np.random.uniform(-0.3, 0.3), lo, hi)
                      for v, (lo, hi) in zip(seed_q, LIMITS)]
        return None

    def move(self, q, secs=3.0, waypoints=None, retries=2):
        """Execute trajectory to q (optionally via intermediate waypoints).
        Controller lag can leave a joint short (error_code -5); resend."""
        code, err = self._move(q, secs, waypoints)
        while err > 0.02 and retries > 0:
            print("  retrying (controller lag)")
            code, err = self._move(q, max(2.0, secs / 2), None)
            retries -= 1
        return code, err

    def _move(self, q, secs=3.0, waypoints=None):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = list(waypoints or []) + [q]
        n = len(pts)
        for i, p in enumerate(pts):
            t = secs * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in p])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        now = self.joints()
        err = float(np.max(np.abs(np.array(now) - np.array(q))))
        print(f"move: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, tcp_world, yaw_deg=0.0, secs=3.0, seed=None):
        R = topdown_R(yaw_deg)
        q = self.ik(tcp_world, R, seed=seed, tries=8)
        if q is None:
            print(f"IK FAILED for {tcp_world} yaw={yaw_deg}")
            return None
        self.move(q, secs)
        _, tcp, _ = self.fk_world()
        print(f"goto: tcp now {np.round(tcp, 4)} (target {np.round(tcp_world, 4)})")
        return q

    def grip(self, open_):
        self.gr.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"grip {'open' if open_ else 'close'}: reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f


def main():
    a = sys.argv[1:]
    r = Rob()
    if a[0] == "fk":
        hand, tcp, R = r.fk_world()
        print("hand world", np.round(hand, 4))
        print("tcp  world", np.round(tcp, 4))
        print("R\n", np.round(R, 3))
    elif a[0] == "js":
        print(dict(zip(JOINTS, np.round(r.joints(), 4))), r.fingers())
    elif a[0] == "goto":
        x, y, z = map(float, a[1:4])
        yaw = float(a[4]) if len(a) > 4 else 0.0
        secs = float(a[5]) if len(a) > 5 else 3.0
        r.goto(np.array([x, y, z]), yaw, secs)
    elif a[0] == "joints":
        q = [float(v) for v in a[1].split(",")]
        r.move(q, float(a[2]) if len(a) > 2 else 3.0)
    elif a[0] == "grip":
        r.grip(a[1] == "open")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
