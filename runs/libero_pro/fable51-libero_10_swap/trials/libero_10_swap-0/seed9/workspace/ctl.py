#!/usr/bin/env python3
"""Small controller: IK -> trajectory, gripper, FK readout.

Usage:
  ctl.py fk                         print hand + TCP pose (world frame), finger gap
  ctl.py goto X Y Z YAW [SECONDS]   move TCP (fingertip point) to world x,y,z,
                                    hand pointing down, fingers sliding along
                                    world axis rotated YAW deg from +Y
  ctl.py joints p1,...,p7 [SECONDS] raw joint move
  ctl.py open | close               gripper
"""
import sys, math
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]
# MoveIt FK/IK poses are in the WORLD frame (verified against the TF chain:
# link0 sits at world (-0.51, 0, 0.42) and the planner includes that offset).
BASE = np.array([0.0, 0.0, 0.0])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    w = math.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.sqrt(max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = math.sqrt(max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = math.sqrt(max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = math.copysign(x, R[2, 1] - R[1, 2])
    y = math.copysign(y, R[0, 2] - R[2, 0])
    z = math.copysign(z, R[1, 0] - R[0, 1])
    return x, y, z, w


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])

    def call(self, client, req, tries=6, timeout=20.0):
        client.wait_for_service(10)
        for i in range(tries):
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            if fut.done() and fut.result() is not None:
                return fut.result()
            print(f"  service call attempt {i+1} timed out; retrying")
        return None

    def joint_state(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_seed(self):
        js = self.joint_state()
        s = JointState()
        for j in JOINTS:
            s.name.append(j); s.position.append(js[j])
        return s, js

    def fk_pose(self):
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        seed, js = self.arm_seed()
        req.robot_state.joint_state = seed
        res = self.call(self.fk, req)
        p = res.pose_stamped[0].pose
        R = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        hand = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        tcp = hand + TCP * R[:, 2]
        return hand, tcp, R, js

    def print_fk(self):
        hand, tcp, R, js = self.fk_pose()
        gap = js.get("panda_finger_joint1", float("nan"))
        print(f"hand(world)={hand.round(4).tolist()} tcp(world)={tcp.round(4).tolist()} "
              f"hand_z_axis={R[:,2].round(3).tolist()} hand_y_axis={R[:,1].round(3).tolist()} finger1={gap:.4f}")
        print("joints:", [round(js[j], 4) for j in JOINTS])
        return hand, tcp

    def solve_ik(self, tcp_world, yaw_deg):
        yaw = math.radians(yaw_deg)
        # hand z down; hand y (finger slide axis) along world +Y rotated by yaw about Z
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        R = Rz @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)
        hand_world = np.array(tcp_world, float) - TCP * R[:, 2]
        hand_base = hand_world - BASE
        qx, qy, qz, qw = R_to_quat(R)
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"  # default tip is link8 (45 deg off)
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = hand_base.tolist()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        seed, js = self.arm_seed()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        cur = np.array([js[j] for j in JOINTS])
        lim = np.array(FJT["limits_rad"])
        rng = np.random.default_rng(0)
        seeds = [cur.tolist()]
        for _ in range(5):  # small perturbations of the current config
            seeds.append(np.clip(cur + rng.normal(0, 0.15, 7), lim[:, 0], lim[:, 1]).tolist())
        seeds += [[0, -0.785, 0, -2.356, 0, 1.571, 0.785], [0, -0.3, 0, -2.2, 0, 2.0, 0.785]]
        for _ in range(4):
            seeds.append(rng.uniform(lim[:, 0], lim[:, 1]).tolist())
        best = None
        for sd in seeds:
            seed.position = [float(v) for v in sd]
            req.ik_request.robot_state.joint_state = seed
            res = self.call(self.ik, req)
            if res is None or res.error_code.val != 1:
                continue
            sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            q = np.array([sol[j] for j in JOINTS])
            dist = np.abs(q - cur).sum()
            if best is None or dist < best[0]:
                best = (dist, q)
            if dist < 0.8:
                break
        if best is None:
            print(f"IK FAILED target hand(world)={hand_base.round(3).tolist()}")
            return None, js
        dist, q = best
        print(f"IK ok: hand(world)={hand_base.round(3).tolist()} q={[round(v,3) for v in q]} L1 move={dist:.2f} rad")
        return q.tolist(), js
    def move_joints(self, q, seconds):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        result = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result)
        code = result.result().result.error_code
        js = self.joint_state()
        err = max(abs(js[j] - v) for j, v in zip(JOINTS, q))
        print(f"trajectory done error_code={code} max joint err={err:.4f} rad")
        return code, err

    def goto(self, tcp_world, yaw_deg, seconds):
        q, js = self.solve_ik(tcp_world, yaw_deg)
        if q is None:
            return False
        self.move_joints(q, seconds)
        hand, tcp, R, _ = self.fk_pose()
        e = np.linalg.norm(tcp - np.array(tcp_world))
        print(f"reached tcp(world)={tcp.round(4).tolist()} target={np.round(tcp_world,4).tolist()} err={e*1000:.1f} mm")
        return e < 0.01

    def gripper(self, width):
        self.grip.wait_for_server(10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res_fut = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res_fut, timeout_sec=300)
        res = res_fut.result().result
        js = self.joint_state()
        print(f"gripper cmd={width} reached_goal={res.reached_goal} stalled={res.stalled} "
              f"finger1={js['panda_finger_joint1']:.4f} finger2={js['panda_finger_joint2']:.4f}")
        return js["panda_finger_joint1"]


def main():
    a = sys.argv[1:]
    c = Ctl()
    cmd = a[0]
    if cmd == "fk":
        c.print_fk()
    elif cmd == "goto":
        x, y, z, yaw = map(float, a[1:5])
        sec = float(a[5]) if len(a) > 5 else 4.0
        c.goto([x, y, z], yaw, sec)
    elif cmd == "joints":
        q = [float(v) for v in a[1].split(",")]
        sec = float(a[2]) if len(a) > 2 else 4.0
        c.move_joints(q, sec)
        c.print_fk()
    elif cmd == "open":
        c.gripper(GRIP["open_m"])
    elif cmd == "close":
        c.gripper(GRIP["closed_m"])
    else:
        raise SystemExit(__doc__)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
