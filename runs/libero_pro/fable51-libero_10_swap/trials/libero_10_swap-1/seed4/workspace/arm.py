#!/usr/bin/env python3
"""Helper: IK moves in WORLD coords (TCP), gripper, joint read. Clients built once."""
import sys, math, time
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
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK/IK model frame IS world here (FK of panda_link0 = (-0.51,0,0.42))

def quat_down(yaw=0.0):
    """hand pointing -Z world, fingers along world y (yaw=0); yaw rotates about world z."""
    # q = qz(yaw) * (1,0,0,0)
    c, s = math.cos(yaw/2), math.sin(yaw/2)
    # (w,x,y,z) product of (c,0,0,s) * (0,1,0,0) = (0, c, s, 0)
    return (c, s, 0.0, 0.0)  # x,y,z,w

def R_of(qx, qy, qz, qw):
    x, y, z, w = qx, qy, qz, qw
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],[2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

class Arm:
    def __init__(self):
        rclpy.init(); self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", lambda m: self.js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js: rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(zip(self.js["m"].name, self.js["m"].position))

    def arm_state(self):
        j = self.joints(); s = JointState(); s.name = list(JOINTS); s.position = [j[n] for n in JOINTS]; return s

    def tcp_world(self):
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_state()
        fut = self.fk.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        R = R_of(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        hand = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return hand + TCP * R[:, 2], (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def solve_ik(self, xyz_world, q, at_tcp=True):
        R = R_of(*q); xyz = np.array(xyz_world) - BASE_IN_WORLD
        if at_tcp: xyz = xyz - TCP * R[:, 2]
        req = GetPositionIK.Request(); req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"  # default tip is panda_link8 (45deg yaw off)
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = self.arm_state()
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"IK failed: {None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def traj(self, positions, seconds):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        j = self.joints(); err = max(abs(j[n] - p) for n, p in zip(JOINTS, positions))
        print(f"  traj code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move(self, xyz_world, yaw=0.0, seconds=3.0, retries=2):
        q = quat_down(yaw)
        for k in range(retries + 1):
            pos = self.solve_ik(xyz_world, q)
            code, err = self.traj(pos, seconds)
            if code == 0 and err < 0.01: break
        tcp, _ = self.tcp_world()
        print(f"  TCP now world=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f}) target={tuple(round(v,3) for v in xyz_world)}", flush=True)
        return tcp

    def gripper(self, width):
        goal = GripperCommand.Goal(); goal.command.position = float(width); goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=180)
        r = res.result().result; j = self.joints()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers=({j['panda_finger_joint1']:.4f},{j['panda_finger_joint2']:.4f})", flush=True)
        return j['panda_finger_joint1']
