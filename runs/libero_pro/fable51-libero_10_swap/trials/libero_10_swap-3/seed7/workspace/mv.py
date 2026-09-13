#!/usr/bin/env python3
"""Move the TCP (fingertip midpoint) to a WORLD-frame pose via IK + trajectory.

Usage: python3 mv.py <x> <y> <z> <qx> <qy> <qz> <qw> [seconds]
       python3 mv.py fk          # just print the current TCP pose in world
World -> base: base is at world (-0.66, 0, 0.912) (from TF).
"""
import sys
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE = np.array([0.0, 0.0, 0.0])  # FK/IK model frame == world on this machine
TCP = 0.1034
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def joint_state(node):
    js = {}
    sub = node.create_subscription(JointState, "/joint_states",
                                   lambda m: js.setdefault("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return dict(zip(js["m"].name, js["m"].position))


def arm_seed(jsd):
    s = JointState()
    for j in JOINTS:
        s.name.append(j); s.position.append(jsd[j])
    return s


def fk_world(node, jsd):
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(timeout_sec=10)
    req = GetPositionFK.Request()
    req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state = arm_seed(jsd)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    res = fut.result()
    p = res.pose_stamped[0].pose
    q = p.orientation
    R = quat_R(q.x, q.y, q.z, q.w)
    hand = np.array([p.position.x, p.position.y, p.position.z]) + BASE
    tcp = hand + TCP * R[:, 2]
    return hand, tcp, (q.x, q.y, q.z, q.w)


def main():
    rclpy.init()
    node = rclpy.create_node("mv")
    jsd = joint_state(node)
    if sys.argv[1] == "fk":
        hand, tcp, q = fk_world(node, jsd)
        print("hand", hand.round(4), "tcp", tcp.round(4), "q", np.round(q, 4))
        print("fingers", round(jsd["panda_finger_joint1"], 4), round(jsd["panda_finger_joint2"], 4))
        return
    x, y, z, qx, qy, qz, qw = map(float, sys.argv[1:8])
    secs = float(sys.argv[8]) if len(sys.argv) > 8 else 3.0
    R = quat_R(qx, qy, qz, qw)
    hand_w = np.array([x, y, z]) - TCP * R[:, 2]
    hand_b = hand_w - BASE
    cli = node.create_client(GetPositionIK, "/compute_ik")
    cli.wait_for_service(timeout_sec=10)
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.ik_link_name = "panda_hand"  # default tip is link8 (45deg off)
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = hand_b
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    req.ik_request.robot_state.joint_state = arm_seed(jsd)
    req.ik_request.avoid_collisions = False
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    res = fut.result()
    if res is None or res.error_code.val != 1:
        raise SystemExit(f"IK FAILED code={None if res is None else res.error_code.val}; no motion")
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    target = [sol[j] for j in JOINTS]
    print("target joints", np.round(target, 3))
    # verify the IK solution actually reaches the requested pose
    hand_s, tcp_s, q_s = fk_world(node, sol)
    perr = np.linalg.norm(tcp_s - np.array([x, y, z]))
    Rs = quat_R(*q_s)
    aerr = np.degrees(np.arccos(np.clip((np.trace(Rs.T @ R) - 1) / 2, -1, 1)))
    print(f"IK check: tcp {tcp_s.round(4)} pos_err={perr*1000:.1f}mm ang_err={aerr:.2f}deg")
    if perr > 0.005 or aerr > 3:
        raise SystemExit("IK solution does not match target; no motion")
    import os
    maxd = float(os.environ.get("MAXDELTA", "99"))
    delta = max(abs(sol[j] - jsd[j]) for j in JOINTS)
    print(f"max joint delta from current: {delta:.3f}")
    if delta > maxd:
        raise SystemExit(f"IK solution reconfigures the arm (delta {delta:.2f} > {maxd}); no motion")
    client = ActionClient(node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
    client.wait_for_server(timeout_sec=10)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    pt = JointTrajectoryPoint(positions=target)
    pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
    goal.trajectory.points = [pt]
    for attempt in range(3):
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send)
        result = send.result().get_result_async()
        rclpy.spin_until_future_complete(node, result)
        print("traj error_code", result.result().result.error_code)
        jsd = joint_state(node)
        err = max(abs(jsd[j] - t) for j, t in zip(JOINTS, target))
        if err < 0.02:
            break
        print(f"joint err {err:.3f}, resending")
    hand, tcp, q = fk_world(node, jsd)
    print("max joint err", round(err, 4))
    print("hand", hand.round(4), "tcp", tcp.round(4), "q", np.round(q, 4))
    print("fingers", round(jsd["panda_finger_joint1"], 4), round(jsd["panda_finger_joint2"], 4))


if __name__ == "__main__":
    main()
