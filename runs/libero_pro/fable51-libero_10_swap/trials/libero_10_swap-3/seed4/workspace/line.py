#!/usr/bin/env python3
"""Straight-line TCP move: IK at N waypoints -> one multi-point trajectory.

Usage: python3 line.py x0 y0 z0 x1 y1 z1 qx qy qz qw seconds [n=6]
Quaternion is the HAND orientation (converted to link8 for IK).
"""
import sys
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, "/workspace")
from move import quat_R, qmul  # noqa: E402


def main():
    a = [float(v) for v in sys.argv[1:]]
    p0, p1 = np.array(a[0:3]), np.array(a[3:6])
    q = tuple(a[6:10]); secs = a[10]; n = int(a[11]) if len(a) > 11 else 6
    M = yaml.safe_load(open("/workspace/machine.yaml"))
    tj = next(e for e in M["actuators"] if e["kind"] == "joint_trajectory")
    off = M["hand"]["tcp_offset_m"]
    R = quat_R(*q)
    lq = qmul(q, (0.0, 0.0, 0.3826834, 0.9238795))

    rclpy.init()
    node = rclpy.create_node("liner")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cur = dict(zip(js["m"].name, js["m"].position))
    seed = [cur[j] for j in tj["joints"]]
    cli = node.create_client(GetPositionIK, M["planning"]["ik_service"])
    cli.wait_for_service(timeout_sec=10)

    pts = []
    for i in range(1, n + 1):
        tcp = p0 + (p1 - p0) * i / n
        hand = tcp - off * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = [float(v) for v in hand]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = lq
        s = JointState()
        for j, v in zip(tj["joints"], seed):
            s.name.append(j); s.position.append(float(v))
        req.ik_request.robot_state.joint_state = s
        req.ik_request.avoid_collisions = False
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK FAILED at waypoint {i} {tcp}", None if res is None else res.error_code.val); sys.exit(2)
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        seed = [sol[j] for j in tj["joints"]]
        pt = JointTrajectoryPoint(positions=seed)
        t = secs * i / n
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        pts.append(pt)
        print(f"wp{i} {np.round(tcp,3)} ->", " ".join(f"{v:.3f}" for v in seed))

    ac = ActionClient(node, FollowJointTrajectory, tj["port"])
    ac.wait_for_server(timeout_sec=10)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = tj["joints"]
    goal.trajectory.points = pts
    send = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    rf = send.result().get_result_async()
    rclpy.spin_until_future_complete(node, rf)
    print("traj error_code", rf.result().result.error_code)

    js.pop("m", None)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cur = dict(zip(js["m"].name, js["m"].position))
    err = max(abs(cur[j] - t) for j, t in zip(tj["joints"], seed))
    print(f"max joint err vs final {err:.4f} rad; fingers {cur['panda_finger_joint1']:.4f} {cur['panda_finger_joint2']:.4f}")
    buf = Buffer(); TransformListener(buf, node)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform("world", "panda_hand", rclpy.time.Time()):
            break
    t = buf.lookup_transform("world", "panda_hand", rclpy.time.Time())
    tr, qq = t.transform.translation, t.transform.rotation
    Rh = quat_R(qq.x, qq.y, qq.z, qq.w)
    tcp = np.array([tr.x, tr.y, tr.z]) + off * Rh[:, 2]
    print(f"TCP world {tcp[0]:.4f} {tcp[1]:.4f} {tcp[2]:.4f}  (target {p1})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
