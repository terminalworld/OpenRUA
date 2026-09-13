#!/usr/bin/env python3
"""Move the FINGERTIP point (TCP) to a world pose: IK -> trajectory -> verify.

Usage: python3 move.py <x> <y> <z> <qx> <qy> <qz> <qw> [seconds=4]
Prints the resulting hand/TCP pose from TF and the joint tracking error.
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


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def main():
    a = [float(v) for v in sys.argv[1:]]
    x, y, z, qx, qy, qz, qw = a[:7]
    secs = a[7] if len(a) > 7 else 4.0
    M = yaml.safe_load(open("/workspace/machine.yaml"))
    tj = next(e for e in M["actuators"] if e["kind"] == "joint_trajectory")
    off = M["hand"]["tcp_offset_m"]
    R = quat_R(qx, qy, qz, qw)
    hand_b = np.array([x, y, z]) - off * R[:, 2]  # FK/IK here report in world
    # IK tip link is panda_link8 = hand rotated +45 deg about its z
    lx, ly, lz, lw = qmul((qx, qy, qz, qw), (0.0, 0.0, 0.3826834, 0.9238795))

    rclpy.init()
    node = rclpy.create_node("mover")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cli = node.create_client(GetPositionIK, M["planning"]["ik_service"])
    cli.wait_for_service(timeout_sec=10)
    req = GetPositionIK.Request()
    req.ik_request.group_name = M["planning"]["group"]
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = [float(v) for v in hand_b]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = lx, ly, lz, lw
    seed = JointState()
    cur = dict(zip(js["m"].name, js["m"].position))
    for j in tj["joints"]:
        seed.name.append(j); seed.position.append(cur[j])
    req.ik_request.robot_state.joint_state = seed
    req.ik_request.avoid_collisions = False
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    res = fut.result()
    if res is None or res.error_code.val != 1:
        print("IK FAILED", None if res is None else res.error_code.val); sys.exit(2)
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    target = [sol[j] for j in tj["joints"]]
    print("IK ok:", " ".join(f"{v:.3f}" for v in target))

    ac = ActionClient(node, FollowJointTrajectory, tj["port"])
    ac.wait_for_server(timeout_sec=10)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = tj["joints"]
    pt = JointTrajectoryPoint(positions=target)
    pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
    goal.trajectory.points = [pt]
    send = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    rf = send.result().get_result_async()
    rclpy.spin_until_future_complete(node, rf)
    print("traj error_code", rf.result().result.error_code)

    # verify
    js.pop("m", None)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cur = dict(zip(js["m"].name, js["m"].position))
    err = max(abs(cur[j] - t) for j, t in zip(tj["joints"], target))
    print(f"max joint err {err:.4f} rad; fingers {cur['panda_finger_joint1']:.4f} {cur['panda_finger_joint2']:.4f}")
    buf = Buffer(); TransformListener(buf, node)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform("world", "panda_hand", rclpy.time.Time()):
            break
    t = buf.lookup_transform("world", "panda_hand", rclpy.time.Time())
    tr, q = t.transform.translation, t.transform.rotation
    Rh = quat_R(q.x, q.y, q.z, q.w)
    tcp = np.array([tr.x, tr.y, tr.z]) + off * Rh[:, 2]
    print(f"hand world {tr.x:.4f} {tr.y:.4f} {tr.z:.4f} q {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}")
    print(f"TCP world {tcp[0]:.4f} {tcp[1]:.4f} {tcp[2]:.4f}  (target {x} {y} {z})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
