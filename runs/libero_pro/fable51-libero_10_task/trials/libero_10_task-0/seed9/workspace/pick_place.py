#!/usr/bin/env python3
"""Pick each target with a top-down grasp and drop it into the basket.
Reusable IK / trajectory / gripper clients; verifies each step from
/joint_states. Usage: python3 -u pick_place.py [can] [cream] > log
"""
import sys, time
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["panda_joint1","panda_joint2","panda_joint3","panda_joint4",
          "panda_joint5","panda_joint6","panda_joint7"]
TCP = 0.1034          # hand frame -> fingertip, along hand +Z
Q_DOWN = (1.0, 0.0, 0.0, 0.0)   # hand Z down, fingers along world Y
TABLE = 0.425
BASKET = (0.0, 0.265)
TARGETS = {
    # name: (x, y, grasp_tcp_z, top_z, release_tcp_z)
    "can":   (-0.067, 0.070, 0.470, 0.520, 0.70),
    "cream": (0.107, -0.191, 0.437, 0.454, 0.68),
}

rclpy.init()
node = rclpy.create_node("pick_place")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.update(m=m), 10)
ik_cli = node.create_client(GetPositionIK, "/compute_ik")
fk_cli = node.create_client(GetPositionFK, "/compute_fk")
fjt = ActionClient(node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
grip = ActionClient(node, GripperCommand, "/franka_gripper/gripper_action")
ik_cli.wait_for_service(20); fk_cli.wait_for_service(20)
fjt.wait_for_server(20); grip.wait_for_server(20)


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def joints(fresh=True):
    if fresh:
        js.pop("m", None)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    d = dict(zip(js["m"].name, js["m"].position))
    return [d[j] for j in JOINTS], (d["panda_finger_joint1"], d["panda_finger_joint2"])


def fk(q):
    req = GetPositionFK.Request()
    req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state = JointState(name=JOINTS, position=q)
    fut = fk_cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    p = fut.result().pose_stamped[0].pose
    x, y, z, w = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    zaxis = np.array([2*(x*z + y*w), 2*(y*z - x*w), 1 - 2*(x*x + y*y)])
    tcp = np.array([p.position.x, p.position.y, p.position.z]) + TCP * zaxis
    return tcp


def ik(tcp_xyz, q=Q_DOWN, seed=None):
    """TCP pose (world) -> arm joints, or None."""
    x, y, z, w = q
    zaxis = np.array([2*(x*z + y*w), 2*(y*z - x*w), 1 - 2*(x*x + y*y)])
    hand = np.array(tcp_xyz) - TCP * zaxis
    req = GetPositionIK.Request()
    r = req.ik_request
    r.group_name = "panda_arm"
    r.ik_link_name = "panda_hand"
    r.pose_stamped.header.frame_id = ""
    r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = map(float, hand)
    r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y = x, y
    r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = z, w
    r.robot_state.joint_state = JointState(name=JOINTS, position=seed or joints()[0])
    r.avoid_collisions = False
    r.timeout = Duration(sec=2)
    for attempt in range(3):
        fut = ik_cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
        res = fut.result()
        if res is not None and res.error_code.val == 1:
            sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            return [sol[j] for j in JOINTS]
        log(f"  IK attempt {attempt} failed:", None if res is None else res.error_code.val)
    return None


def move(q, secs):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    pt = JointTrajectoryPoint(positions=[float(v) for v in q])
    pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
    goal.trajectory.points = [pt]
    for attempt in range(3):
        fut = fjt.send_goal_async(goal); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
        rf = fut.result().get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        cur, _ = joints()
        err = max(abs(a - b) for a, b in zip(cur, q))
        log(f"  move done code={code} max_joint_err={err:.4f} tcp={np.round(fk(cur),3)}")
        if err < 0.02:
            return True
    return False


def gripper(width):
    g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
    fut = grip.send_goal_async(g); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    rf = fut.result().get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=300)
    r = rf.result().result
    _, f = joints()
    log(f"  gripper({width}) reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
    return f


def goto_tcp(xyz, secs, q=Q_DOWN):
    log(f" goto tcp {np.round(xyz,3)}")
    sol = ik(xyz, q)
    if sol is None:
        raise SystemExit(f"IK failed for {xyz}")
    if not move(sol, secs):
        raise SystemExit(f"move failed for {xyz}")


def pick_and_place(name):
    x, y, gz, top, rz = TARGETS[name]
    log(f"=== {name} ===")
    gripper(0.04)
    cur = fk(joints()[0])
    goto_tcp((cur[0], cur[1], 0.72), 3.0) if cur[2] < 0.70 else None
    goto_tcp((x, y, top + 0.12), 4.0)            # pre-grasp above
    goto_tcp((x, y, gz), 2.5)                    # descend
    f = gripper(0.0)
    if f[0] < 0.004:
        log("  !! closed on air")
        return False
    goto_tcp((x, y, 0.72), 2.5)                  # lift
    _, f = joints()
    log(f"  after lift fingers={f[0]:.4f} (still holding: {f[0] > 0.004})")
    goto_tcp((BASKET[0], BASKET[1], 0.72), 4.0)  # over basket
    goto_tcp((BASKET[0], BASKET[1], rz), 2.0)
    gripper(0.04)
    goto_tcp((BASKET[0], BASKET[1], 0.75), 2.0)  # retreat
    return True


names = sys.argv[1:] or ["can", "cream"]
log("start joints", np.round(joints()[0], 3), "tcp", np.round(fk(joints()[0]), 3))
for n in names:
    ok = pick_and_place(n)
    log(f"=== {n}: {'OK' if ok else 'FAILED'}")
    if not ok:
        break
rclpy.shutdown()
