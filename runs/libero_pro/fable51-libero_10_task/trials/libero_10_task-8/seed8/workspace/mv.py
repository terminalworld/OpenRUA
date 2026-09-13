#!/usr/bin/env python3
"""IK (base frame) -> trajectory -> verify. Usage: mv.py x y z qx qy qz qw [sec] [--tcp]
Coordinates are WORLD frame; converted to base frame internally."""
import sys, time, numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

BASE = np.array([0.0, 0.0, 0.0])  # IK model frame is world here (verified via FK)
TCP_OFF = 0.1034
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]

def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def get_js(node):
    got = {}
    sub = node.create_subscription(JointState, "/joint_states", lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return dict(zip(got["m"].name, got["m"].position))

def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    x, y, z, qx, qy, qz, qw = map(float, args[:7])
    sec = float(args[7]) if len(args) > 7 else 4.0
    p = np.array([x, y, z])
    if "--tcp" in sys.argv:
        R = quat_R(qx, qy, qz, qw)
        p = p - TCP_OFF * R[:, 2]
    pb = p - BASE
    rclpy.init(); node = rclpy.create_node("mv")
    js = get_js(node)
    cli = node.create_client(GetPositionIK, "/compute_ik")
    cli.wait_for_service(10)
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"; req.ik_request.ik_link_name = "panda_hand"
    req.ik_request.pose_stamped.header.frame_id = ""
    pp = req.ik_request.pose_stamped.pose
    pp.position.x, pp.position.y, pp.position.z = map(float, pb)
    pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, (qx, qy, qz, qw))
    seed = JointState(); seed.name = JOINTS; seed.position = [js[j] for j in JOINTS]
    req.ik_request.robot_state.joint_state = seed
    req.ik_request.avoid_collisions = False
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    res = fut.result()
    if res is None or res.error_code.val != 1:
        raise SystemExit(f"IK FAILED code={None if res is None else res.error_code.val}")
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    target = [sol[j] for j in JOINTS]
    print("IK target:", np.round(target, 4))
    ac = ActionClient(node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
    ac.wait_for_server(10)
    goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
    pt = JointTrajectoryPoint(positions=target)
    pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
    goal.trajectory.points = [pt]
    f = ac.send_goal_async(goal); rclpy.spin_until_future_complete(node, f)
    rf = f.result().get_result_async(); rclpy.spin_until_future_complete(node, rf)
    print("error_code", rf.result().result.error_code)
    js = get_js(node)
    err = np.array([js[j] for j in JOINTS]) - np.array(target)
    print("joint err max", np.abs(err).max().round(4), "fingers", round(js["panda_finger_joint1"], 4))
    rclpy.shutdown()

if __name__ == "__main__":
    main()
