#!/usr/bin/env python3
"""Send one joint-space trajectory goal and wait for the result.

Usage: python3 tools/action/fjt_send.py <p1,p2,...,pN> <seconds>
Joint names and action port come from machine.yaml (kind
joint_trajectory); N must match that entry's joint count.
"""
import sys
from pathlib import Path

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint


def manifest_entry() -> dict:
    root = Path(__file__).resolve().parents[2]
    m = yaml.safe_load((root / "machine.yaml").read_text())
    return next(a for a in m["actuators"] if a["kind"] == "joint_trajectory")


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    entry = manifest_entry()
    joints, action = entry["joints"], entry["port"]
    positions = [float(x) for x in sys.argv[1].split(",")]
    seconds = float(sys.argv[2])
    if len(positions) != len(joints):
        raise SystemExit(f"need {len(joints)} positions ({joints}), "
                         f"got {len(positions)}")
    rclpy.init()
    node = rclpy.create_node("fjt_send")
    client = ActionClient(node, FollowJointTrajectory, action)
    if not client.wait_for_server(timeout_sec=10.0):
        raise SystemExit(f"no action server at {action}")
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = joints
    pt = JointTrajectoryPoint(positions=positions)
    pt.time_from_start = Duration(sec=int(seconds),
                                  nanosec=int((seconds % 1) * 1e9))
    goal.trajectory.points = [pt]
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    result = send.result().get_result_async()
    rclpy.spin_until_future_complete(node, result)
    code = result.result().result.error_code
    print(f"done error_code={code}" + ("" if code == 0 else " (nonzero = tracking problem)"))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
