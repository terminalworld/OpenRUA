#!/usr/bin/env python3
"""Send one joint-space trajectory goal and wait for the result.

Usage: python3 tools/action/fjt_send.py <p1,p2,...,pN> <seconds>
Joint names and action port come from machine.yaml (kind
joint_trajectory); N must match that entry's joint count.
"""
import sys
from pathlib import Path

import yaml


def manifest_entry() -> dict:
    root = Path(__file__).resolve().parents[2]
    m = yaml.safe_load((root / "machine.yaml").read_text())
    return next(a for a in m["actuators"] if a["kind"] == "joint_trajectory")


def main() -> int:
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    import rclpy
    from action_msgs.msg import GoalStatus
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from trajectory_msgs.msg import JointTrajectoryPoint

    entry = manifest_entry()
    joints, action = entry["joints"], entry["port"]
    positions = [float(x) for x in sys.argv[1].split(",")]
    seconds = float(sys.argv[2])
    if len(positions) != len(joints):
        raise SystemExit(f"need {len(joints)} positions ({joints}), "
                         f"got {len(positions)}")
    rclpy.init()
    node = rclpy.create_node("fjt_send")
    try:
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
        handle = send.result()
        if handle is None or not handle.accepted:
            print(f"trajectory goal rejected by {action}; no trajectory accepted", file=sys.stderr)
            return 1
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result)
        outcome = result.result()
        if outcome is None:
            print(f"trajectory result unavailable from {action}; outcome unknown", file=sys.stderr)
            return 1
        code = outcome.result.error_code
        print(f"done error_code={code}" + ("" if code == 0 else " (nonzero = tracking problem)"))
        if outcome.status != GoalStatus.STATUS_SUCCEEDED or code != 0:
            print(f"trajectory failed: status={outcome.status} error_code={code} "
                  f"error_string={outcome.result.error_string}", file=sys.stderr)
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
