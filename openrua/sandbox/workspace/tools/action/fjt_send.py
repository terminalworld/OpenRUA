#!/usr/bin/env python3
"""Send one joint-space trajectory goal, wait for the result, then read
where the arm actually is.

Usage: python3 tools/action/fjt_send.py <p1,p2,...,pN> <seconds> [--arm LABEL]
Joint names and action port come from machine.yaml (kind
joint_trajectory); N must match that entry's joint count. --arm picks the
arm on a multi-arm machine. After the result it prints the largest gap
between target and /joint_states: a controller can report success short
of the target, or report failure for a move that happened. Interrupted
(SIGINT, SIGTERM, SIGHUP) while the arm moves, it cancels the goal before
exiting, as `ros2 action send_goal` does on Ctrl-C; otherwise the
controller would finish the motion on its own.
"""
import argparse
import signal
import sys
from pathlib import Path

import yaml


def manifest_entry(arm: str | None) -> dict:
    root = Path(__file__).resolve().parents[2]
    m = yaml.safe_load((root / "machine.yaml").read_text())
    entries = [a for a in m["actuators"] if a["kind"] == "joint_trajectory"]
    if not entries:
        raise SystemExit("machine.yaml lists no joint_trajectory port")
    if arm is None and len(entries) > 1:
        raise SystemExit("this machine has several arms; pick one with --arm "
                         + "/".join(e.get("arm", "?") for e in entries))
    entry = next((e for e in entries if arm in (None, e.get("arm"))), None)
    if entry is None:
        raise SystemExit(f"no joint_trajectory port for arm {arm!r} (see machine.yaml)")
    return entry


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("positions", help="comma-separated joint positions, radians")
    ap.add_argument("seconds", type=float, help="time to reach them")
    ap.add_argument("--arm", default=None, help="arm label on a multi-arm machine")
    args = ap.parse_args()
    import rclpy
    from action_msgs.msg import GoalStatus
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectoryPoint

    entry = manifest_entry(args.arm)
    joints, action = entry["joints"], entry["port"]
    positions = [float(x) for x in args.positions.split(",")]
    seconds = args.seconds
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
        # A signal only sets a flag: the cancel request goes out from this
        # loop, which wakes often enough to see it.
        stop: list[int] = []
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, lambda signum, _frame: stop.append(signum))
        while not result.done() and not stop:
            rclpy.spin_until_future_complete(node, result, timeout_sec=0.2)
        if stop:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
            answer = cancel.result()
            print("interrupted: trajectory goal "
                  + ("canceled" if answer is not None and answer.goals_canceling
                     else "cancel not confirmed; the arm may still be moving"), file=sys.stderr)
            return 128 + stop[0]
        outcome = result.result()
        if outcome is None:
            print(f"trajectory result unavailable from {action}; outcome unknown", file=sys.stderr)
            return 1
        code = outcome.result.error_code
        print(f"done error_code={code}" + ("" if code == 0 else " (nonzero = tracking problem)"))
        # Where the arm is, by its own sensors.
        seen = {}
        node.create_subscription(JointState, "/joint_states",
                                 lambda msg: seen.update(zip(msg.name, msg.position)), 10)
        end = node.get_clock().now().nanoseconds / 1e9 + 3.0
        while not set(joints) <= seen.keys() and node.get_clock().now().nanoseconds / 1e9 < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if set(joints) <= seen.keys():
            gaps = {j: abs(seen[j] - p) for j, p in zip(joints, positions)}
            worst = max(gaps, key=gaps.get)
            print(f"largest gap to target: {gaps[worst]:.4f} rad ({worst})")
        else:
            print("could not read every arm joint from /joint_states", file=sys.stderr)
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
