#!/usr/bin/env python3
"""Send the gripper to a position and wait for the result.

Usage: python3 tools/action/gripper_cmd.py <open|closed|position> [--arm LABEL]
                                           [--effort N]
`open` and `closed` are the positions machine.yaml's gripper entry
declares; a number is a GripperCommand position in that entry's `unit`.
Read `holds_partway` and `effort_honoured` on the entry before relying on
a position between the two or on the effort limit. --arm picks the
gripper of one arm on a multi-arm machine. Absent from the manifest =
this machine has no gripper. Interrupted (SIGINT, SIGTERM, SIGHUP) while
the gripper moves, it cancels the goal before exiting, as `ros2 action
send_goal` does on Ctrl-C.
"""
import argparse
import signal
import sys
import time
from pathlib import Path

import rclpy
import yaml
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", help="open, closed, or a position in the entry's unit")
    ap.add_argument("--arm", default=None, help="arm label on a multi-arm machine")
    ap.add_argument("--effort", type=float, default=None,
                    help="GripperCommand max_effort (default: the entry's max_effort)")
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[2]
    m = yaml.safe_load((root / "machine.yaml").read_text())
    grippers = [a for a in m["actuators"] if a["kind"] == "gripper"]
    if not grippers:
        raise SystemExit("this machine has no gripper (see machine.yaml)")
    if args.arm is None and len(grippers) > 1:
        raise SystemExit("this machine has several grippers; pick one with --arm "
                         + "/".join(g.get("arm", "?") for g in grippers))
    entry = next((g for g in grippers if args.arm in (None, g.get("arm"))), None)
    if entry is None:
        raise SystemExit(f"no gripper on arm {args.arm!r} (see machine.yaml)")
    if args.target in ("open", "closed"):
        position = float(entry[args.target])
    else:
        position = float(args.target)
    rclpy.init()
    node = rclpy.create_node("gripper_cmd")
    client = ActionClient(node, GripperCommand, entry["port"])
    if not client.wait_for_server(timeout_sec=10.0):
        raise SystemExit(f"no action server at {entry['port']}")
    goal = GripperCommand.Goal()
    goal.command.position = position
    goal.command.max_effort = float(args.effort if args.effort is not None
                                    else entry["max_effort"])
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    handle = fut.result()
    if handle is None or not handle.accepted:
        raise SystemExit(f"goal not accepted by {entry['port']}")
    res_fut = handle.get_result_async()
    # A signal only sets a flag: the cancel request goes out from this
    # loop, which wakes often enough to see it.
    stop: list[int] = []
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda signum, _frame: stop.append(signum))
    end = time.monotonic() + 120
    while not res_fut.done() and not stop and time.monotonic() < end:
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=0.2)
    if stop:
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
        answer = cancel.result()
        print("interrupted: gripper goal "
              + ("canceled" if answer is not None and answer.goals_canceling
                 else "cancel not confirmed"), file=sys.stderr)
        raise SystemExit(128 + stop[0])
    if not res_fut.done():
        raise SystemExit("no result within 120 s")
    res = res_fut.result().result
    unit = entry.get("unit", "")
    print(f"position={res.position:.4f} {unit} reached_goal={res.reached_goal} "
          f"stalled={res.stalled} (status {res_fut.result().status})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
