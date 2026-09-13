#!/usr/bin/env python3
"""Open/close the gripper to a target width and wait for the result.

Usage: python3 tools/action/gripper_cmd.py <width_m>
width_m is the desired PER-FINGER position (machine.yaml gripper:
open_m..closed_m). Command a target width, not 0: driving to 0 against
an object keeps squeezing it. Absent from the manifest = this machine
has no gripper.
"""
import sys
from pathlib import Path

import rclpy
import yaml
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    width = float(sys.argv[1])
    root = Path(__file__).resolve().parents[2]
    m = yaml.safe_load((root / "machine.yaml").read_text())
    entry = next((a for a in m["actuators"] if a["kind"] == "gripper"), None)
    if entry is None:
        raise SystemExit("this machine has no gripper (see machine.yaml)")
    rclpy.init()
    node = rclpy.create_node("gripper_cmd")
    client = ActionClient(node, GripperCommand, entry["port"])
    if not client.wait_for_server(timeout_sec=10.0):
        raise SystemExit(f"no action server at {entry['port']}")
    goal = GripperCommand.Goal()
    goal.command.position = width
    goal.command.max_effort = float(entry.get("max_effort", 30.0))
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    handle = fut.result()
    res_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut, timeout_sec=120)
    res = res_fut.result().result
    print(f"reached_goal={res.reached_goal} stalled={res.stalled}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
