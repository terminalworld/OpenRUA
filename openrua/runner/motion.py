"""Stopping a robot's motion: which action servers move it, and asking
each of them, from the sandbox, to cancel every goal it holds. Used when
a person interrupts the agent (a conversation's Esc) and when a run is
stopped (Ctrl-C during ``openrua exec`` or ``bench``)."""

from __future__ import annotations

import subprocess


def motion_actions(machine: dict) -> list[str]:
    """The action servers the robot moves through: each arm's trajectory
    and gripper ports, and the motion planner's move action."""
    names: list[str] = []
    for arm in machine.get("arms") or []:
        for key in ("trajectory", "gripper"):
            port = (arm.get("ports") or {}).get(key)
            if port and port not in names:
                names.append(port)
    move = (machine.get("planning") or {}).get("move_action")
    if move and move not in names:
        names.append(move)
    return names


def cancel_motion(sandbox: str, actions: list[str], timeout_s: float = 10) -> dict[str, bool]:
    """Ask each action server, from the sandbox and all at once, to cancel
    every goal it holds (CancelGoal with a zero goal ID and stamp means
    all of them). A controller that cancels holds where it is."""
    calls = {action: subprocess.Popen(
        ["docker", "exec", sandbox, "bash", "-c",
         f"timeout {timeout_s:g} ros2 service call {action}/_action/cancel_goal "
         "action_msgs/srv/CancelGoal '{}'"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) for action in actions}
    outcome = {}
    for action, call in calls.items():
        try:
            outcome[action] = call.wait(timeout=timeout_s + 5) == 0
        except subprocess.TimeoutExpired:
            call.kill()
            outcome[action] = False
    return outcome
