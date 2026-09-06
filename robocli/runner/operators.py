"""Who acts on the robot during a trial.

- ``none``: nobody acts (plumbing tests; trials score false).
- ``script``: a caller-supplied command file runs inside the sandbox on
  the native surface (the same shell the agent gets; no privileged
  verbs). Canonical source: a trial's commands.sh. Open-loop replay.
- ``agent``: the headless agent (``session.agent_operator``).

An operator takes the trial context dict and returns the metadata
recorded under ``operator_meta``.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

from robocli.runner.session import agent_operator


def none_operator(ctx: dict) -> dict:
    return {"operator": "none"}


def script_operator(ctx: dict) -> dict:
    """Run a caller-supplied command sequence in the sandbox.

    Surface-only by construction: the file is copied into the sandbox
    and executed by bash as the sandbox user with ROS sourced, the same
    shell the agent gets; no control-line verb is touched. The wall-clock
    cap is enforced here like the agent's; the verdict comes from the
    ordinary blind ask afterwards."""
    path = ctx.get("script")
    if not path:
        raise ValueError("--operator script requires --script <file>")
    body = Path(path).read_text()
    name = ctx["sandbox"]
    subprocess.run(
        ["docker", "exec", "-i", name, "sh", "-c",
         "cat > /tmp/operator-script.sh"],
        input=body, text=True, capture_output=True,
    )
    meta: dict = {"operator": "script", "script": str(path)}
    try:
        r = subprocess.run(
            ["docker", "exec", name, "bash", "-c",
             "source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash 2>/dev/null; "
             "cd /workspace && bash /tmp/operator-script.sh"],
            timeout=ctx["active_wall_clock_min"] * 60,
            capture_output=True, text=True,
        )
        meta["termination"] = "self_finished"
        meta["returncode"] = r.returncode
        meta["output_tail"] = (r.stdout + r.stderr)[-2000:]
    except subprocess.TimeoutExpired:
        meta["termination"] = "wall_clock_cap"
    return meta


OPERATORS = {
    "none": none_operator,
    "script": script_operator,
    "agent": agent_operator,
}
