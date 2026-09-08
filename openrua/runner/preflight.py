"""Preflight: every promise the workspace docs make, checked before the agent starts.

Each check is asserted from the sandbox's own vantage (the same shell
and the same DDS view the agent gets). A red check refuses the trial:
the record shows an anomaly and the agent never pays for a wrong
workspace description. Success predicates are not here; scoring stays
with the benchmark's own code in the bridge, and this module verifies
the apparatus only.

Three sources of checks:
1. machine.yaml is generated from the resolved config, and so are the
   checks, so the two cannot drift; a capability the config does not
   declare gets a negative check.
2. Claims made in the workspace prose are extracted by hand into named
   checks (a finite set; shell provisioning, for one).
3. Every failure found in a trial adds a regression check.

This gate proves that the workspace docs do not lie, not that the robot
has no defects; unknown defects are found by reading transcripts and
feed new checks back here.

Leaf: no openrua imports; the agent and the container name are handed
in by the runner.
"""

from __future__ import annotations

import subprocess

_DEF_GRIPPER = "/franka_gripper/gripper_action"

# Per-check ceiling for the ROS topic/action/flow probes. A ceiling, not
# a settle time: `topic echo --once` returns the instant the first
# message lands, so a healthy bring-up pays nothing; the timeout only
# bounds a slow one. 60 s covers a bimanual bring-up under heavy host
# load, where two arm controllers and doubled TF take longer to publish
# their first message.
_CHECK_TIMEOUT_S = 60


def build_checks(cfg: dict, agent) -> list[tuple[str, str]]:
    """(name, bash snippet exiting 0 on pass) pairs, from the resolved
    config after suite overrides, the same source machine.yaml is
    generated from. The agent is handed in by the runner."""
    m = cfg.get("machine", {})
    ports = m.get("ports", {})
    checks: list[tuple[str, str]] = [
        # The workspace docs say ROS is sourced in every shell, /bin/sh
        # included.
        ("shell_sh_provisioned", "sh -c 'command -v ros2'"),
        # The agent's credentials are mounted readably, and the sandbox
        # CLI is the version the manifest pins. Both commands are the
        # agent's own knowledge; either is None for an agent without
        # that arrangement, and a None check is not minted.
        *(c for c in (agent.credentials_check(), agent.sandbox_cli_check()) if c),
        # Polls until /clock is registered rather than asking once: the
        # other probes block until their first message, but `topic list
        # | grep` returns at once and would fail any bring-up slower than
        # the moment it ran. A fast bring-up still pays nothing.
        ("clock_topic",
         f"timeout {_CHECK_TIMEOUT_S} bash -c "
         "'until ros2 topic list | grep -qx /clock; do sleep 1; done'"),
        ("joint_states_flow",
         f"timeout {_CHECK_TIMEOUT_S} ros2 topic echo --once /joint_states"),
    ]
    # Declared frames are a promise: the pixel-to-world tool needs the
    # camera/world transform as much as depth and intrinsics, so the
    # transform stream is asserted, not assumed.
    if m.get("frames"):
        checks.append(("tf_flow", f"timeout {_CHECK_TIMEOUT_S} ros2 topic echo --once /tf"))
    # Per-arm checks over the normalized view (normalize_arms wrote
    # machine.arms into the resolved config); a single-arm machine has
    # one unlabeled arm and mints the unsuffixed check names.
    for arm in m.get("arms", []):
        sfx = f"_{arm['label']}" if arm.get("label") else ""
        aports = arm.get("ports", {})
        # The documented joint names must be the ones actually flowing: a
        # joint_name_map slip would have the agent commanding panda_joint1
        # while the robot publishes robot0_joint1.
        joints = arm.get("joints", [])
        if joints:
            checks.append((f"joint_names_match_manual{sfx}",
                           f"timeout {_CHECK_TIMEOUT_S} bash -c \"ros2 topic echo --once "
                           f"/joint_states | grep -q '{joints[0]}'\""))
        if aports.get("trajectory"):
            checks.append((f"trajectory_action{sfx}",
                           f"timeout {_CHECK_TIMEOUT_S} bash -c \"ros2 action list | grep -qx "
                           f"'{aports['trajectory']}'\""))
        if aports.get("twist"):
            checks.append((f"twist_subscribed{sfx}",
                           f"timeout {_CHECK_TIMEOUT_S} bash -c \"ros2 topic info "
                           f"'{aports['twist']}'"
                           f" | grep -q 'Subscription count: [1-9]'\""))
        if aports.get("gripper"):
            checks.append((f"gripper_action{sfx}",
                           f"timeout {_CHECK_TIMEOUT_S} bash -c \"ros2 action list | grep -qx "
                           f"'{aports['gripper']}'\""))
        if aports.get("wrench"):
            checks.append((f"wrench_flow{sfx}",
                           f"timeout {_CHECK_TIMEOUT_S} ros2 topic echo --once "
                           f"'{aports['wrench']}'"))
    if not any(a.get("ports", {}).get("gripper") for a in m.get("arms", [])):
        # Negative check: a machine whose config lists no gripper must
        # not expose one.
        checks.append(("gripper_absent",
                       f"timeout {_CHECK_TIMEOUT_S} bash -c \"! ros2 action list | grep -qx "
                       f"'{_DEF_GRIPPER}'\""))
    if ports.get("base_twist"):
        checks.append(("base_twist_subscribed",
                       f"timeout {_CHECK_TIMEOUT_S} bash -c \"ros2 topic info "
                       f"'{ports['base_twist']}' | grep -q "
                       f"'Subscription count: [1-9]'\""))
    if ports.get("odom"):
        checks.append(("odom_flow",
                       f"timeout {_CHECK_TIMEOUT_S} ros2 topic echo --once '{ports['odom']}'"))
    # Cameras: at least one color stream must actually deliver a frame
    # (list may be null = scene-defined; discover instead of assuming).
    checks.append(("camera_frame_flow",
                   "timeout 60 bash -c 'cam=$(ros2 topic list | grep -m1 "
                   "color/image_raw) && [ -n \"$cam\" ] && "
                   "ros2 topic echo --once \"$cam\" >/dev/null'"))
    # machine.yaml promises depth and intrinsics with every color stream;
    # the pixel-to-world tool dies without them, so the promise is
    # asserted, not assumed.
    checks.append(("camera_depth_intrinsics_flow",
                   "timeout 60 bash -c 'cam=$(ros2 topic list | grep -m1 "
                   "color/image_raw) && [ -n \"$cam\" ] && "
                   "base=${cam%/color/image_raw} && "
                   "ros2 topic echo --once \"$base/depth/image_raw\" "
                   ">/dev/null && ros2 topic echo --once "
                   "\"$base/color/camera_info\" >/dev/null'"))
    # The workspace tools must actually run: import each tool module
    # (all carry __main__ guards, so an import checks dependencies and
    # syntax without side effects).
    if m.get("workspace_template"):
        checks.append(("workspace_tools_importable",
                       'timeout 60 python3 -c "import glob, importlib.util as u; '
                       "fs = glob.glob('/workspace/tools/*/*.py'); "
                       "assert fs, 'no tools in workspace'; "
                       "[(s := u.spec_from_file_location('tool_smoke', f))"
                       '.loader.exec_module(u.module_from_spec(s)) for f in fs]"'))
    if m.get("planning", {}).get("moveit"):
        ik = m["planning"].get("ik_service", "/compute_ik")
        # The service must answer, not merely be listed.
        checks.append(("moveit_ik_answers",
                       f"timeout 45 ros2 service call '{ik}' "
                       f"moveit_msgs/srv/GetPositionIK '{{}}'"))
    return checks


def run_preflight(cfg: dict, sandbox_name: str, agent,
                 timeout_s: float | None = None) -> dict:
    """Execute all checks inside the sandbox; returns
    {ok, checks: [(name, passed)], failed: [names]}."""
    # The checks run sequentially in one bash script, so the wrapper
    # budget covers every per-check ceiling firing in a row; a shorter
    # budget would surface as a raw TimeoutExpired instead of a readable
    # list of red checks. Derived from the ceiling and the count, so
    # raising the ceiling cannot under-budget the wrapper.
    checks = build_checks(cfg, agent)
    if timeout_s is None:
        timeout_s = len(checks) * _CHECK_TIMEOUT_S + 60.0
    lines = ["set -u", "fail=0"]
    for name, snippet in checks:
        lines.append(
            f"if {snippet} >/dev/null 2>&1; then echo \"OK {name}\"; "
            f"else echo \"FAIL {name}\"; fail=1; fi")
    lines.append("exit $fail")
    script = "\n".join(lines)
    r = subprocess.run(
        ["docker", "exec", sandbox_name, "bash", "-c", script],
        capture_output=True, text=True, timeout=timeout_s,
    )
    results, failed = [], []
    for line in r.stdout.splitlines():
        if line.startswith(("OK ", "FAIL ")):
            status, name = line.split(" ", 1)
            results.append((name, status == "OK"))
            if status == "FAIL":
                failed.append(name)
    if not results:  # exec itself broke; that too is a red gate
        failed = ["preflight_script_did_not_run"]
    return {"ok": r.returncode == 0 and not failed,
            "checks": results, "failed": failed}
