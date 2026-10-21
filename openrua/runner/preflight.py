"""Check declared interfaces from the trial sandbox before starting its agent.

The resolved config supplies the same port names used in machine.yaml.
Simulation checks also verify the bridge's clock and gripper absence.
An external real graph may contain interfaces beyond the profile's index.
These checks validate selected apparatus properties, not task success or
physical safety. Interactive startup separately waits for graph visibility.

Leaf: no openrua imports; the agent and container name are explicit inputs.
"""

from __future__ import annotations

import subprocess

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
    ]
    simulated = m.get("backend", {}).get("kind") == "sim"
    if simulated:
        checks.append(("clock_topic",
                       f"timeout {_CHECK_TIMEOUT_S} bash -c "
                       "'until ros2 topic list | grep -qx /clock; do sleep 1; done'"))
    if m.get("base") or any(a.get("joints") or a.get("gripper") for a in m.get("arms", [])):
        checks.append(("joint_states_flow",
                       f"timeout {_CHECK_TIMEOUT_S} ros2 topic echo --once /joint_states"))
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
        grip_joint = (arm.get("gripper") or {}).get("joint")
        if grip_joint:
            # the declared joint is where the agent reads the gripper from
            checks.append((f"gripper_joint_reported{sfx}",
                           f"timeout {_CHECK_TIMEOUT_S} bash -c \"ros2 topic echo --once "
                           f"/joint_states | grep -q '{grip_joint}'\""))
        if aports.get("gripper"):
            checks.append((f"gripper_action{sfx}",
                           f"timeout {_CHECK_TIMEOUT_S} bash -c \"ros2 action list | grep -qx "
                           f"'{aports['gripper']}'\""))
        if aports.get("wrench"):
            checks.append((f"wrench_flow{sfx}",
                           f"timeout {_CHECK_TIMEOUT_S} ros2 topic echo --once "
                           f"'{aports['wrench']}'"))
    if simulated and not any(a.get("ports", {}).get("gripper") for a in m.get("arms", [])):
        # A scene that removed the gripper must not still serve one: no
        # GripperCommand action at all, whatever its name.
        checks.append(("gripper_absent",
                       f"timeout {_CHECK_TIMEOUT_S} bash -c \"! ros2 action list -t | grep -q "
                       "'control_msgs/action/GripperCommand'\""))
    if ports.get("base_twist"):
        checks.append(("base_twist_subscribed",
                       f"timeout {_CHECK_TIMEOUT_S} bash -c \"ros2 topic info "
                       f"'{ports['base_twist']}' | grep -q "
                       f"'Subscription count: [1-9]'\""))
    if ports.get("odom"):
        checks.append(("odom_flow",
                       f"timeout {_CHECK_TIMEOUT_S} ros2 topic echo --once '{ports['odom']}'"))
    # Cameras: every declared camera's streams must deliver (the
    # pixel-to-world tool dies without depth and intrinsics); a scene
    # that decides its cameras (list: null) must deliver at least one,
    # found by its type's topic pattern. [] declares none.
    cams = m.get("cameras", {})
    types = cams.get("types") or {}
    entries = cams.get("list")
    if entries is None and types.get("openrua-sim"):
        t = types["openrua-sim"]["topics"]
        color, depth, info = (t[k].split("{name}")[-1] for k in ("color", "depth", "info"))
        checks.append(("camera_frame_flow",
                       f"timeout 60 bash -c 'cam=$(ros2 topic list | grep -m1 {color}$) && "
                       "[ -n \"$cam\" ] && ros2 topic echo --once \"$cam\" >/dev/null'"))
        checks.append(("camera_depth_intrinsics_flow",
                       f"timeout 60 bash -c 'cam=$(ros2 topic list | grep -m1 {color}$) && "
                       f"[ -n \"$cam\" ] && base=${{cam%{color}}} && "
                       f"ros2 topic echo --once \"$base{depth}\" >/dev/null && "
                       f"ros2 topic echo --once \"$base{info}\" >/dev/null'"))
    for e in entries or []:
        e = {"name": e, "type": "openrua-sim"} if isinstance(e, str) else e
        topics = {**types[e["type"]]["topics"], **(e.get("topics") or {})}
        streams = [topics[k].replace("{name}", e["name"])
                   for k in ("color", "depth", "info") if topics.get(k)]
        checks.append((f"camera_flow_{e['name']}",
                       f"timeout {_CHECK_TIMEOUT_S} bash -c '"
                       + " && ".join(f"ros2 topic echo --once {t} >/dev/null" for t in streams)
                       + "'"))
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
