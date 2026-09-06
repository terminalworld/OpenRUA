"""Pre-agent machine-manual precheck (A6, 2026-08-12; named 2026-08-18).

Before the agent boards, every verifiable promise the workspace manual
makes is asserted FROM THE SANDBOX'S OWN VANTAGE (same shell, same DDS
view the agent will use) by examiner-owned code. A red check refuses the
trial (anomaly -> classify INFRA -> free rerun); the agent never pays
for our manual being wrong. Success predicates are NOT here: scoring
identity keeps the benchmark's own predicates in the sim process
(robot.sim.bridge); this module only verifies the apparatus.

Three assertion sources (design discussion 2026-08-12):
1. machine.yaml is GENERATED from the resolved config; checks are
   generated from the same config, so manifest and gate cannot drift;
   absent capabilities get NEGATIVE checks (the wipe gripper lesson).
2. Prose-doc claims are extracted by hand into named checks (finite
   set; e.g. shell provisioning).
3. Every incident mints a regression check.

Coverage contract: this gate proves "the manual does not lie", NOT
"the machine has no defects"; unknown defects stay the job of the
canary + transcript-audit loop, which feeds new checks back here.

Leaf discipline: no robocli imports; the agent adapter and container
names are handed in by the conductor.
"""

from __future__ import annotations

import subprocess

_DEF_GRIPPER = "/franka_gripper/gripper_action"

# Per-check ceiling for the ROS topic/action/flow probes. It is a CEILING,
# not a settle: `topic echo --once` returns the instant the first message
# lands, so the happy path pays nothing; the timeout only bounds a slow
# bringup. Raised 30->60 (2026-08-19) after bimanual trials red-checked
# joint_states_flow/tf_flow at ~35s during a 7-way load spike (~50 load):
# two Panda controllers + doubled TF take longer to first-publish, and 30s
# was too tight under contention. Single-arm is unaffected (it passed at
# 30s and pays nothing for the higher ceiling). Not a measurement-chain
# change: precheck gates apparatus setup, never the scored episode.
_CHECK_TIMEOUT_S = 60


def build_checks(cfg: dict, agent) -> list[tuple[str, str]]:
    """(name, bash snippet exiting 0 on pass) pairs, from the assembly
    config AFTER suite overrides; the manifest's single source. The agent
    adapter is handed in by the conductor (leaf discipline: this package
    looks nothing up)."""
    m = cfg.get("machine", {})
    ports = m.get("ports", {})
    checks: list[tuple[str, str]] = [
        # docs 10-machine: "ROS is already sourced in every shell";
        # regression for the 2026-08-12 /bin/sh hole.
        ("shell_sh_provisioned", "sh -c 'command -v ros2'"),
        # Agent auth mounted readably (uid-mismatch regression; the check
        # command is agent knowledge; the adapter owns the auth layout) and
        # sandbox CLI == adapter pin (schema-drift vaccine, 2026-08-12
        # logout wave). Either is None for an adapter without that
        # arrangement, and a None check is simply not minted.
        *(c for c in (agent.credentials_check(), agent.sandbox_cli_check()) if c),
        # Polls until /clock is registered rather than asking once. The
        # timeout is the ceiling for every other probe here because they
        # block until their first message; `topic list | grep` returns
        # instantly, so a single ask spent none of the 60s and failed any
        # bringup slower than the moment precheck happened to run. LIBERO
        # at 14 concurrent trials red-checked this on ~100% of launches at
        # a uniform 38-47s while the same cell run alone passed (2026-08-26).
        # Now it waits like its siblings; a fast bringup still pays nothing.
        ("clock_topic",
         f"timeout {_CHECK_TIMEOUT_S} bash -c "
         "'until ros2 topic list | grep -qx /clock; do sleep 1; done'"),
        ("joint_states_flow",
         f"timeout {_CHECK_TIMEOUT_S} ros2 topic echo --once /joint_states"),
    ]
    # The manual promises frames; px2world needs the camera/world TF as
    # much as it needs depth + intrinsics, so the transform stream is
    # asserted, not assumed.
    if m.get("frames"):
        checks.append(("tf_flow", f"timeout {_CHECK_TIMEOUT_S} ros2 topic echo --once /tf"))
    # Per-arm checks over the normalized view (run.normalize_arms wrote
    # it into the assembly); a single-arm machine has one unlabeled arm
    # and mints exactly the historical check names.
    for arm in m.get("arms", []):
        sfx = f"_{arm['label']}" if arm.get("label") else ""
        aports = arm.get("ports", {})
        # The manual's joint names must be the ones actually flowing: a
        # joint_name_map slip would have the agent commanding panda_joint1
        # while the machine publishes robot0_joint1 (generated from the
        # same config as the manifest, so any promised arm gets its check).
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
        # NEGATIVE: a machine whose manual lists no gripper must not
        # expose one (wipe canary: manual/machine disagreement).
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
    # ... and the manifest's camera_set promises depth + intrinsics too;
    # px2world (the agent's pixel-to-world ruler) dies without them, so
    # the promise is asserted, not assumed (perception review 2026-08-15).
    checks.append(("camera_depth_intrinsics_flow",
                   "timeout 60 bash -c 'cam=$(ros2 topic list | grep -m1 "
                   "color/image_raw) && [ -n \"$cam\" ] && "
                   "base=${cam%/color/image_raw} && "
                   "ros2 topic echo --once \"$base/depth/image_raw\" "
                   ">/dev/null && ros2 topic echo --once "
                   "\"$base/color/camera_info\" >/dev/null'"))
    # Workspace tools the manual promises must actually run (ruling
    # 2026-08-14 Q3): import each tool module (all carry __main__ guards,
    # so import = deps + syntax smoke without side effects).
    if m.get("workspace_template"):
        checks.append(("workspace_tools_importable",
                       'timeout 60 python3 -c "import glob, importlib.util as u; '
                       "fs = glob.glob('/workspace/tools/*/*.py'); "
                       "assert fs, 'no tools in workspace'; "
                       "[(s := u.spec_from_file_location('tool_smoke', f))"
                       '.loader.exec_module(u.module_from_spec(s)) for f in fs]"'))
    if m.get("planning", {}).get("moveit"):
        ik = m["planning"].get("ik_service", "/compute_ik")
        # The service must ANSWER, not merely be listed (2026-08-12
        # GatherVegetables: manual promised MoveIt; agent found none).
        checks.append(("moveit_ik_answers",
                       f"timeout 45 ros2 service call '{ik}' "
                       f"moveit_msgs/srv/GetPositionIK '{{}}'"))
    return checks


def run_precheck(cfg: dict, sandbox_name: str, agent,
                 timeout_s: float | None = None) -> dict:
    """Execute all checks inside the sandbox; returns
    {ok, checks: [(name, passed)], failed: [names]}."""
    # The checks run sequentially in one bash script, so the wrapper budget
    # must cover the worst case of every per-check ceiling firing in a row;
    # a shorter budget would surface as a raw TimeoutExpired instead of a
    # readable red-check list. DERIVED from the per-check ceiling x count
    # (+ slack) rather than a hand-maintained constant, so raising the
    # ceiling can never silently under-budget the wrapper.
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
        failed = ["precheck_script_did_not_run"]
    # RETIRED 2026-09-02: host_cli_matches_pin. It enforced the host side
    # of sandbox == pin == host, and existed only because the host binary
    # and every sandbox shared one rotating credentials file: a host
    # auto-update rewrote the schema and the older sandbox CLI launched
    # logged-out (2026-08-12). Sandboxes now authenticate with their own
    # minted token and share no file with the host, so host version drift
    # cannot reach a trial. Keeping the check would only stop runs for a
    # coupling that no longer exists -- which is exactly what it did on
    # 2026-09-01, costing 138 trials when the host auto-updated to 2.1.257.
    # The sandbox-side check stays (build fact, in build_checks()).
    return {"ok": r.returncode == 0 and not failed,
            "checks": results, "failed": failed}
