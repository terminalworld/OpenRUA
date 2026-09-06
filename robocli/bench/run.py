"""The conductor: robocli-run lives here; every trial's when/where/who.

``robocli run --config benchmarks/<b>.yaml --run-id <label> --task-suite S
--task-ids 0,1 --seeds 0,1,2 --operator agent``

One file, layered by function: the machine remote control (boot the sim
container, talk to it over its private stdio line), the operators (who
acts on the robot during a trial), and the trial sequence (reset ->
machine up -> precheck -> operator -> verdict -> record ->
teardown). All composition lives here; the packages it composes are
single functions that never see each other. WHAT gets written under
``runs/`` is the examiner's knowledge (robocli.bench.record, the single
writer); this file only decides when and passes the paths.

Blind single-episode protocol (locked): success predicates belong to the
examiner side and are never exposed to the agent; reset and initial
placement are invisible on the agent's surface (this is what removes the
material basis of oracle-retry).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import select
import shutil
import subprocess
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

import yaml

from robocli import agents, config, paths
from robocli.bench import record
from robocli.bench import triallock
from robocli.bench.precheck import run_precheck
from robocli.proxy.up import ensure as ensure_proxy
# Host code touches the robot ONLY through its ground-side verbs;
# robocli.robot.onboard stays sealed (layering contract).
from robocli.robot.down import down as sim_down
from robocli.robot.up import up as sim_up
from robocli.sandbox import workspace
from robocli.sandbox.down import down as sandbox_down
from robocli.sandbox.up import up as sandbox_up

# ---------------------------------------------------------------- config view
# The conductor's ONE piece of domain knowledge: how a trial's view of
# the benchmark config is derived. Computed once per trial and
# distributed as data (the assembly.yaml artifact + rendered peers
# profiles); nobody else resolves configs (ruling 2026-08-17, absorbing
# the former assembly.py leaf -- its importers had collapsed to main
# after the compute-once refactor).

# ROS_STATIC_PEERS exists only from Iron on; Humble's Fast DDS ignores
# it. The portable equivalent is a Fast DDS initial-peers profile; DNS
# names resolve inside it, so container names work as-is. BOTH sides of
# a multicast-less network get a rendered copy (sim container and
# sandbox).
FASTDDS_PEERS_XML = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="robocli_initial_peers" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator><udpv4><address>{peer}</address></udpv4></locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
"""


def apply_suite_overrides(cfg: dict, task_suite: str) -> dict:
    """Deep-merge ``cfg["suite_overrides"][task_suite]`` into cfg, in place.

    ``None`` deletes a key; a suite whose robot mounts no gripper (wipe's
    0-DoF pad, 2026-08-11 canary) nulls ``machine.ports.gripper`` and
    ``machine.gripper`` so machine and manifest agree. Every consumer
    reads the RESULT (the trial's assembly.yaml artifact); only the
    conductor runs this.
    """
    def merge(dst: dict, src: dict) -> None:
        for k, v in src.items():
            if v is None:
                dst.pop(k, None)
            elif isinstance(v, dict) and isinstance(dst.get(k), dict):
                merge(dst[k], v)
            else:
                dst[k] = v

    ov = (cfg.get("suite_overrides") or {}).get(task_suite)
    if ov:
        merge(cfg, ov)
        # The override is authored as raw yaml; the merged view must still
        # fit the schema (a misspelled key in an override is the same
        # mistake as one in the profile).
        checked = config.dump(config.validate(
            config.Assembly, cfg, f"suite_overrides.{task_suite}"))
        cfg.clear()
        cfg.update(checked)
    return cfg


def normalize_arms(cfg: dict) -> dict:
    """Materialize ``machine.arms`` (the per-arm view), in place.

    Single-arm configs author the flat fields (``arm``/``ports``/
    ``gripper``/``frames``); this synthesizes ``arms[0]`` from them so
    every downstream unit (bridge ports, sensors, manifest, precheck)
    loops one uniform list. Multi-arm suites author ``machine.arms``
    explicitly in their suite override and the flat fields are ignored.
    Runs once here, lands in assembly.yaml; nobody else re-derives it.
    """
    m = cfg.get("machine")
    if not m or "arms" in m:
        return cfg
    ports = m.get("ports", {})
    m["arms"] = [{
        "label": "",
        "joints": m.get("arm", {}).get("joints", []),
        "limits_rad": m.get("arm", {}).get("limits_rad", []),
        "gripper": m.get("gripper"),
        "ports": {k: ports[k] for k in
                  ("trajectory", "gripper", "twist", "wrench") if ports.get(k)},
        "hand_body": "robot0_right_hand",
        "tf_base_body": m.get("tf", {}).get("base_body", "robot0_base"),
        "base_frame": m.get("frames", {}).get("base", "panda_link0"),
        "hand_frame": m.get("frames", {}).get("hand", "panda_hand"),
    }]
    return cfg


# One shared fallback when a config omits protocol.active_wall_clock_minutes
# (diff-review 2026-08-14 F-H: divergent 30/45 fallbacks were the same
# trap F18 removed for prompts). All real configs set the key explicitly.
DEFAULT_WALL_CLOCK_MIN = 30.0

# Renamed from protocol.wall_clock_minutes on 2026-08-20, when the budget
# stopped counting time a trial spends suspended at a quota wall. A config
# still carrying the old key is REFUSED rather than read as if nothing
# changed: silently accepting it would run the new active-time semantics
# under a name that promised total time.
LEGACY_WALL_CLOCK_KEY = "wall_clock_minutes"


def resolve_wall_clock_min(cfg: dict) -> float:
    """The trial's ACTIVE wall-clock budget in minutes, from the config."""
    protocol = cfg.get("protocol", {})
    if LEGACY_WALL_CLOCK_KEY in protocol:
        raise ValueError(
            f"config uses protocol.{LEGACY_WALL_CLOCK_KEY}, which was renamed "
            "to protocol.active_wall_clock_minutes on 2026-08-20 (the budget "
            "no longer counts time suspended at a quota wall). Fix it with:\n"
            f"  sed -i 's/{LEGACY_WALL_CLOCK_KEY}:/active_wall_clock_minutes:/' "
            "<config.yaml>")
    return float(protocol.get("active_wall_clock_minutes",
                              DEFAULT_WALL_CLOCK_MIN))

# Run data lands under the caller's working directory by default
# (--runs-root overrides); the package never writes into itself.
RUNS_ROOT = Path("runs")


def load_robot(robot: str, home: Path | None = None) -> dict:
    """A robot profile by name (bundled, then ``<home>/robots/``) or by
    path, validated (config.RobotProfile). Returns it as a dict: its
    ``machine:`` section is the robot, ``world:`` the default scene."""
    p = paths.find("robots", robot, home)
    return config.dump(config.validate(config.RobotProfile, config.load_yaml(p), p))


def load_config(path: Path | str, robot: str | None = None,
                home: Path | None = None) -> dict:
    """A benchmark config (name or path), validated and assembled:
    ``robot: <name>`` in the file (or the argument, which wins; or the
    user's default) pulls in that profile's ``machine:`` section; a file
    carrying its own ``machine:`` is taken as is. The user's
    ``~/.robocli/config.yaml`` supplies agent defaults under the file's
    own. Returns the assembled dict (config.Assembly, validated)."""
    p = paths.find("benchmarks", path, home)
    bench = config.validate(config.Benchmark, config.load_yaml(p), p)
    user = config.load_user_config(paths.config_path(home))
    cfg = config.dump(bench)
    robot = robot or cfg.pop("robot", None) or user.robot
    if robot:
        cfg["machine"] = load_robot(robot, home)["machine"]
    if "machine" not in cfg:
        raise config.ConfigError(
            f"{p}: names no robot (robot: <name>, --robot, or robot: in "
            f"{paths.config_path(home)}) and carries no machine: section")
    cfg["agent"] = config.layer_agent(user, cfg.get("agent", {}))
    return config.dump(config.validate(config.Assembly, cfg, p))


def substrate_venv(body: dict, home: Path | None = None) -> Path:
    """The simulator venv named by machine.body.substrate.venv (see
    paths.substrate_venv for how relative names resolve)."""
    return paths.substrate_venv(body["substrate"]["venv"], home)


def resolve_body_files(cfg: dict, dest: Path, home: Path | None = None) -> None:
    """Files the body reads by path (today: ``machine.controller_config``)
    are copied next to the assembly and named there by absolute path, so
    the container sees them through the one mount it has on that
    directory and the assembly stays self-contained. Names resolve
    bundled (``robocli/robots/``), then ``<home>/robots/``, then as a
    path."""
    machine = cfg.get("machine", {})
    spec = machine.get("controller_config")
    if not spec:
        return
    p = Path(spec).expanduser()
    candidates = [p] if p.is_absolute() else [
        paths.bundled("robots") / p, paths.user_dir("robots", home) / p, p]
    src = next((c for c in candidates if c.is_file()), None)
    if src is None:
        looked = ", ".join(str(c) for c in candidates)
        raise FileNotFoundError(
            f"machine.controller_config {spec!r} not found (looked at: {looked})")
    dst = dest / src.name
    shutil.copyfile(src, dst)
    machine["controller_config"] = str(dst.resolve())


# =====================================================================
# The robot's phone: the conductor-held end of the private stdio line.
# The body is built by robot.up (which hands over the pipes);
# from then on the ONLY ongoing relationship is conversation here --
# rpc questions on stdin/stdout, stderr already streaming to bridge.log.
# Conductor dies -> pipe EOF -> the robot powers itself off (no
# orphans). When the conversation itself is dead, the e-stop
# (robot.down) cuts power from outside.
# =====================================================================

class MachineClient:
    def __init__(self, proc, name: str):
        self.name = name
        self._proc = proc
        # One question, one answer, one thread at a time (audit 2026-08-14
        # F5): the watchdog probe thread and the main thread share this
        # pipe; unlocked, a stale probe could steal the final success
        # verdict and hand the main thread a pre-latch answer.
        self._rpc_lock = threading.Lock()
        self._rx = b""  # raw-fd line buffer (see _read_line)

    # ------------------------------------------------------------------- rpc
    def _read_line(self, deadline: float, note: str) -> str:
        """One channel line via raw-fd reads (never TextIO readline: its
        buffer would hide bytes from select and fake a timeout)."""
        fd = self._proc.stdout.fileno()
        while b"\n" not in self._rx:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise RuntimeError(f"machine rpc timeout {note}")
            ready, _, _ = select.select([fd], [], [], remaining)
            if not ready:
                raise RuntimeError(f"machine rpc timeout {note}")
            import os as _os
            chunk = _os.read(fd, 65536)
            if not chunk:
                raise RuntimeError(f"machine died mid-rpc {note}")
            self._rx += chunk
        line, _, self._rx = self._rx.partition(b"\n")
        return line.decode()

    def rpc(self, obj: dict, timeout_note: str = "",
            timeout_s: float = 900.0) -> dict:
        """One request/response, serialized, bounded, id-matched.

        The lock makes write+read atomic across threads (audit 2026-08-14
        F5); the timeout surfaces a wedged sim thread as an exception
        (F16); the request id lets a later call DISCARD the late answer
        of an earlier abandoned (timed-out) request instead of taking it
        for its own reply (diff-review 2026-08-14 F-A; both watchdog
        probes and the verdict ask "success", so shape alone can't tell
        them apart).
        """
        assert self._proc.stdin and self._proc.stdout
        rid = uuid.uuid4().hex[:12]
        deadline = time.time() + timeout_s
        note = f"{obj.get('cmd')} {timeout_note}"
        with self._rpc_lock:
            self._proc.stdin.write(json.dumps({**obj, "id": rid}) + "\n")
            self._proc.stdin.flush()
            while True:
                resp = json.loads(self._read_line(deadline, note))
                if resp.get("id") in (rid, None):  # None: pre-id machine
                    return resp
                import sys as _sys
                print(f"[rpc] drained stale answer (id {resp.get('id')}) "
                      f"while waiting for {note}", file=_sys.stderr, flush=True)

    def wait_ready(self, timeout_s: float = 1800.0) -> None:
        """The control channel answers once the WHOLE robot is up (env,
        graph, and -- since the robot boots complete -- MoveIt).

        The bound is deliberately generous (diff-review 2026-08-14 F-B:
        the pre-timeout code effectively waited unboundedly for boot;
        concurrent llvmpipe whole-room builds legitimately take many
        minutes); it exists to convert a truly dead boot into a clean
        anomaly, not to police boot speed.
        """
        try:
            if self.rpc({"cmd": "success"}, "boot", timeout_s=timeout_s).get("ok"):
                return
        except (RuntimeError, json.JSONDecodeError) as e:
            raise TimeoutError(f"machine not ready: {e}") from e
        raise TimeoutError("machine not ready: control channel answered not-ok")

    # -------------------------------------------------------------- teardown
    def shutdown(self) -> None:
        try:
            # Short leash (diff-review 2026-08-14 F-G): a wedged machine
            # must fall through to the e-stop in seconds, not squat on
            # the default rpc timeout per teardown.
            self.rpc({"cmd": "shutdown"}, timeout_s=15.0)
            self._proc.wait(timeout=30)
        except Exception:  # noqa: BLE001
            sim_down(self.name)


def ensure_internal_network(name: str = "robocli-internal") -> str:
    subprocess.run(
        ["docker", "network", "create", "--internal", name], capture_output=True
    )
    return name


# =====================================================================
# Operators: who acts on the robot during a trial.
#
# - ``none``: nobody acts (plumbing tests; trials score false).
# - ``script``: a caller-supplied command-sequence file runs inside the
#   sandbox on the native surface (same shell the agent gets; zero
#   privileged verbs). Canonical source: a trial's commands.sh
#   condensate. Open-loop best-effort replay; signals-only.
# - ``agent``: the real operator (headless agent) via robocli.agents.
# =====================================================================

def none_operator(ctx: dict) -> dict:
    return {"operator": "none"}


def agent_operator(ctx: dict) -> dict:
    """The real operator: a headless off-the-shelf agent (P5+).

    Launched as a subprocess of the agents launcher (never imported;
    architecture contract). Self-finish vs max-turns is read back from the
    transcript's final result record.

    The trial runs as one or more SEGMENTS: hitting the account's quota
    wall suspends it (body and sim left standing, clock paused) and the
    same session resumes when the window reopens. Both the wall-clock and
    the turn budget are spent across segments, and the wall clock counts
    active time only.
    """
    import os
    import sys

    cfg = ctx["cfg"]
    agent_cfg = cfg.get("agent", {})
    agent = agents.get(agent_cfg.get("cli"), ctx.get("home"))
    model = agent_cfg.get("model") or agent.default_model
    # Adapter knobs (reasoning effort, compaction threshold, tool
    # timeouts, whatever the CLI exposes) are load-bearing experimental parameters that do not
    # appear in the transcript: the merged set is pinned here from the
    # adapter's defaults and the config, passed to the launcher explicitly,
    # and recorded in the trial's meta (2026-08-19, 2026-08-20).
    options = {**agent.default_options, **agent_cfg.get("options", {})}
    # Naming the session up front is what makes a suspended trial resumable
    # without scraping an id back out of a half-written transcript.
    session_id = str(uuid.uuid4())
    trial_dir: Path = ctx["trial_dir"]
    transcript = trial_dir / "transcript.jsonl"
    max_turns = int(cfg.get("protocol", {}).get("max_turns", 100))
    base = [
        sys.executable, "-m", "robocli.agents.launcher",
        "--sandbox", ctx["sandbox"],
        "--task", ctx["task_language"],
        "--transcript", str(transcript),
        "--cli", agent.name,
        "--model", model,
        *(x for k, v in options.items() for x in ("--option", f"{k}={v}")),
        *(["--home", str(ctx["home"])] if ctx.get("home") else []),
        "--session-id", session_id,
        "--proxy", ctx.get("proxy", "http://robocli-proxy:8888"),
        *(["--token-file", str(ctx["token_file"])] if ctx.get("token_file")
          else []),
    ]
    meta: dict = {"operator": "agent", "cli": agent.name, "model": model,
                  "options": options, "session_id": session_id}
    # The launcher runs with cwd=trial_dir: RELATIVE PYTHONPATH entries
    # (a natural way to invoke the master) would silently break `-m
    # robocli.agents.launcher` there (2026-08-14 rehearsal: 6/6 trials
    # launcher_failed). Absolutize against the conductor's own cwd.
    env = dict(os.environ)
    if env.get("PYTHONPATH"):
        env["PYTHONPATH"] = os.pathsep.join(
            str(Path(p).resolve()) for p in env["PYTHONPATH"].split(os.pathsep) if p)

    # ---- the trial as a sequence of segments (ruling 2026-08-20) --------
    # OFF BY DEFAULT, and deliberately so: a quota wall is an artifact of
    # running at scale on subscription accounts, not part of the method.
    # Someone reproducing a single trial through an API key never meets one,
    # and should not inherit our workaround. robocli therefore ships the
    # capability; the config decides whether to use it, and the trial
    # records which way it ran.
    #
    # A trial that hits the account's quota wall is SUSPENDED, not thrown
    # away: the sandbox and the simulated robot stay up (the clock is
    # paused, so a wait of hours is physically a no-op), and when the
    # window reopens the same session is resumed. Waiting happens here,
    # inside the trial, rather than in the batch master: the containers
    # then never outlive the process that owns them, and the master keeps
    # seeing one process per trial.
    #
    # Both budgets are spent across segments, never per segment:
    #   - wall clock counts ACTIVE seconds only (suspension is not the
    #     agent's time), which is why the config key says so;
    #   - the turn budget must be carried too, because the CLI restarts it
    #     on every --resume (measured 2026-08-20). Turns already spent come
    #     from the transcript's own count, which runs slightly AHEAD of the
    #     budget scale (a budget of 3 reports 4), so a resumed trial ends up
    #     with marginally FEWER turns than an uninterrupted one -- the safe
    #     direction for comparability.
    # Waiting out a wall is bounded on both axes. A five-hour window is the
    # ordinary case; a weekly wall would otherwise park a lane for days, and
    # a wall that keeps reappearing would park it forever. Past either bound
    # the trial gives up as quota-limited, exactly as it did before resume
    # existed, and the master requeues it (onto another account if there is
    # one). Both are explicit parameters, not constants buried here.
    protocol = cfg.get("protocol", {})
    resume_on_wall = bool(protocol.get("resume_on_quota_wall", False))
    meta["resume_on_quota_wall"] = resume_on_wall
    max_wait_s = float(protocol.get("max_quota_wait_minutes", 360)) * 60
    max_suspensions = int(protocol.get("max_suspensions", 4))
    budget_s = ctx["active_wall_clock_min"] * 60
    active_s = 0.0
    suspended_s = 0.0
    turns_used = 0
    segments: list[dict] = []
    resumed = False
    last_quota: dict = {}
    while True:
        remaining_s = budget_s - active_s
        remaining_turns = max_turns - turns_used
        if remaining_s <= 0:
            meta["termination"] = "wall_clock_cap"
            break
        if remaining_turns < 1:
            meta["termination"] = "max_turns"
            break
        cmd = base + ["--max-turns", str(remaining_turns)]
        if resumed:
            cmd.append("--resume")
        # Where this segment starts in the transcript. The runner owns this
        # boundary rather than letting the reader infer one from record
        # shapes: a correctness-critical decision should not depend on the
        # CLI continuing to emit a particular marker on every resume.
        # The first segment starts at 0 because the launcher TRUNCATES for
        # it; trial directories are reused across retries, so reading the
        # length here would carry a dead attempt's line count into a fresh
        # file and scan past its end -- missing a real wall entirely.
        mark = _line_count(transcript) if resumed else 0
        t0 = time.time()
        try:
            proc = subprocess.run(
                cmd, cwd=trial_dir, timeout=remaining_s, env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            meta["termination"] = "self_finished"
            meta["launcher_returncode"] = proc.returncode
            timed_out = False
        except subprocess.TimeoutExpired:
            meta["termination"] = "wall_clock_cap"
            timed_out = True
        t1 = time.time()
        spent = t1 - t0
        active_s += spent
        # Absolute bounds, not just a duration: post-hoc budget judging asks
        # how much ACTIVE time had passed when a success latched, and only
        # per-segment timestamps can answer that for a suspended trial.
        segments.append({"started_unix": round(t0, 1),
                         "ended_unix": round(t1, 1),
                         "active_s": round(spent, 1),
                         "turn_budget": remaining_turns,
                         "resumed": resumed})
        fin_so_far = agent.read_final(transcript)
        if fin_so_far.get("num_turns") is not None:
            turns_used = fin_so_far["num_turns"]
        if timed_out:
            break
        # A quota rejection is the ONE reason to wait rather than finish.
        # Anything else -- success, max turns, a dead CLI -- ends the trial.
        # Read THIS segment only: the audit's whole-file verdict is sticky
        # by design, and a sticky verdict here would re-suspend a trial that
        # already finished, until it ran out of suspensions and the finished
        # work was thrown away.
        last_quota = agent.quota_since(transcript, mark) or {}
        resets_at = last_quota.get("resets_at")
        if not resets_at:
            break
        if not resume_on_wall:
            # Switched off: end here exactly as this ran before suspension
            # existed. The wall is in the transcript and unresolved, so the
            # audit voids the attempt and the master requeues it.
            break
        # BOTH bodies must still be standing: the sandbox holds the session
        # to resume, the sim holds the world it was working on. Resuming
        # into a half-present world would produce a trial that looks
        # complete and measured nothing. (The stall watchdog can itself kill
        # the sandbox when the sim wedges, so this also catches that.)
        missing = [n for n in (ctx["sandbox"], ctx.get("sim"))
                   if n and not _container_running(n)]
        if missing:
            meta["termination"] = "sandbox_lost"
            meta["lost_containers"] = missing
            break
        try:
            wait_s = max(0.0, float(resets_at) - time.time())
        except (TypeError, ValueError):
            # A reset time we cannot read is a reset time we cannot wait
            # for. Stop here rather than killing the trial: the wall is
            # still in the transcript and still unresolved, so the audit
            # voids the attempt and the master requeues it -- the old path.
            meta["quota_gave_up_on"] = "unreadable-reset-time"
            break
        if wait_s > max_wait_s or len(segments) > max_suspensions:
            why = ("wait" if wait_s > max_wait_s else "suspensions")
            meta["termination"] = "quota_limit"
            meta["quota_gave_up_on"] = why
            print(f"[trial] quota wall beyond the {why} bound; giving up "
                  "for the master to requeue", flush=True)
            break
        print(f"[trial] quota wall; suspending {round(wait_s / 60)}min "
              f"until the window reopens", flush=True)
        time.sleep(wait_s)
        suspended_s += wait_s
        resumed = True

    meta["segments"] = len(segments)
    meta["resumes"] = max(0, len(segments) - 1)
    # Whether a quota wall this trial hit was WAITED OUT rather than fatal.
    # The audit voids a trial whose transcript carries a quota rejection
    # (ruling 2026-08-14 Q1) because a trial cut off mid-task is not a fair
    # measurement -- but a resumed trial was not cut off: it kept its
    # remaining budgets and ran on. This flag is what tells the two apart,
    # and it is the runner's to set: only the runner knows whether the wait
    # actually happened (ruling 2026-08-20).
    # Only a trial whose LAST segment came back clean counts as resolved.
    # A wall can also arrive with no reset time (the 2026-08-09 weekly shape
    # carries only an error phrase): the loop cannot wait for a time it was
    # not given, so it stops -- and that trial ended AT a wall, however it
    # is labelled. Requiring the last segment to be clean keeps every
    # doubtful case on the old path, where the master requeues it.
    meta["quota_resolved"] = bool(
        meta["resumes"] and not last_quota and meta.get("termination") not in
        ("quota_limit", "sandbox_lost", "launcher_failed"))
    meta["active_seconds"] = round(active_s, 1)
    meta["suspended_seconds"] = round(suspended_s, 1)
    meta["segment_detail"] = segments
    # Never-boarded detection, BOTH paths (audit 2026-08-14 F6 +
    # diff-review F-C): the launcher opens the transcript file before the
    # CLI runs, so a CLI that hung producing nothing (bad credentials,
    # dead proxy) leaves an EMPTY file; even on the timeout path, where
    # it would otherwise burn the full cap and enter the denominator as a
    # fake capability failure. Nonzero exit alone is NOT the signal (the
    # CLI exits nonzero on legitimate max-turns too).
    if not transcript.exists() or not transcript.read_text().strip():
        meta["termination"] = "launcher_failed"
    # Turn count / max-turns verdict from the CLI's final record, via the
    # adapter (the transcript format is agent knowledge; across segments it
    # reports the trial's totals).
    # NOTE: no quota/validity judgment here; the conductor records what
    # happened; deciding whether the attempt counts (quota-voided,
    # corrupt, ...) is orchestration/audit/audit.py reading the artifacts.
    fin = agent.read_final(transcript)
    if fin:
        meta["num_turns"] = fin.get("num_turns")
        for k in ("usage", "cost_usd", "duration_ms"):
            if fin.get(k) is not None:
                meta[k] = fin[k]
        if fin.get("hit_max_turns"):
            meta["termination"] = "max_turns"
    return meta


def _line_count(path: Path) -> int:
    """Lines a file has right now, 0 if it is not there yet."""
    try:
        return len(path.read_text(errors="replace").splitlines())
    except OSError:
        return 0


def _container_running(name: str) -> bool:
    """Whether a container is still up. Consulted before waiting out a
    quota wall: resuming needs the bodies that were left mid-task."""
    try:
        out = subprocess.run(
            ["docker", "inspect", "-f", "{{.State.Running}}", name],
            capture_output=True, text=True, timeout=30)
    except (subprocess.TimeoutExpired, OSError):
        return False
    return out.stdout.strip() == "true"


def script_operator(ctx: dict) -> dict:
    """Run a caller-supplied command sequence in the sandbox.

    Surface-only by construction: the file is copied into the sandbox
    and executed by bash as the seat user with ROS sourced, exactly the
    shell the agent gets; no control-channel verb is touched. Wall-clock
    cap enforced here like the agent's. The world answers or not; the
    verdict comes from the ordinary blind ask afterwards."""
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


# =====================================================================
# Trial sequence: reset -> machine up -> precheck -> operator ->
# verdict -> record -> teardown. Owns runs/<benchmark>/<run-id>/ layout;
# content semantics live in robocli.bench.record (paths passed in).
# =====================================================================

def run_trial(cfg, cfg_path, run_dir, task_suite, task_id, seed, operator,
              wall_cap_min, ros_domain=0, credentials_dir=None, home=None,
              account_alias=None, script=None, token_file=None):
    trial_dir = run_dir / "trials" / f"{task_suite}-{task_id}" / f"seed{seed}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    # Container names double as DNS labels for DDS peer resolution; a
    # label over the DNS limit silently fails to resolve and the machine
    # dies at node creation (batch-1 incident: run-id + suite repeated =
    # 70 chars, 60/60 trials down). Budget is DERIVED from the actual
    # suffixes so renaming one can never silently reopen the hole.
    # Attempt-unique suffix: reruns of the same trial (and any stale
    # leftovers from a crashed attempt) must never collide on names;
    # names identify ATTEMPTS, not trial keys (2026-08-10 orphan incident).
    # Computed before the directory is claimed, because the claim records
    # this exact stem: after truncation it is the only string that finds
    # this attempt's containers.
    DNS_LABEL_MAX = 63  # RFC 1035, single label
    suffixes = ("-sim", "-sandbox")
    attempt_tag = uuid.uuid4().hex[:6]
    stem = f"rc-{run_dir.name}-{task_id}-{seed}"
    budget = (DNS_LABEL_MAX - len(f"-{attempt_tag}")
              - max(len(s) for s in suffixes))
    if len(stem) > budget:
        digest = hashlib.sha256(stem.encode()).hexdigest()[:8]
        stem = f"{stem[:budget - len(digest) - 1]}-{digest}"
    stem = f"{stem}-{attempt_tag}"
    # Claim the directory BEFORE touching it: the archive step below moves
    # every existing artifact aside, so a second writer's opening act would
    # pull a running attempt's transcript out from under it.
    lock = triallock.acquire(trial_dir, stem)
    if lock is None:
        info = triallock.holder(trial_dir) or {}
        raise triallock.TrialLocked(
            f"{trial_dir}: a live attempt already owns this trial "
            f"(pid {info.get('pid')} on {info.get('host')}); refusing to "
            "write alongside it")
    record.archive_prior_attempt(trial_dir)
    sim_name, sandbox_name = f"{stem}-sim", f"{stem}-sandbox"
    assert all(len(stem + s) <= DNS_LABEL_MAX for s in suffixes)
    network = ensure_internal_network()

    t0 = time.time()
    rec: dict = {
        "task_suite": task_suite,
        "task_id": task_id,
        "init_state_id": seed,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        # Which workspace the occupant actually got. config.json carries
        # this too, but the runner rewrites config.json every trial, so
        # after a mid-campaign template edit only the trial-level stamp
        # can still tell the two halves apart (2026-08-21: the gripper
        # manifest edit landed mid-run and had nothing to be audited by).
        "workspace_template_sha256": workspace.template_hash(cfg),
        # Names identify attempts (uuid tag): the handle that joins this
        # record to docker events/logs when a body dies out-of-band.
        "containers": {"sim": sim_name, "sandbox": sandbox_name},
        "ros_domain": ros_domain,
    }
    proxy_url = ensure_proxy(network)
    if account_alias:
        rec["account_alias"] = account_alias
    agent = agents.get(cfg.get("agent", {}).get("cli"), home)
    creds_home = Path(
        credentials_dir
        or cfg.get("agent", {}).get("credentials_dir")
        or paths.credentials_dir(home) / agent.name
    ).expanduser()
    # A token file is the sandbox's whole auth story, so the login profile
    # is no longer required to carry credentials (see agents.prepare_profile).
    cfg_dir, creds_file = agents.prepare_profile(
        creds_home, agent, require_credentials=token_file is None)
    secrets = record.secret_strings(creds_home)  # pre-trial token values
    if token_file:
        # The minted token never reaches the record: it is as much a secret
        # as anything in the credentials file, and the agent can print its
        # own environment.
        secrets += record.secret_strings_from_env_file(Path(token_file))
    # Container construction INSIDE the try (audit 2026-08-14 F15): a
    # machine-constructor exception after the sandbox is up must still
    # write result.json (anomaly) and tear the sandbox down, not orphan it
    # and kill the rest of the invocation's queue.
    sandbox_live = False
    machine = None
    try:
        # Sandbox first: the sim's ROS_STATIC_PEERS must resolve the sandbox's
        # name at participant creation (mutual unicast discovery, P4).
        # The house package seeds the workspace and births the container;
        # the conductor only decides when/where and passes the wiring.
        # The trial's resolved suite view: computed once (main() already
        # applied the overrides), written as an artifact, and consumed by
        # every party below -- the manual, the body, the machine itself
        # all read the SAME file (ruling 2026-08-16; kills the
        # same-code-different-arguments drift class). Peers profiles are
        # rendered here too; the two ups just paste them.
        resolve_body_files(cfg, trial_dir, home)
        assembly_path = record.write_assembly(trial_dir, cfg)
        sandbox_up(
            config=assembly_path,
            workspace=trial_dir / "workspace",
            network=network,
            static_peer=sim_name,
            peers_xml=FASTDDS_PEERS_XML.format(peer=sim_name),
            ros_domain=ros_domain,
            internet=f"proxy:{proxy_url}",
            name=sandbox_name,
            mounts=agent.sandbox_mounts(cfg_dir, creds_file),
        )
        sandbox_live = True
        body = cfg.get("machine", {}).get("body", {})
        venv = substrate_venv(body, home)
        proc = sim_up(
            name=sim_name,
            gpus=bool(body.get("gpus", False)),
            resources=body.get("resources"),
            image=body.get("image", "robocli-sim-jazzy"),
            config_path=str(assembly_path),
            task_suite=task_suite,
            task_id=task_id,
            substrate=str(paths.substrate_root(venv)),
            venv=str(venv),
            code_root=str(paths.code_root()),
            log_path=trial_dir / "bridge.log",
            moveit_log=str(trial_dir / "moveit.log"),
            network=network,
            static_peer=sandbox_name,
            peers_xml=FASTDDS_PEERS_XML.format(peer=sandbox_name),
            ros_domain=ros_domain,
        )
        machine = MachineClient(proc, sim_name)
        machine.wait_ready()
        r = machine.rpc({"cmd": "reset", "init_state_id": seed})
        if not r.get("ok"):
            raise RuntimeError(f"reset failed: {r}")
        info = machine.rpc({"cmd": "task_info"})
        task_language = info.get("language", "")
        rec["task_language"] = task_language
        # Optional generic slot: whatever the loader considers the facts
        # that identify THIS episode's world (robocasa fills the kitchen
        # it ran in). Recorded verbatim; the conductor does not interpret
        # it, and a loader that fills nothing costs nothing.
        if info.get("init_state"):
            rec["init_state"] = info["init_state"]
        if not task_language.strip():
            # An empty mission is never a valid trial (2026-08-11 canary:
            # a task_info bug fed agents "" and they surveyed for 30 min).
            raise RuntimeError("task_info returned empty language")

        # Machine-manual precheck (A6): every verifiable manual promise
        # asserted from the sandbox's vantage BEFORE the agent boards.
        # Red -> refuse trial -> anomaly -> classify INFRA rerun; the
        # agent never pays for a lying manual. Examiner's knowledge;
        # the adapter is handed over (precheck looks nothing up).
        conf = run_precheck(cfg, sandbox_name, agent)
        rec["precheck"] = conf["checks"]
        if not conf["ok"]:
            raise RuntimeError(
                f"precheck failed: {','.join(conf['failed'])}")

        # Sim-stall watchdog (2026-08-12 composite canaries: two trials
        # spent 20+ min against a wedged physics thread, recorded as agent
        # failures with anomaly=null). The "success" verb runs THROUGH the
        # sim thread; if it stops answering, the physics owner is wedged:
        # kill the sandbox (frees the agent's quota burn), flag the trial as
        # an anomaly so classify sends it back for a rerun.
        #
        # Timeout sizing: a single trajectory goal executes as ONE sim-
        # thread job and can legitimately run minutes of wall time (20s of
        # sim = ~400 ticks x ~0.2s under llvmpipe whole-room rendering);
        # a short timeout would KILL healthy trials, which poisons data;
        # a long one merely detects wedges later. 10min clears any
        # plausible single job; a real wedge still surfaces ~12min in
        # (vs the 20-25min agents burned probing dead sims unaided).
        stall: dict = {}
        wd_stop = threading.Event()
        probe_timeout = float(cfg.get("protocol", {})
                              .get("stall_probe_timeout_s", 600.0))

        def _stall_watchdog():
            while not wd_stop.wait(120.0):
                probe: dict = {}

                def _probe():
                    try:  # rpc timeout == probe timeout: the thread ends
                        # (and releases the rpc lock) the moment the
                        # verdict is due, instead of squatting on the pipe
                        probe["r"] = machine.rpc({"cmd": "success"},
                                                 "watchdog",
                                                 timeout_s=probe_timeout)
                    except Exception:  # noqa: BLE001; timeout = no answer
                        pass

                t = threading.Thread(target=_probe, daemon=True)
                t.start()
                t.join(probe_timeout + 10.0)
                if t.is_alive() or "r" not in probe:
                    stall["msg"] = ("sim stall: control channel "
                                    f"unresponsive >{probe_timeout:.0f}s "
                                    "(watchdog)")
                    subprocess.run(["docker", "kill", sandbox_name],
                                   capture_output=True)
                    return

        wd = threading.Thread(target=_stall_watchdog, daemon=True)
        wd.start()
        try:
            op_meta = operator(
                {
                    "machine": machine,
                    "trial_dir": trial_dir,
                    "cfg": cfg,
                    "sandbox": sandbox_name,
                    "sim": sim_name,
                    "task_language": task_language,
                    "active_wall_clock_min": wall_cap_min,
                    "proxy": proxy_url,
                    "script": script,
                    "token_file": token_file,
                    "home": home,
                }
            )
        finally:
            wd_stop.set()
        if stall:
            raise RuntimeError(stall["msg"])

        # Blind single episode: the examiner asks once, after the operator
        # is done. The machine latches per step (official any-step-success
        # semantics); the end-state verdict rides along for our own books.
        verdict = machine.rpc({"cmd": "success"})
        rec["success"] = bool(verdict["success"])
        if "success_end_state" in verdict:
            rec["success_end_state"] = bool(verdict["success_end_state"])
        if "success_at" in verdict:  # time-to-success (post-hoc budgets)
            rec["success_at"] = verdict["success_at"]
        # Closing step snapshot (ruling 2026-08-14 Q2): total sim steps so
        # step-economy stats exist for FAILED trials too. Individually
        # guarded (diff-review F-D): auxiliary telemetry must never
        # retract an already-obtained verdict.
        try:
            rec["steps_total"] = machine.rpc(
                {"cmd": "steps"}, timeout_s=60.0).get("steps")
        except Exception:  # noqa: BLE001
            rec["steps_total"] = None
        rec["steps_semantics"] = "episode"  # post-F7 counter scope
        # (diff-review F-E: pre-2026-08-14 records counted process-lifetime
        # steps; trim.py only step-judges records that carry this marker)
        rec["operator_meta"] = op_meta
        rec["termination"] = op_meta.get("termination", "operator_done")
        # Validity judgment (quota voiding etc.) belongs to orchestration
        # (classify.py, from artifacts); the conductor records faithfully
        # and flags only its OWN failures below.
        rec["anomaly"] = None
    except Exception as exc:  # noqa: BLE001; anomaly ledger, not a crash
        rec["success"] = False
        rec["termination"] = "anomaly"
        rec["anomaly"] = f"{type(exc).__name__}: {exc}"
        # The one-liner names the failure; the stack finds it. Host-side
        # errors have no bridge.log to fall back on.
        import traceback
        rec["anomaly_traceback"] = traceback.format_exc()
    finally:
        if machine is not None:
            machine.shutdown()
        if sandbox_live:
            sandbox_down(sandbox_name)
        # Post-trial token values too; a mid-trial rotation would leave
        # BOTH generations potentially visible in the record.
        secrets += record.secret_strings(creds_home)
        shutil.rmtree(cfg_dir, ignore_errors=True)
        record.finalize_trial(trial_dir, agent, secrets)
        lock.release()
    # wall_seconds spans the whole harness (boot + gate + agent +
    # teardown). The wall cap is enforced ONLY on the agent subprocess
    # (agent_operator timeout) and its verdict already sits in
    # rec["termination"]; never overwrite it from the harness span
    # (audit 2026-08-14 F3: the old unconditional rewrite relabeled
    # near-cap self-finishes and anomalies as wall_clock_cap; the
    # PreSoakPan self-contradiction).
    rec["wall_seconds"] = round(time.time() - t0, 1)
    # Named for the semantics, so a result file says which budget it ran
    # under without anyone dating it against a commit (ruling 2026-08-20).
    rec["active_wall_cap_min"] = wall_cap_min
    record.write_result(trial_dir, rec)
    return rec


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True,
                    help="benchmark config (benchmarks/<name>.yaml)")
    ap.add_argument("--robot", default=None,
                    help="robot profile name or path; overrides the "
                    "config's robot: line")
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--task-suite", required=True)
    ap.add_argument("--task-ids", default="0")
    ap.add_argument("--seeds", default="0")
    ap.add_argument("--operator", default="none", choices=sorted(OPERATORS))
    ap.add_argument(
        "--script", default=None,
        help="command-sequence file for --operator script (canonical "
        "source: a trial's commands.sh condensate); runs in the sandbox "
        "on the native surface, open-loop best-effort",
    )
    ap.add_argument(
        "--wall-clock-min", type=float, default=None,
        help="override of the config's protocol.active_wall_clock_minutes "
        "(audit 2026-08-14 F4: the config is the default's single source; "
        "a forgotten flag must not silently shrink the protocol cap)",
    )
    ap.add_argument(
        "--ros-domain", type=int, default=0,
        help="ROS_DOMAIN_ID for this run's containers; concurrent runs "
        "MUST use distinct domains (same DDS network would cross-talk)",
    )
    ap.add_argument(
        "--credentials-dir", default=None,
        help="agent login-profile override (account pool sets this; "
        "default falls back to cfg agent.credentials_dir, then the "
        "adapter's default)",
    )
    ap.add_argument(
        "--token-file", default=None,
        help="file holding <token_env>=<token> for the sandbox CLI (the "
        "account pool sets this); given, the sandbox authenticates with "
        "that token and no credentials file is mounted",
    )
    ap.add_argument(
        "--runs-root", default=None,
        help="where run data lands (default: ./runs)",
    )
    ap.add_argument(
        "--home", default=None,
        help="the user directory (default: ~/.robocli); robot and benchmark "
        "names, substrates and login profiles are looked up under it",
    )
    ap.add_argument(
        "--account-alias", default=None,
        help="non-secret label of the credentials profile, recorded in the "
        "trial result for per-account accounting",
    )
    args = ap.parse_args()
    if args.token_file:
        # See --token-file: the launcher runs with cwd=trial_dir, so a
        # relative path would resolve to nothing by the time docker reads it.
        args.token_file = str(Path(args.token_file).expanduser().resolve())
    args.task_ids = [int(x) for x in str(args.task_ids).split(",")]
    args.seeds = [int(x) for x in str(args.seeds).split(",")]

    home = paths.home(args.home)
    cfg_path = paths.find("benchmarks", args.config, home).resolve()
    cfg = load_config(cfg_path, args.robot, home)
    if args.wall_clock_min is None:
        args.wall_clock_min = resolve_wall_clock_min(cfg)
    # The per-suite view, computed exactly once (see config-view section
    # above); everyone downstream consumes the resulting artifact.
    apply_suite_overrides(cfg, args.task_suite)
    normalize_arms(cfg)
    runs_root = Path(args.runs_root).expanduser() if args.runs_root else RUNS_ROOT
    run_dir = runs_root.resolve() / cfg["task"]["benchmark"] / args.run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    agent = agents.get(cfg.get("agent", {}).get("cli"), home)
    prov = {**record.provenance(
        cfg_path, cfg, args, agent=agent,
        template_hash=workspace.template_hash(cfg),
        prompt=agents.PROMPT, resume_prompt=agents.RESUME_PROMPT,
        code_root=paths.code_root(),
        substrate_venv=substrate_venv(cfg["machine"].get("body", {}), home),
    ), "assembly": cfg}
    record.write_run_config(run_dir, prov)

    op = OPERATORS[args.operator]
    for task_id in args.task_ids:
        for seed in args.seeds:
            try:
                rec = run_trial(
                    cfg, cfg_path, run_dir, args.task_suite, task_id, seed, op,
                    args.wall_clock_min, ros_domain=args.ros_domain,
                    credentials_dir=args.credentials_dir,
                    account_alias=args.account_alias, script=args.script,
                    token_file=args.token_file, home=home,
                )
            except triallock.TrialLocked as e:
                # Not a failure of this trial: someone else is doing it.
                # Say so and leave their work alone (SystemExit belongs to
                # the entry point; the library raised).
                raise SystemExit(f"[trial] {e}")
            trial_dir = (run_dir / "trials" / f"{args.task_suite}-{task_id}"
                         / f"seed{seed}")
            record.write_trial_provenance(trial_dir, prov)
            print(
                f"[{args.task_suite}:{task_id} seed{seed}] "
                f"success={rec['success']} term={rec['termination']} "
                f"wall={rec['wall_seconds']}s"
            )


if __name__ == "__main__":
    main()
