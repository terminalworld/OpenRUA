"""The agent operator: a headless agent run as one session of segments.

The agent is launched as a subprocess of the agents launcher (never
imported). A trial that hits the account's quota wall is suspended, not
thrown away: the sandbox and the robot stay up (the clock is paused, so
a wait of hours changes nothing in the world), and when the window
reopens the same session is resumed. Waiting happens here, inside the
trial, so the containers never outlive the process that owns them.

Suspension is off by default: a quota wall is an artifact of running at
scale on subscription accounts, not part of the method. The protocol
switches it on (``resume_on_quota_wall``), and the record says which way
a trial ran.

Both budgets are spent across segments, never per segment: the wall
clock counts active seconds only, and the turn budget is carried
because the CLI restarts its own count on every resume. Turns already
spent come from the transcript's count, which runs slightly ahead of the
budget scale, so a resumed trial ends up with marginally fewer turns
than an uninterrupted one, the safe direction for comparability.
"""

from __future__ import annotations

import subprocess
import time
import uuid
from pathlib import Path

from robocli import agents


def agent_operator(ctx: dict) -> dict:
    """Run the agent on the trial; returns the metadata recorded under
    ``operator_meta`` (termination, turns, segments, quota facts)."""
    import os
    import sys

    cfg = ctx["cfg"]
    agent_cfg = cfg.get("agent", {})
    agent = agents.get(agent_cfg.get("name"), ctx.get("home"), version=agent_cfg.get("version"))
    model = agent_cfg.get("model") or agent.default_model
    # Agent knobs (reasoning effort, compaction threshold, tool timeouts)
    # are experimental parameters that do not appear in the transcript:
    # the merged set is pinned here from the manifest's defaults and the
    # config, passed to the launcher explicitly, and recorded.
    options = {**agent.default_options, **agent_cfg.get("options", {})}
    # Naming the session up front is what makes a suspended trial
    # resumable without scraping an id out of a half-written transcript.
    session_id = str(uuid.uuid4())
    trial_dir: Path = ctx["trial_dir"]
    transcript = trial_dir / "transcript.jsonl"
    max_turns = int(cfg.get("protocol", {}).get("max_turns", 100))
    base = [
        sys.executable, "-m", "robocli.agents.launcher",
        "--sandbox", ctx["sandbox"],
        "--task", ctx["task_language"],
        "--transcript", str(transcript),
        "--agent", agent.name,
        "--model", model,
        *(x for k, v in options.items() for x in ("--option", f"{k}={v}")),
        *(["--home", str(ctx["home"])] if ctx.get("home") else []),
        "--session-id", session_id,
        "--proxy", ctx.get("proxy", "http://robocli-proxy:8888"),
        *(["--token-file", str(ctx["token_file"])] if ctx.get("token_file")
          else []),
    ]
    meta: dict = {"operator": "agent", "agent": agent.name, "model": model,
                  "options": options, "session_id": session_id}
    # The launcher runs with cwd=trial_dir, where relative PYTHONPATH
    # entries would no longer resolve: absolutize them against this
    # process's cwd.
    env = dict(os.environ)
    if env.get("PYTHONPATH"):
        env["PYTHONPATH"] = os.pathsep.join(
            str(Path(p).resolve()) for p in env["PYTHONPATH"].split(os.pathsep) if p)

    # Waiting out a wall is bounded on both axes: a wall further away
    # than max_quota_wait_minutes, or one that keeps reappearing past
    # max_suspensions, ends the trial as quota-limited for the caller to
    # requeue.
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
        # Where this segment starts in the transcript. The runner owns
        # this boundary rather than inferring it from record shapes. The
        # first segment starts at 0 because the launcher truncates for
        # it; trial directories are reused across retries, so reading the
        # length here would carry a dead attempt's line count into a
        # fresh file and scan past its end.
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
        # Absolute bounds, not just a duration: post-hoc budget judging
        # asks how much active time had passed when a success latched,
        # and only per-segment timestamps can answer that for a
        # suspended trial.
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
        # A quota rejection is the one reason to wait rather than
        # finish; success, max turns and a dead CLI all end the trial.
        # Read this segment only: a whole-file verdict would re-suspend a
        # trial that already finished.
        last_quota = agent.quota_since(transcript, mark) or {}
        resets_at = last_quota.get("resets_at")
        if not resets_at:
            break
        if not resume_on_wall:
            # The wall is in the transcript and unresolved; post-hoc
            # classification voids the attempt and the caller requeues it.
            break
        # Both containers must still be standing: the sandbox holds the
        # session to resume, the robot holds the world it was working
        # on. (The stall watchdog can itself kill the sandbox when the
        # sim wedges; this catches that too.)
        missing = [n for n in (ctx["sandbox"], ctx.get("sim"))
                   if n and not _container_running(n)]
        if missing:
            meta["termination"] = "sandbox_lost"
            meta["lost_containers"] = missing
            break
        try:
            wait_s = max(0.0, float(resets_at) - time.time())
        except (TypeError, ValueError):
            # A reset time that cannot be read cannot be waited for; the
            # wall stays in the transcript and the caller requeues.
            meta["quota_gave_up_on"] = "unreadable-reset-time"
            break
        if wait_s > max_wait_s or len(segments) > max_suspensions:
            why = ("wait" if wait_s > max_wait_s else "suspensions")
            meta["termination"] = "quota_limit"
            meta["quota_gave_up_on"] = why
            print(f"[trial] quota wall beyond the {why} bound; giving up "
                  "for the caller to requeue", flush=True)
            break
        print(f"[trial] quota wall; suspending {round(wait_s / 60)}min "
              f"until the window reopens", flush=True)
        time.sleep(wait_s)
        suspended_s += wait_s
        resumed = True

    meta["segments"] = len(segments)
    meta["resumes"] = max(0, len(segments) - 1)
    # Whether a quota wall was waited out rather than fatal. A transcript
    # carrying a quota rejection voids a trial that was cut off; a
    # resumed trial was not cut off, and this flag tells the two apart.
    # Only a trial whose last segment came back clean counts as
    # resolved: a wall can arrive with no reset time, and such a trial
    # ended at a wall however it is labelled.
    meta["quota_resolved"] = bool(
        meta["resumes"] and not last_quota and meta.get("termination") not in
        ("quota_limit", "sandbox_lost", "launcher_failed"))
    meta["active_seconds"] = round(active_s, 1)
    meta["suspended_seconds"] = round(suspended_s, 1)
    meta["segment_detail"] = segments
    # The launcher opens the transcript before the CLI runs, so a CLI
    # that hung producing nothing (bad credentials, dead proxy) leaves an
    # empty file, on the timeout path too. A nonzero exit alone is not
    # the signal: the CLI exits nonzero on a legitimate max-turns as well.
    if not transcript.exists() or not transcript.read_text().strip():
        meta["termination"] = "launcher_failed"
    # Turn count and the max-turns verdict from the CLI's final record,
    # read by the agent's hooks (the transcript format is agent
    # knowledge; across segments it reports the trial's totals). Whether
    # the attempt counts (quota-voided, corrupt) is decided post hoc from
    # the artifacts, not here.
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
    """Whether a container is still up (consulted before waiting out a
    quota wall: resuming needs the containers left mid-task)."""
    try:
        out = subprocess.run(
            ["docker", "inspect", "-f", "{{.State.Running}}", name],
            capture_output=True, text=True, timeout=30)
    except (subprocess.TimeoutExpired, OSError):
        return False
    return out.stdout.strip() == "true"
