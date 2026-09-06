"""The agent operator: a headless agent run as one session of segments.

The agent is launched as a subprocess of the agents launcher (never
imported). Hitting the account's quota wall suspends the trial (sandbox
and robot left standing, clock paused) and the same session resumes
when the window reopens, if the protocol allows it. Both the wall clock
and the turn budget are spent across segments; the wall clock counts
active time only.
"""

from __future__ import annotations

import subprocess
import time
import uuid
from pathlib import Path

from robocli import agents


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
    agent = agents.get(agent_cfg.get("name"), ctx.get("home"))
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
