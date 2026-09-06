"""The trial as a sequence of segments (ruling 2026-08-20).

Suspension is a config switch, OFF in the code: a quota wall comes from
running at scale on subscription accounts, not from the method, and a
single-trial reproduction never meets one. These cases turn it on except
where they say otherwise.

Drives agent_operator's real loop against a fake launcher that writes real
transcript records, so the adapter's own readers run too. What matters here
is not that a subprocess was called but that the loop SPENDS BOTH BUDGETS
ACROSS SEGMENTS and knows when to stop waiting.
"""

import json

import pytest

from robocli.bench import run as R

WALL = {"type": "rate_limit_event",
        "rate_limit_info": {"status": "rejected", "rateLimitType": "five_hour",
                            "resetsAt": 0}}          # resets_at patched per test
INIT = {"type": "system", "subtype": "init"}


def _result(turns, subtype="success"):
    return {"type": "result", "subtype": subtype, "num_turns": turns,
            "is_error": subtype != "success",
            "usage": {"output_tokens": 1}, "total_cost_usd": 0.1,
            "duration_ms": 1000}


@pytest.fixture
def harness(tmp_path, monkeypatch):
    """A trial whose segments are scripted, with waiting made instant."""
    slept: list[float] = []
    launched: list[list[str]] = []
    monkeypatch.setattr(R.time, "sleep", lambda s: slept.append(s))
    monkeypatch.setattr(R, "_container_running", lambda name: True)

    def build(segments, wait_min=1.0, **protocol):
        """segments: list of record-lists, one per launcher invocation."""
        transcript = tmp_path / "transcript.jsonl"
        pending = list(segments)

        def fake_run(cmd, **kw):
            launched.append(cmd)
            recs = pending.pop(0) if pending else [INIT, _result(1)]
            # Mirror the real launcher: it TRUNCATES for a fresh session and
            # APPENDS on resume. A fake that always appends cannot catch a
            # runner that mis-marks where the segment starts.
            mode = "a" if "--resume" in cmd else "w"
            with open(transcript, mode) as f:
                for r in recs:
                    r = dict(r)
                    info = r.get("rate_limit_info")
                    # Only fill in the numeric placeholder; a test that puts
                    # a deliberately malformed value there must get it back
                    # unchanged, or the fake quietly repairs the input and
                    # the case under test never happens.
                    if info and isinstance(info.get("resetsAt"), (int, float)):
                        r["rate_limit_info"] = {
                            **info, "resetsAt": R.time.time() + wait_min * 60}
                    f.write(json.dumps(r) + "\n")
            return type("P", (), {"returncode": 0})()

        monkeypatch.setattr(R.subprocess, "run", fake_run)
        cfg = {"agent": {"name": "claude-code", "model": "m"},
               # suspension is off by default in the code; these cases are
               # about what happens when a campaign turns it on
               "protocol": {"max_turns": 100, "resume_on_quota_wall": True,
                            **protocol}}
        ctx = {"cfg": cfg, "sandbox": "rc-x-sandbox", "task_language": "T",
               "trial_dir": tmp_path, "proxy": "http://p",
               "sim": "rc-x-sim",
               "active_wall_clock_min": 240}
        return R.agent_operator(ctx)

    build.slept = slept
    build.launched = launched
    return build


def test_wall_suspends_and_the_same_session_resumes(harness):
    meta = harness([[INIT, WALL, _result(4, "error_during_execution")],
                    [INIT, _result(6)]])
    assert meta["segments"] == 2 and meta["resumes"] == 1
    assert meta["termination"] == "self_finished"
    # the whole point: a waited-out wall does not void the trial
    assert meta["quota_resolved"] is True
    # one session throughout; the second call continues it
    first, second = harness.launched
    sid = first[first.index("--session-id") + 1]
    assert second[second.index("--session-id") + 1] == sid
    assert "--resume" in second and "--resume" not in first


def test_turn_budget_is_carried_not_restarted(harness):
    # The CLI restarts --max-turns on every resume (measured 2026-08-20), so
    # a loop that passed the full budget again would hand a resumed trial
    # more turns than an uninterrupted one ever gets.
    harness([[INIT, WALL, _result(30, "error_during_execution")],
             [INIT, _result(40)]], max_turns=100)
    first, second = harness.launched
    assert first[first.index("--max-turns") + 1] == "100"
    assert second[second.index("--max-turns") + 1] == "70"   # 100 - 30 spent


def test_suspension_is_not_charged_to_the_wall_clock(harness):
    meta = harness([[INIT, WALL, _result(1, "error_during_execution")],
                    [INIT, _result(2)]], wait_min=90)
    assert meta["suspended_seconds"] == pytest.approx(90 * 60, rel=0.01)
    # active time is the agent's own; it must not include the wait
    assert meta["active_seconds"] < 60


def test_finished_trial_is_not_re_suspended_by_the_sticky_verdict(harness):
    # Regression: the audit's quota verdict is sticky across the whole file,
    # so reading IT here would see segment 1's rejection again after segment
    # 2 finished, and keep suspending a completed trial until it ran out of
    # suspensions and the work was thrown away.
    meta = harness([[INIT, WALL, _result(1, "error_during_execution")],
                    [INIT, _result(2)]])
    assert meta["segments"] == 2          # not 5, not quota_limit
    assert len(harness.slept) == 1


def test_wall_beyond_the_wait_bound_gives_up_for_the_master(harness):
    # A weekly wall must not park the lane for days: give up as quota-limited
    # exactly as before resume existed, and let the master requeue.
    meta = harness([[INIT, WALL, _result(1, "error_during_execution")]],
                   wait_min=60 * 24 * 7, max_quota_wait_minutes=360)
    assert meta["termination"] == "quota_limit"
    assert meta["quota_gave_up_on"] == "wait"
    assert meta["quota_resolved"] is False
    assert harness.slept == []


def test_repeated_walls_stop_at_the_suspension_bound(harness):
    walled = [INIT, WALL, _result(1, "error_during_execution")]
    meta = harness([walled] * 8, max_suspensions=2)
    assert meta["termination"] == "quota_limit"
    assert meta["quota_gave_up_on"] == "suspensions"
    assert meta["quota_resolved"] is False


@pytest.mark.parametrize("gone", ["rc-x-sandbox", "rc-x-sim"])
def test_a_missing_body_stops_instead_of_resuming_into_nothing(
        harness, monkeypatch, gone):
    # Either body missing is fatal: the sandbox holds the session to resume,
    # the sim holds the world. Resuming into a half-present world would
    # produce a trial that looks complete and measured nothing.
    monkeypatch.setattr(R, "_container_running", lambda name: name != gone)
    meta = harness([[INIT, WALL, _result(1, "error_during_execution")]])
    assert meta["termination"] == "sandbox_lost"
    assert meta["lost_containers"] == [gone]
    assert meta["quota_resolved"] is False
    assert harness.slept == []


def test_wall_is_judged_without_relying_on_the_init_marker(harness):
    # The segment boundary is the runner's line mark, not a record shape:
    # a CLI that stopped emitting system/init on resume must not turn a
    # finished trial back into an endlessly re-suspended one.
    meta = harness([[WALL, _result(1, "error_during_execution")],   # no INIT
                    [_result(2)]])                                  # no INIT
    assert meta["segments"] == 2 and meta["quota_resolved"] is True
    assert len(harness.slept) == 1


def test_uninterrupted_trial_is_one_segment_and_unflagged(harness):
    meta = harness([[INIT, _result(9)]])
    assert meta["segments"] == 1 and meta["resumes"] == 0
    assert meta["quota_resolved"] is False   # nothing to resolve
    assert meta["num_turns"] == 9
    assert "--resume" not in harness.launched[0]


WALL_NO_RESET = {"type": "result", "subtype": "error_during_execution",
                 "is_error": True, "num_turns": 1,
                 "result": "You've hit your weekly limit · resets Monday"}


def test_wall_without_a_reset_time_is_not_counted_as_resolved(harness):
    # The 2026-08-09 weekly shape carries an error phrase and no resetsAt.
    # The loop cannot wait for a time it was not given, so it stops -- but
    # the trial ended AT a wall, and calling that resolved would slip a
    # truncated trial into the denominator.
    meta = harness([[INIT, WALL, _result(1, "error_during_execution")],
                    [INIT, WALL_NO_RESET]])
    assert meta["resumes"] == 1                 # the first wall was waited out
    assert meta["quota_resolved"] is False      # the second one ended it


def test_retry_reuses_the_directory_and_still_sees_a_wall(harness, tmp_path):
    # Trial directories are reused across retries, so a dead attempt's
    # transcript is still on disk when a fresh one starts. The launcher
    # truncates for the first segment, so the first segment's mark must be
    # 0: carrying the old length forward would scan past the new file's end
    # and miss the wall entirely, and resume would never fire.
    (tmp_path / "transcript.jsonl").write_text("\n".join(
        json.dumps({"type": "assistant"}) for _ in range(400)) + "\n")
    meta = harness([[INIT, WALL, _result(1, "error_during_execution")],
                    [INIT, _result(2)]])
    assert meta["resumes"] == 1 and meta["quota_resolved"] is True


def test_segments_carry_absolute_bounds(harness):
    # Post-hoc budget judging asks how much ACTIVE time had passed when a
    # success latched; only per-segment timestamps can answer that once a
    # trial has been suspended.
    meta = harness([[INIT, WALL, _result(1, "error_during_execution")],
                    [INIT, _result(2)]])
    for seg in meta["segment_detail"]:
        assert seg["started_unix"] <= seg["ended_unix"]
    a, b = meta["segment_detail"]
    assert b["started_unix"] >= a["ended_unix"]


def test_unreadable_reset_time_ends_the_trial_instead_of_crashing(harness):
    # A reset time we cannot read is a reset time we cannot wait for. The
    # trial stops; the wall is still in the transcript and still unresolved,
    # so the audit voids it and the master requeues -- the old path.
    bad = {"type": "rate_limit_event",
           "rate_limit_info": {"status": "rejected", "rateLimitType": "five_hour",
                               "resetsAt": "soon-ish"}}
    meta = harness([[INIT, bad, _result(1, "error_during_execution")]])
    assert meta["quota_gave_up_on"] == "unreadable-reset-time"
    assert meta["quota_resolved"] is False
    assert harness.slept == []


def test_switched_off_a_wall_ends_the_trial_as_it_always_did(harness):
    # The default. Byte-for-byte the pre-suspension path: one segment, no
    # waiting, the wall left unresolved for the audit to void.
    meta = harness([[INIT, WALL, _result(1, "error_during_execution")],
                    [INIT, _result(2)]],
                   resume_on_quota_wall=False)
    assert meta["segments"] == 1 and meta["resumes"] == 0
    assert meta["quota_resolved"] is False
    assert harness.slept == []
    assert meta["resume_on_quota_wall"] is False


def test_the_switch_is_recorded_on_the_trial(harness):
    # Which way a trial ran must be readable from the trial, not inferred
    # from a config file's history.
    assert harness([[INIT, _result(1)]])["resume_on_quota_wall"] is True
