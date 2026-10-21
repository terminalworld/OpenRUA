"""A person stops a run (Ctrl-C, or the terminal closing): the agent is
ended, a real robot's motions are cancelled, the robot is powered off,
and the record says it was stopped, not that something broke."""

from openrua.runner import session as R
from openrua.runner import trial
from tests.runner.test_trial_real import _harness


def test_ctrl_c_during_the_agent_ends_its_segment_as_stopped(tmp_path, monkeypatch):
    stopped = []

    def interrupted(cmd, **kw):
        assert kw.get("start_new_session")   # the terminal's Ctrl-C is the runner's alone
        raise KeyboardInterrupt

    monkeypatch.setattr(R.subprocess, "run", interrupted)
    monkeypatch.setattr(R, "stop_processes", stopped.append)
    (tmp_path / "transcript.jsonl").write_text('{"type": "system", "subtype": "init"}\n')
    meta = R.agent_operator({
        "cfg": {"agent": {"name": "claude-code", "model": "m"}, "protocol": {}},
        "sandbox": "rc-x-sandbox", "task_language": "T", "trial_dir": tmp_path,
        "profile_dir": tmp_path / "profile", "proxy": "", "sim": "rc-x-sim",
        "limits": {"internet": "open", "web_search": True, "account_connectors": True},
        "active_wall_clock_min": 5})
    assert meta["termination"] == "stopped" and meta["segments"] == 1
    assert stopped == ["rc-x-sandbox"]   # the CLI outlives its launcher otherwise


def test_a_stopped_real_robot_has_its_motions_cancelled_and_is_recorded_stopped(
        monkeypatch, tmp_path, proxy_address):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    cancelled, ended = [], []
    monkeypatch.setattr(trial, "cancel_motion", lambda sandbox, actions: cancelled.append(sandbox))
    monkeypatch.setattr(trial, "stop_processes", ended.append)
    rec = trial.run_trial(cfg, tmp_path / "bench.yaml", tmp_path / "runs" / "r1", "suite", 0, 0,
                          lambda ctx: {"operator": "agent", "termination": "stopped"}, 1.0,
                          token_file=str(token), task="open the drawer", home=tmp_path / "home")
    assert rec["termination"] == "stopped" and rec["anomaly"] is None
    assert len(cancelled) == 1 and ended == []   # the operator already ended the agent
    assert handle.calls[-1] == "shutdown"


def test_ctrl_c_during_bring_up_is_a_stop_not_an_anomaly(monkeypatch, tmp_path, proxy_address):
    handle, cfg, token = _harness(monkeypatch, tmp_path)

    def bring_up(*a, **k):
        raise KeyboardInterrupt

    monkeypatch.setattr(trial, "bring_up", bring_up)
    rec = trial.run_trial(cfg, tmp_path / "bench.yaml", tmp_path / "runs" / "r2", "suite", 0, 0,
                          lambda ctx: {"operator": "none"}, 1.0, token_file=str(token),
                          task="open the drawer", home=tmp_path / "home")
    assert rec["termination"] == "stopped" and rec["anomaly"] is None
    assert rec["success"] is None
