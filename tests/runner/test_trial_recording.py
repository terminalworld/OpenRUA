"""Recording follows execution and keeps its outcome separate from the task."""

import json

import pytest

from openrua.runner import trial
from tests.runner.test_trial_real import _harness


class Tap:
    cameras = {"front": "/camera"}

    def __init__(self, calls, result, *, failure=None, missing=False):
        self.calls, self.result = calls, result
        self.failure, self.missing = failure, missing
        self.stopped = False

    def stop(self):
        self.calls.append("recording stopped")
        self.stopped = True
        if self.failure:
            saved = json.loads(self.result.read_text())
            assert saved["termination"] == "self_finished"
            raise RuntimeError(self.failure)

    def status(self):
        return {"status": "incomplete" if self.missing else "completed" if self.stopped else "running",
                "missing_cameras": ["front"] if self.missing else [],
                "errors": [], "cameras": {"front": {"count": 0 if self.missing else 2}}}


def test_recording_starts_after_preflight_and_finishes_before_judgment(monkeypatch, tmp_path, proxy_address):
    _, cfg, token = _harness(monkeypatch, tmp_path)
    calls = []
    run = tmp_path / "run"
    result = run / "trials/suite-0/seed0/result.json"
    tap = Tap(calls, result)

    def preflight(*a, **kw):
        calls.append("preflight")
        return {"ok": True, "checks": [], "failed": []}

    def start(*a):
        calls.append("recording started")
        return tap

    def operator(ctx):
        assert not tap.stopped
        calls.append("agent")
        return {"termination": "self_finished"}

    def judge(words):
        assert tap.stopped
        saved = json.loads(result.read_text())
        assert saved["recording"]["status"] == "completed"
        assert saved["verdict"] == "pending"
        calls.append("judge")
        return True

    monkeypatch.setattr(trial, "run_preflight", preflight)
    monkeypatch.setattr(trial, "record_cameras_of", start)
    rec = trial.run_trial(cfg, tmp_path / "config.yaml", run, "suite", 0, 0,
                          operator, 60, token_file=str(token), task="inspect",
                          home=tmp_path / "home", judge=judge, record_cameras=())
    assert calls == ["preflight", "recording started", "agent", "recording stopped", "judge"]
    assert rec["success"] is True
    assert rec["recording"]["status"] == "completed"


@pytest.mark.parametrize("error", [RuntimeError, KeyboardInterrupt])
def test_recording_is_stopped_when_execution_raises(monkeypatch, tmp_path, proxy_address, error):
    _, cfg, token = _harness(monkeypatch, tmp_path)
    calls = []
    run = tmp_path / "run"
    tap = Tap(calls, run / "trials/suite-0/seed0/result.json")
    monkeypatch.setattr(trial, "record_cameras_of", lambda *a: tap)
    monkeypatch.setattr(trial, "halt", lambda *a, **kw: None)

    def operator(ctx):
        raise error("execution interrupted")

    rec = trial.run_trial(cfg, tmp_path / "config.yaml", run, "suite", 0, 0,
                          operator, 60, token_file=str(token), task="inspect",
                          home=tmp_path / "home", record_cameras=())
    assert calls == ["recording stopped"]
    assert rec["termination"] == ("stopped" if error is KeyboardInterrupt else "anomaly")
    assert rec["recording"]["status"] == "completed"


@pytest.mark.parametrize("failure,missing", [("camera stop failed", False), (None, True)])
def test_recording_problem_does_not_change_a_completed_task(monkeypatch, tmp_path, proxy_address, failure, missing):
    _, cfg, token = _harness(monkeypatch, tmp_path)
    calls = []
    run = tmp_path / "run"
    tap = Tap(calls, run / "trials/suite-0/seed0/result.json", failure=failure, missing=missing)
    monkeypatch.setattr(trial, "record_cameras_of", lambda *a: tap)
    rec = trial.run_trial(cfg, tmp_path / "config.yaml", run, "suite", 0, 0,
                          lambda _: {"termination": "self_finished"}, 60,
                          token_file=str(token), task="inspect", home=tmp_path / "home",
                          record_cameras=(), judge=lambda _: True)
    assert rec["success"] is True and rec["termination"] == "self_finished"
    assert rec["anomaly"] is None
    if failure:
        assert rec["cleanup"]["status"] == "failed"
        assert rec["cleanup"]["errors"][0]["step"] == "camera recording"
    else:
        assert rec["recording"]["status"] == "incomplete"
        assert rec["cleanup"]["status"] == "completed"


def test_failed_preflight_does_not_start_recording(monkeypatch, tmp_path, proxy_address):
    _, cfg, token = _harness(monkeypatch, tmp_path)
    monkeypatch.setattr(trial, "run_preflight", lambda *a, **kw:
                        {"ok": False, "checks": [], "failed": ["robot interface"]})
    monkeypatch.setattr(trial, "record_cameras_of", lambda *a:
                        pytest.fail("recording must follow successful preflight"))
    rec = trial.run_trial(cfg, tmp_path / "config.yaml", tmp_path / "run", "suite", 0, 0,
                          lambda _: pytest.fail("agent must not start"), 60,
                          token_file=str(token), task="inspect", home=tmp_path / "home",
                          record_cameras=())
    assert rec["termination"] == "anomaly"
    assert "preflight failed" in rec["anomaly"]
