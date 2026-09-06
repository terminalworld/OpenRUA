"""A trial on a robot with no truth side: no reset, the caller's task,
a not-applicable verdict, everything else as usual."""

import json
from pathlib import Path

from robocli.runner import trial


class _Handle:
    name = "r-sim"

    def __init__(self):
        self.calls = []

    def rpc(self, obj, timeout_note="", timeout_s=900.0):
        self.calls.append(obj["cmd"])
        return {"ok": True, "not_applicable": True}

    def shutdown(self):
        self.calls.append("shutdown")


def _harness(monkeypatch, tmp_path):
    handle = _Handle()
    monkeypatch.setattr(trial, "ensure_internal_network", lambda: "net")
    monkeypatch.setattr(trial, "ensure_proxy", lambda net: "http://p:8888")
    monkeypatch.setattr(trial, "bring_up", lambda *a, **k: (tmp_path / "config.yaml", handle))
    monkeypatch.setattr(trial, "sandbox_down", lambda name: None)
    monkeypatch.setattr(trial, "run_preflight",
                        lambda cfg, sandbox, agent: {"ok": True, "checks": [("x", True)], "failed": []})
    monkeypatch.setattr(trial.record, "finalize_trial", lambda *a, **k: None)
    token = tmp_path / "token.env"
    token.write_text("T=abcdefghijklmnopqrstuvwxyz0123456789\n")
    cfg = {"agent": {"name": "claude-code"}, "protocol": {},
           "machine": {"backend": {"kind": "real", "discovery": {"network": "host"}},
                       "workspace_template": None}}
    return handle, cfg, token


def test_real_trial_uses_the_given_task_and_scores_nothing(monkeypatch, tmp_path):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    run_dir = tmp_path / "runs" / "r1"
    rec = trial.run_trial(cfg, tmp_path / "bench.yaml", run_dir, "suite", 0, 0,
                          lambda ctx: {"operator": "none"}, 1.0, token_file=str(token),
                          task="open the drawer")
    assert rec["task_language"] == "open the drawer"
    assert rec["success"] is None and rec["verdict"] == "not_applicable"
    assert rec["anomaly"] is None and rec["termination"] == "operator_done"
    assert "reset" not in handle.calls and "success" not in handle.calls
    written = json.loads((run_dir / "trials" / "suite-0" / "seed0" / "result.json").read_text())
    assert written["verdict"] == "not_applicable"


def test_real_trial_without_a_task_is_an_anomaly(monkeypatch, tmp_path):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    rec = trial.run_trial(cfg, tmp_path / "bench.yaml", tmp_path / "runs" / "r2", "suite", 0, 0,
                          lambda ctx: {"operator": "none"}, 1.0, token_file=str(token))
    assert rec["termination"] == "anomaly" and "--task" in rec["anomaly"]
