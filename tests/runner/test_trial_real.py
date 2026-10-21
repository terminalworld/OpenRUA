"""A trial on a robot with no truth side: no reset, the caller's task,
a not-applicable verdict, everything else as usual."""

import json
from pathlib import Path

import pytest

from openrua.config.schema import TRIAL_LIMITS
from openrua.runner import trial


class _Handle:
    name = "r-sim"

    def __init__(self):
        self.calls = []

    def rpc(self, obj, timeout_note="", timeout_s=900.0):
        self.calls.append(obj["cmd"])
        return {"ok": True, "not_applicable": True}

    def shutdown(self):
        self.calls.append("shutdown")


def _harness(monkeypatch, tmp_path, seen=None):
    handle = _Handle()
    seen = seen if seen is not None else []
    monkeypatch.setattr(trial, "ensure_internal_network", lambda: "net")
    monkeypatch.setattr(trial, "ensure_proxy", lambda network, whitelist:
                        seen.append(("proxy", whitelist)) or "http://openrua-proxy-ab12cd34:8888")
    def bring_up(cfg, dest, sim, sandbox, suite, task_id, network, internet, *a, **k):
        seen.append(("internet", internet))
        return tmp_path / "config.yaml", handle, 0
    monkeypatch.setattr(trial, "bring_up", bring_up)
    monkeypatch.setattr(trial, "sandbox_down", lambda name: None)
    monkeypatch.setattr(trial, "run_preflight",
                        lambda cfg, sandbox, agent: {"ok": True, "checks": [("x", True)], "failed": []})
    monkeypatch.setattr(trial.record, "finalize_trial", lambda *a, **k: None)
    monkeypatch.setattr(trial, "_sandbox_version", lambda sandbox, agent: "9.9.9")
    token = tmp_path / "token.env"
    token.write_text("T=abcdefghijklmnopqrstuvwxyz0123456789\n")
    cfg = {"agent": {"name": "claude-code"}, "protocol": {"limits": TRIAL_LIMITS.model_dump()},
           "machine": {"backend": {"kind": "real", "discovery": {"network": "host"}},
                       "workspace_template": None}}
    return handle, cfg, token


def test_real_trial_uses_the_given_task_and_scores_nothing(monkeypatch, tmp_path, proxy_address):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    run_dir = tmp_path / "runs" / "r1"
    rec = trial.run_trial(cfg, tmp_path / "bench.yaml", run_dir, "suite", 0, 0,
                          lambda ctx: {"operator": "none"}, 1.0, token_file=str(token),
                          task="open the drawer", home=tmp_path / "home")
    assert rec["task_language"] == "open the drawer"
    assert rec["success"] is None and rec["verdict"] == "not_applicable"
    assert rec["anomaly"] is None and rec["termination"] == "operator_done"
    assert "reset" not in handle.calls and "success" not in handle.calls
    written = json.loads((run_dir / "trials" / "suite-0" / "seed0" / "result.json").read_text())
    assert written["verdict"] == "not_applicable"


def test_real_trial_without_a_task_is_an_anomaly(monkeypatch, tmp_path):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    rec = trial.run_trial(cfg, tmp_path / "bench.yaml", tmp_path / "runs" / "r2", "suite", 0, 0,
                          lambda ctx: {"operator": "none"}, 1.0, token_file=str(token), home=tmp_path / "home")
    assert rec["termination"] == "anomaly" and "--task" in rec["anomaly"]


@pytest.mark.parametrize("name", ["codex", "claude-code"])
def test_api_trial_uses_isolated_profile_and_redacts_key(monkeypatch, tmp_path, name, proxy_address):
    handle, cfg, _ = _harness(monkeypatch, tmp_path)
    key_file = tmp_path / "key"
    secret = "synthetic-openrua-api-secret-for-redaction"
    key_file.write_text(secret)
    cfg["agent"] = {"name": name, "auth": {"mode": "api", "key_file": str(key_file)}}
    monkeypatch.setattr(trial.agents, "resolve_profile", lambda *a, **kw: pytest.fail("no native login fallback"))
    finalized = []
    def finalize(directory, agent, secrets, *, profile_dir):
        finalized.append((secrets, profile_dir))
        if name == "codex":
            assert json.loads((profile_dir / "auth.json").read_text())["OPENAI_API_KEY"] == secret
        else:
            assert (profile_dir / "api-key").read_text() == secret
            assert not (profile_dir / ".credentials.json").exists()
            # a trial keeps the account's extras out (protocol.limits)
            settings = json.loads((profile_dir / "settings.json").read_text())
            assert settings["disableClaudeAiConnectors"] is True
    monkeypatch.setattr(trial.record, "finalize_trial", finalize)
    rec = trial.run_trial(cfg, tmp_path / "config.yaml", tmp_path / "run", "suite", 0, 0,
                          lambda ctx: {"operator": "fixture"}, 1.0,
                          task="inspect", home=tmp_path / "home")
    assert rec["anomaly"] is None and rec["auth_mode"] == "api"
    assert secret in finalized[0][0]
    assert not finalized[0][1].exists()
    assert key_file.read_text() == secret
    with pytest.raises(ValueError, match="cannot be combined"):
        trial.run_trial(cfg, tmp_path / "config.yaml", tmp_path / "conflict", "suite", 0, 0,
                        lambda _: None, 1.0, token_file="other.env", home=tmp_path / "home")
    assert not (tmp_path / "conflict").exists()


def test_a_trial_is_kept_to_its_model_service_by_its_protocol(monkeypatch, tmp_path, proxy_address):
    seen = []
    handle, cfg, token = _harness(monkeypatch, tmp_path, seen)
    contexts = []
    rec = trial.run_trial(cfg, tmp_path / "bench.yaml", tmp_path / "runs" / "r3", "suite", 0, 0,
                          lambda ctx: contexts.append(ctx) or {"operator": "none"}, 1.0,
                          token_file=str(token), task="inspect", home=tmp_path / "home")
    assert rec["anomaly"] is None
    whitelist = "\n".join(trial.agents.get("claude-code").whitelist)
    assert ("proxy", whitelist) in seen                       # the agent's own hosts
    assert ("internet", "proxy:http://openrua-proxy-ab12cd34:8888") in seen
    assert contexts[0]["limits"] == TRIAL_LIMITS.model_dump()
    assert contexts[0]["proxy"] == proxy_address      # host network: by address
    assert rec["limits"] == TRIAL_LIMITS.model_dump()


def test_a_trial_whose_protocol_opens_the_internet_starts_no_proxy(monkeypatch, tmp_path):
    seen = []
    handle, cfg, token = _harness(monkeypatch, tmp_path, seen)
    cfg["protocol"]["limits"] = {**TRIAL_LIMITS.model_dump(), "internet": "open"}
    contexts = []
    rec = trial.run_trial(cfg, tmp_path / "bench.yaml", tmp_path / "runs" / "r4", "suite", 0, 0,
                          lambda ctx: contexts.append(ctx) or {"operator": "none"}, 1.0,
                          token_file=str(token), task="inspect", home=tmp_path / "home")
    assert rec["anomaly"] is None
    assert not [x for x in seen if x[0] == "proxy"]
    assert ("internet", "open") in seen
    assert contexts[0]["proxy"] == "" and rec["limits"]["internet"] == "open"
