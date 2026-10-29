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


@pytest.mark.parametrize('success', [True, False])
def test_completed_execution_is_saved_before_judgment_and_cleanup(monkeypatch, tmp_path, proxy_address, success):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    run = tmp_path / 'run'
    result = run / 'trials/suite-0/seed0/result.json'
    secret = 'abcdefghijklmnopqrstuvwxyz0123456789'
    monkeypatch.setattr(trial, 'last_words', lambda *_: f'Finished. {secret}')
    observed = []

    def judge(words):
        snapshot = json.loads(result.read_text())
        assert snapshot['termination'] == 'self_finished'
        assert snapshot['operator_meta']['active_seconds'] == 12.5
        assert snapshot['agent_final_message'] == 'Finished. [REDACTED]'
        assert snapshot['success'] is None and snapshot['verdict'] == 'pending'
        assert snapshot['active_wall_cap_min'] == 60
        assert snapshot['cleanup'] == {'status': 'pending', 'errors': []}
        assert 'shutdown' not in handle.calls
        observed.append('judge')
        return success

    def shutdown():
        snapshot = json.loads(result.read_text())
        assert snapshot['success'] is success and snapshot['verdict'] == 'person'
        assert snapshot['cleanup']['status'] == 'pending'
        observed.append('shutdown')

    handle.shutdown = shutdown
    rec = trial.run_trial(cfg, tmp_path / 'config.yaml', run, 'suite', 0, 0,
                          lambda _: {'termination': 'self_finished', 'active_seconds': 12.5}, 60,
                          token_file=str(token), task='inspect', home=tmp_path / 'home', judge=judge)
    assert observed == ['judge', 'shutdown']
    assert rec['success'] is success and rec['cleanup']['status'] == 'completed'
    assert json.loads(json.dumps(rec)) == json.loads(result.read_text())
    assert secret not in result.read_text()


@pytest.mark.parametrize('error', [KeyboardInterrupt, EOFError, RuntimeError])
def test_judgment_error_does_not_rewrite_a_completed_execution(monkeypatch, tmp_path, proxy_address, error):
    handle, cfg, token = _harness(monkeypatch, tmp_path)

    def judge(words):
        raise error('judgment unavailable')

    rec = trial.run_trial(cfg, tmp_path / 'config.yaml', tmp_path / 'run', 'suite', 0, 0,
                          lambda _: {'termination': 'self_finished'}, 60,
                          token_file=str(token), task='inspect', home=tmp_path / 'home', judge=judge)
    assert rec['termination'] == 'self_finished' and rec['anomaly'] is None
    assert rec['success'] is None
    assert rec['cleanup']['status'] == 'completed' and handle.calls[-1] == 'shutdown'
    if error is RuntimeError:
        assert rec['verdict'] == 'pending'
        assert rec['verdict_error'] == 'RuntimeError: judgment unavailable'
    else:
        assert rec['verdict'] == 'not_applicable' and 'verdict_error' not in rec


def test_cleanup_failures_preserve_verdict_and_attempt_remaining_cleanup(monkeypatch, tmp_path, proxy_address):
    from types import SimpleNamespace
    from openrua.runner import lock

    handle, cfg, token = _harness(monkeypatch, tmp_path)
    run = tmp_path / 'run'
    directory = run / 'trials/suite-0/seed0'
    calls = []
    secret = 'abcdefghijklmnopqrstuvwxyz0123456789'

    def fail(step):
        calls.append(step)
        assert json.loads((directory / 'result.json').read_text())['success'] is True
        raise RuntimeError(f'{step} unavailable: {secret}')

    monkeypatch.setattr(trial, 'record_cameras_of', lambda *_: SimpleNamespace(
        cameras={'front': '/camera'}, stop=lambda: fail('camera')))
    handle.shutdown = lambda: fail('robot')
    monkeypatch.setattr(trial, 'sandbox_down', lambda _: fail('sandbox'))
    monkeypatch.setattr(trial.record, 'finalize_trial', lambda *a, **k: fail('artifacts'))
    rec = trial.run_trial(cfg, tmp_path / 'config.yaml', run, 'suite', 0, 0,
                          lambda _: {'termination': 'self_finished'}, 60,
                          token_file=str(token), task='inspect', home=tmp_path / 'home',
                          judge=lambda _: True, record_cameras=())
    assert calls == ['camera', 'robot', 'sandbox', 'artifacts']
    assert rec['termination'] == 'self_finished' and rec['success'] is True
    assert rec['anomaly'] is None and rec['verdict'] == 'person'
    assert rec['cleanup']['status'] == 'failed'
    assert [e['step'] for e in rec['cleanup']['errors']] == [
        'camera recording', 'robot', 'agent sandbox', 'trial artifacts']
    assert Path(rec['cleanup']['retained_directory']).is_dir()
    assert json.loads(json.dumps(rec)) == json.loads((directory / 'result.json').read_text())
    assert secret not in (directory / 'result.json').read_text()
    assert secret not in json.dumps(rec)
    assert lock.holder(directory) is None


def test_killed_runner_leaves_completed_execution_waiting_for_judgment(monkeypatch, tmp_path, proxy_address):
    import multiprocessing

    _, cfg, token = _harness(monkeypatch, tmp_path)
    context = multiprocessing.get_context('fork')
    waiting = context.Event()

    def judge(words):
        waiting.set()
        context.Event().wait(30)

    def run():
        trial.run_trial(cfg, tmp_path / 'config.yaml', tmp_path / 'run', 'suite', 0, 0,
                        lambda _: {'termination': 'self_finished'}, 60,
                        token_file=str(token), task='inspect', home=tmp_path / 'home', judge=judge)

    process = context.Process(target=run)
    process.start()
    try:
        assert waiting.wait(5), 'runner did not reach its judgment prompt'
        process.kill()
        process.join(5)
        assert process.exitcode is not None and process.exitcode < 0
        snapshot = json.loads((tmp_path / 'run/trials/suite-0/seed0/result.json').read_text())
        assert snapshot['termination'] == 'self_finished'
        assert snapshot['success'] is None and snapshot['verdict'] == 'pending'
        assert snapshot['cleanup']['status'] == 'pending'
    finally:
        if process.is_alive():
            process.kill()
            process.join(5)


def test_result_write_failure_still_cleans_resources_and_releases_claim(monkeypatch, tmp_path, proxy_address):
    from openrua.runner import lock

    handle, cfg, token = _harness(monkeypatch, tmp_path)
    cleaned = []
    monkeypatch.setattr(trial, 'sandbox_down', lambda _: cleaned.append('sandbox'))
    monkeypatch.setattr(trial.record, 'finalize_trial', lambda *a, **k: cleaned.append('artifacts'))

    def write_result(*args):
        raise OSError('disk unavailable')

    monkeypatch.setattr(trial.record, 'write_result', write_result)
    with pytest.raises(OSError, match='disk unavailable'):
        trial.run_trial(cfg, tmp_path / 'config.yaml', tmp_path / 'run', 'suite', 0, 0,
                        lambda _: {'termination': 'self_finished'}, 60,
                        token_file=str(token), task='inspect', home=tmp_path / 'home')
    assert handle.calls[-1] == 'shutdown'
    assert cleaned == ['sandbox', 'artifacts']
    assert lock.holder(tmp_path / 'run/trials/suite-0/seed0') is None


def test_failed_automatic_measurement_keeps_unknown_success_and_execution(monkeypatch, tmp_path, proxy_address):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    cfg['machine']['backend']['kind'] = 'sim'
    result = tmp_path / 'run/trials/suite-0/seed0/result.json'

    def score(rec, machine):
        saved = json.loads(result.read_text())
        assert saved['termination'] == 'self_finished' and saved['success'] is None
        raise RuntimeError('scene not answering')

    monkeypatch.setattr(trial, 'score', score)
    rec = trial.run_trial(cfg, tmp_path / 'config.yaml', tmp_path / 'run', 'suite', 0, 0,
                          lambda _: {'termination': 'self_finished'}, 60,
                          token_file=str(token), task='inspect', home=tmp_path / 'home')
    assert rec['success'] is None and rec['termination'] == 'self_finished'
    assert rec['anomaly'] == 'RuntimeError: scene not answering'
    assert rec['cleanup']['status'] == 'completed'
    assert handle.calls[-1] == 'shutdown'
