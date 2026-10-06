"""Native profile layouts belong to plugins, not robot startup or clients."""

from types import SimpleNamespace

import pytest

from openrua import agents
from openrua.runner import live, trial


@pytest.fixture
def external_profile(tmp_path):
    source = tmp_path / "native"
    source.mkdir()
    (source / "config.toml").write_text('model = "fixture"\n')
    (source / "credentials").mkdir()
    (source / "credentials" / "account.json").write_text('{"token":"fixture"}')
    (source / "oauth").mkdir()
    (source / "history.json").write_text('{"private":"do not copy"}')
    plugin = tmp_path / "profile_agent.py"
    plugin.write_text('''
import json, shutil
from openrua.agents import Agent, PreparedProfile
class ProfileAgent(Agent):
    def prepare_profile(self, source, dest, *, require_credentials=True):
        if require_credentials and not (source / "credentials").is_dir():
            raise RuntimeError("native credentials missing")
        dest.mkdir(parents=True, mode=0o700)
        shutil.copy2(source / "config.toml", dest / "config.toml")
        mounts = (f"{dest}:/native",)
        if require_credentials:
            mounts += (f"{source / 'credentials'}:/native/credentials",
                       f"{source / 'oauth'}:/native/oauth")
        def read_secrets():
            if not require_credentials:
                return []
            return [json.loads((source / "credentials/account.json").read_text())["token"]]
        return PreparedProfile(dest, mounts, read_secrets)
    def launch_argv(self, sandbox, prompt, model, max_turns, proxy, **kwargs):
        return [*self.exec_argv(sandbox), "fixture", prompt]
HOOKS = ProfileAgent
''')
    manifest = tmp_path / "profile-agent.yaml"
    manifest.write_text("name: profile-agent\ndefault_model: fixture\nentry_point: ./profile_agent.py\n")
    return agents.get(str(manifest)), source


def test_external_plugin_copies_selected_config_and_shares_credential_directories(external_profile, tmp_path):
    agent, source = external_profile
    prepared = agent.prepare_profile(source, tmp_path / "session")
    assert "prepare_profile" in agent.capabilities
    assert (prepared.directory / "config.toml").read_bytes() == (source / "config.toml").read_bytes()
    assert sorted(p.name for p in prepared.directory.iterdir()) == ["config.toml"]
    assert prepared.mounts[1:] == (
        f"{source / 'credentials'}:/native/credentials", f"{source / 'oauth'}:/native/oauth")
    assert prepared.read_secrets() == ["fixture"]
    supplied_token = agent.prepare_profile(source, tmp_path / "token-session", require_credentials=False)
    assert supplied_token.mounts == (f"{supplied_token.directory}:/native",)
    assert supplied_token.read_secrets() == []
    assert (source / "history.json").read_text() == '{"private":"do not copy"}'


@pytest.mark.parametrize("name", ["claude-code", "codex"])
def test_existing_plugins_keep_their_shared_login_contract(tmp_path, name):
    agent = agents.get(name)
    source = tmp_path / "native"
    source.mkdir()
    (source / agent.credentials.filename).write_text('{"token":"fixture"}')
    (source / "settings.json").write_text("{}")
    profile = agent.prepare_profile(source, tmp_path / "session")
    assert profile.mounts == agent.sandbox_mounts(profile.directory, source / agent.credentials.filename)
    assert (profile.directory / "settings.json").exists()
    assert not (profile.directory / agent.credentials.filename).exists()


def test_profile_preparation_does_not_destroy_source_or_previous_files_on_missing_login(tmp_path):
    agent = agents.get("claude-code")
    source = tmp_path / "native"
    source.mkdir()
    original = source / "settings.json"
    original.write_text("keep")
    for dest in (source, tmp_path):
        with pytest.raises(ValueError, match="separate"):
            agent.prepare_profile(source, dest)
        assert original.read_text() == "keep"
    dest = tmp_path / "session"
    dest.mkdir()
    saved = dest / "retained.json"
    saved.write_text("keep")
    with pytest.raises(RuntimeError, match="credentials missing"):
        agent.prepare_profile(source, dest)
    assert saved.read_text() == "keep"
    link = tmp_path / "alias"
    link.symlink_to(dest)
    with pytest.raises(ValueError, match="separate"):
        agent.prepare_profile(source, link, require_credentials=False)
    assert saved.read_text() == "keep"


def test_live_and_trial_consume_the_external_profile_hook(external_profile, tmp_path, monkeypatch):
    agent, source = external_profile
    cfg = {"machine": {"backend": {"kind": "real"}, "robot": {"model": "fixture"}},
           "agent": {"name": "profile-agent", "credentials_dir": str(source)}, "protocol": {}}
    home = tmp_path / "home"
    machine = SimpleNamespace(shutdown=lambda: None)
    seen = []

    def bring_up(cfg, workdir, sim, sandbox, suite, task_id, network, proxy, mounts, *a, **kw):
        seen.append(mounts)
        return workdir / "config.yaml", machine, 0

    monkeypatch.setattr(agents, "get", lambda *a, **kw: agent)
    monkeypatch.setattr(live, "compose", lambda *a, **kw: SimpleNamespace(
        cfg=cfg, suite=None, task_id=None, robot="fixture", simulator=None, benchmark=None))
    for module in (live, trial):
        monkeypatch.setattr(module, "ensure_internal_network", lambda: "fixture")
        monkeypatch.setattr(module, "ensure_proxy", lambda _: "fixture")
        monkeypatch.setattr(module, "bring_up", bring_up)
        monkeypatch.setattr(module, "sandbox_down", lambda _: None)
    robot = live.open_robot(live.RobotRequest(home=home, name="live", task="inspect"))
    robot.power_off()
    assert seen[0][1:] == (f"{source / 'credentials'}:/native/credentials", f"{source / 'oauth'}:/native/oauth")

    monkeypatch.setattr(trial, "run_preflight", lambda *a: {"ok": True, "checks": [], "failed": []})
    monkeypatch.setattr(trial, "_sandbox_version", lambda *a: "fixture")
    collected = []
    monkeypatch.setattr(trial.record, "finalize_trial", lambda *a, **kw: collected.append(kw["profile_dir"]))
    token = tmp_path / "token.env"
    token.write_text("TOKEN=explicit-fixture-token-value\n")
    result = trial.run_trial(cfg, tmp_path / "config.yaml", tmp_path / "run", "suite", 0, 0,
                             lambda ctx: {"operator": "fixture"}, 1.0,
                             token_file=str(token), task="inspect", home=home)
    assert result["anomaly"] is None
    assert seen[1] == (f"{collected[0]}:/native",)
    assert (source / "credentials" / "account.json").exists()

    captured = []
    monkeypatch.setattr(trial.record, "finalize_trial", lambda directory, adapter, secrets, **kw:
                        captured.extend(secrets))
    def operate(ctx):
        (source / "credentials/account.json").write_text('{"token":"rotated-fixture-token"}')
        return {"operator": "fixture"}
    result = trial.run_trial(cfg, tmp_path / "config.yaml", tmp_path / "native-run", "suite", 0, 0,
                             operate, 1.0, task="inspect", home=home)
    assert result["anomaly"] is None
    assert "fixture" in captured and "rotated-fixture-token" in captured
