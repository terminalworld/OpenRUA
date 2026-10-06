"""API billing requires a deliberate selection, separate from native profiles."""

import json
from types import SimpleNamespace

import pytest

from openrua import agents, config
from openrua.cli import build_parser
from openrua.cli.commands import config as config_command
from openrua.config.schema import Authentication
from openrua.doctor.checks import check_login, run as doctor_run
from openrua.errors import UsageError
from openrua.runner import live, live_state


def configure(home, *flags):
    args = build_parser().parse_args(["--home", str(home), "config", "set", *flags])
    return config_command.run(args)


def test_explicit_selection_roundtrips_without_saving_key_contents(tmp_path, capsys):
    key_file = tmp_path / "key"
    key_file.write_text("synthetic-openrua-secret-value")
    configure(tmp_path, "--agent", "codex", "--auth", "api", "--api-key-file", str(key_file))
    user = config.load_user_config(tmp_path / "config.yaml")
    assert user.agents["codex"].auth.mode == "api"
    cfg = config.compose("panda", "robosuite", None, tmp_path).cfg
    assert cfg["agent"]["auth"] == {"mode": "api", "key_file": str(key_file)}
    report = doctor_run(home=tmp_path, checks=(check_login,))
    assert report.ok
    assert "API authentication explicitly selected" in report.render()
    assert key_file.read_text() not in report.render() + capsys.readouterr().out + (tmp_path / "config.yaml").read_text()
    configure(tmp_path, "--auth", "native")
    assert config.load_user_config(tmp_path / "config.yaml").agents["codex"].auth.key_file is None
    assert key_file.read_text() == "synthetic-openrua-secret-value"


def test_invalid_or_unsupported_selection_does_not_change_defaults(tmp_path):
    configure(tmp_path, "--agent", "codex", "--model", "chosen")
    original = (tmp_path / "config.yaml").read_bytes()
    key_file = tmp_path / "key"
    key_file.write_text("fixture")
    cases = [
        ("--auth", "api"),
        ("--api-key-file", str(key_file)),
        ("--auth", "native", "--api-key-file", str(key_file)),
        ("--auth", "api", "--api-key-file", str(tmp_path / "missing")),
        ("--agent", "claude-code", "--auth", "api", "--api-key-file", str(key_file)),
    ]
    for flags in cases:
        with pytest.raises((ValueError, UsageError)):
            configure(tmp_path, *flags)
        assert (tmp_path / "config.yaml").read_bytes() == original
    for data in ({"mode": "api"}, {"key_file": str(key_file)}, {"mode": "auto"}):
        with pytest.raises(ValueError):
            Authentication.model_validate(data)


def test_native_default_never_probes_key_files(tmp_path, monkeypatch):
    agent = agents.get("codex")
    from openrua.agents import credentials
    monkeypatch.setattr(credentials, "read_api_key", lambda _: pytest.fail("must not discover keys"))
    assert agents.api_key_file(agent, None) is None
    assert agents.api_key_file(agent, {"mode": "native"}) is None


def test_api_profile_is_isolated_and_rejects_destructive_paths(tmp_path):
    agent = agents.get("codex")
    key_file = tmp_path / "key"
    key_file.write_text("synthetic-openrua-secret-value\n")
    with pytest.raises(ValueError, match="separate"):
        agent.prepare_api_profile(key_file, tmp_path)
    profile = agent.prepare_api_profile(key_file, tmp_path / "profile")
    assert profile.mounts == (f"{profile.directory}:/codex-home",)
    auth = profile.directory / "auth.json"
    assert json.loads(auth.read_text()) == {"auth_mode": "apikey", "OPENAI_API_KEY": key_file.read_text().strip()}
    assert auth.stat().st_mode & 0o777 == 0o600
    assert profile.directory.stat().st_mode & 0o777 == 0o700
    assert 'forced_login_method = "api"' in (profile.directory / "config.toml").read_text()
    assert profile.read_secrets() == [key_file.read_text().strip()]
    key_file.write_text("\n")
    with pytest.raises(ValueError, match="nonempty"):
        agent.prepare_api_profile(key_file, profile.directory)
    assert auth.exists()  # invalid replacement must not clear an existing profile


def test_live_api_start_skips_native_discovery_and_records_only_mode(tmp_path, monkeypatch):
    agent = agents.get("codex")
    key_file = tmp_path / "key"
    key_file.write_text("synthetic-secret-for-live-profile")
    cfg = {"machine": {"backend": {"kind": "real"}, "robot": {"model": "fixture"}},
           "agent": {"name": "codex", "auth": {"mode": "api", "key_file": str(key_file)}}}
    monkeypatch.setattr(live, "compose", lambda *a, **kw: SimpleNamespace(
        cfg=cfg, suite=None, task_id=None, robot="fixture", simulator=None, benchmark=None))
    monkeypatch.setattr(live.agents, "resolve_profile", lambda *a, **kw: pytest.fail("must not discover native auth"))
    monkeypatch.setattr(live, "ensure_internal_network", lambda: "fixture")
    monkeypatch.setattr(live, "ensure_proxy", lambda _: "fixture")
    mounts = []
    def bring_up(*args, **kwargs):
        mounts.extend(args[8])
        return None, SimpleNamespace(shutdown=lambda: None), 0
    monkeypatch.setattr(live, "bring_up", bring_up)
    monkeypatch.setattr(live, "sandbox_down", lambda _: None)
    home = tmp_path / "home"
    robot = live.open_robot(live.RobotRequest(home=home, name="fixture", task="inspect"))
    assert mounts == [f"{home / 'sandboxes/fixture/profile'}:/codex-home"]
    facts = live_state.load("fixture", home)
    assert facts["auth_mode"] == "api"
    assert key_file.read_text() not in json.dumps(facts)
    robot.power_off()
    assert key_file.exists()
    cfg["agent"]["auth"]["key_file"] = str(tmp_path / "missing")
    with pytest.raises(UsageError):
        live.open_robot(live.RobotRequest(home=home, name="invalid"))
    assert not (home / "sandboxes/invalid").exists()
