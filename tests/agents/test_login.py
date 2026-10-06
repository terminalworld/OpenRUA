"""Discovery and doctor consume a plugin's local login inspection."""

import pytest

from openrua import agents
from openrua.doctor.checks import Context, check_login


@pytest.fixture
def directory_agent(tmp_path):
    (tmp_path / "native.py").write_text('''
from openrua.agents.base import Agent, ProfileLogin
class Native(Agent):
    def inspect_login(self, source):
        records = source / "credentials"
        available = records.is_dir() and any(records.glob("*.json"))
        return ProfileLogin(available, "native credential directory checked")
    def login_hint(self, source):
        return "run native login"
HOOKS = Native
''')
    manifest = tmp_path / "native.yaml"
    manifest.write_text('''name: native
default_model: fixture
entry_point: ./native.py
credentials:
  dirname: native
  filename: unused-legacy-file
  config_env: NATIVE_PROFILE
  mount_point: /native
  native_dir: .native
''')
    return agents.get(str(manifest))


def test_directory_login_discovery_and_doctor_agree(directory_agent, tmp_path):
    agent = directory_agent
    source = tmp_path / ".native"
    alias = tmp_path / "openrua/credentials/native"
    ctx = Context(tmp_path, [agent], None, None, login_directories={agent: source})
    assert "inspect_login" in agent.capabilities
    assert check_login(ctx)[0].severity == "error"
    assert agents.resolve_profile(agent, None, alias, user_home=tmp_path,
                                  environment={}, link=True) == source
    assert not alias.exists()
    directory = source / "credentials"
    directory.mkdir(parents=True)
    record = directory / "account.json"
    record.write_text('{"token":"synthetic-secret-must-not-appear"}')
    original = record.read_bytes()
    assert check_login(ctx)[0].severity == "ok"
    assert "authentication and quota" in check_login(ctx)[0].detail
    assert "synthetic-secret" not in repr(check_login(ctx))
    assert not alias.exists()  # doctor never creates an alias
    assert agents.resolve_profile(agent, None, alias, user_home=tmp_path,
                                  environment={}, link=True) == alias
    assert alias.resolve() == source
    assert record.read_bytes() == original
    record.unlink()
    ctx.login_directories[agent] = alias
    assert check_login(ctx)[0].severity == "error"
    assert alias.is_symlink()  # missing credentials do not switch accounts


@pytest.mark.parametrize("name", ["claude-code", "codex"])
def test_empty_and_unreadable_credentials_are_not_aliased(tmp_path, monkeypatch, name):
    agent = agents.get(name)
    source = tmp_path / agent.credentials.native_dir
    source.mkdir()
    record = source / agent.credentials.filename
    alias = tmp_path / "alias"
    record.touch()
    assert not agent.inspect_login(source).available
    assert agents.resolve_profile(agent, None, alias, user_home=tmp_path,
                                  environment={}, link=True) == source
    assert not alias.exists()
    record.write_text("synthetic-credential")
    original_open = type(record).open
    def open_file(path, *args, **kwargs):
        if path == record:
            raise PermissionError("permission denied")
        return original_open(path, *args, **kwargs)
    monkeypatch.setattr(type(record), "open", open_file)
    check = agent.inspect_login(source)
    assert not check.available and "permission denied" in check.detail
    assert agents.resolve_profile(agent, None, alias, user_home=tmp_path,
                                  environment={}, link=True) == source
    assert not alias.exists()


def test_no_profile_is_not_reported_as_verified_login(tmp_path):
    agent = agents.Agent(name="fixture", default_model="fixture")
    assert agent.inspect_login(tmp_path) is None
    result = check_login(Context(tmp_path, [agent], None, None))[0]
    assert result.severity == "ok"
    assert "no profile login to check" in result.label


def test_custom_login_check_does_not_require_a_legacy_credential_file(directory_agent, tmp_path):
    agent = directory_agent
    agent.credentials = None
    ctx = Context(tmp_path, [agent], None, None, login_directories={agent: tmp_path})
    result = check_login(ctx)[0]
    assert result.severity == "error"
    assert result.hint == "run native login"
