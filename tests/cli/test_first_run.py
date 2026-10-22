"""A computer that is not ready yet: Docker set up with the person, and the
agent signed in with its own sign-in, run in the sandbox image."""

import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from openrua import agents
from openrua.cli import docker_setup
from openrua.cli.commands import login
from openrua.errors import AuthError, UnavailableError

DENIED = ("permission denied while trying to connect to the Docker daemon socket at "
          "unix:///var/run/docker.sock")
STOPPED = "Cannot connect to the Docker daemon at unix:///var/run/docker.sock. Is the docker daemon running?"


@pytest.fixture
def docker_on_path(monkeypatch):
    real = docker_setup.shutil.which
    monkeypatch.setattr(docker_setup.shutil, "which", lambda name: f"/usr/bin/{name}"
                        if name in ("docker", "curl", "sg") else real(name))


def test_the_state_names_what_is_missing(monkeypatch, docker_on_path):
    assert docker_setup.state(lambda: "") == ("ready", "")
    assert docker_setup.state(lambda: DENIED)[0] == "no access"
    assert docker_setup.state(lambda: STOPPED)[0] == "stopped"
    assert docker_setup.state(lambda: "something else")[0] == "other"
    monkeypatch.setattr(docker_setup.shutil, "which", lambda name: None)
    assert docker_setup.state(lambda: "") == ("missing", "")


def test_ready_docker_asks_nothing(docker_on_path):
    docker_setup.ensure(ask=lambda q: pytest.fail("asked"), engine_error=lambda: "")


def test_missing_docker_is_installed_on_a_yes_and_the_start_goes_on_in_the_group(monkeypatch):
    monkeypatch.setattr(docker_setup.shutil, "which", lambda name: None if name == "docker" else f"/usr/bin/{name}")
    monkeypatch.setattr(docker_setup.getpass, "getuser", lambda: "ana")
    ran, restarted = [], []
    docker_setup.ensure(ask=lambda q: "", run=lambda step: ran.append(step) or SimpleNamespace(returncode=0),
                        engine_error=lambda: "", restart=lambda: restarted.append(True)
                        or monkeypatch.setattr(docker_setup.shutil, "which", lambda n: f"/usr/bin/{n}"))
    assert [step[:2] for step in ran] == [["curl", "-fsSL"], ["sudo", "sh"], ["sudo", "usermod"]]
    assert ran[0][2] == "https://get.docker.com" and ran[2] == ["sudo", "usermod", "-aG", "docker", "ana"]
    assert restarted == [True]


def test_no_access_joins_the_group_and_a_stopped_service_is_started(monkeypatch, docker_on_path):
    monkeypatch.setattr(docker_setup.getpass, "getuser", lambda: "ana")
    ran = []
    run = lambda step: ran.append(step) or SimpleNamespace(returncode=0)
    errors = iter([DENIED, ""])
    docker_setup.ensure(ask=lambda q: "y", run=run, engine_error=lambda: next(errors),
                        restart=lambda: None)
    assert ran == [["sudo", "usermod", "-aG", "docker", "ana"]]
    ran.clear()
    errors = iter([STOPPED, ""])
    docker_setup.ensure(ask=lambda q: "y", run=run, engine_error=lambda: next(errors))
    assert ran == [["sudo", "systemctl", "enable", "--now", "docker"]]


def test_declining_or_a_failed_step_says_where_the_guide_is(monkeypatch, docker_on_path):
    with pytest.raises(UnavailableError, match="not running") as caught:
        docker_setup.ensure(ask=lambda q: "n", engine_error=lambda: STOPPED)
    assert "docs.docker.com" in caught.value.hint
    with pytest.raises(UnavailableError, match="systemctl"):
        docker_setup.ensure(ask=lambda q: "", run=lambda step: SimpleNamespace(returncode=1),
                            engine_error=lambda: STOPPED)


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, "home", lambda: tmp_path / "user")
    for env in ("CLAUDE_CONFIG_DIR", "CODEX_HOME"):
        monkeypatch.delenv(env, raising=False)
    return tmp_path / "openrua"


def test_each_agent_names_its_own_sign_in():
    assert agents.get("claude-code").credentials.login == ("claude", "auth", "login")
    assert agents.get("codex").credentials.login == ("codex", "login", "--device-auth")


def test_sign_in_runs_the_agents_own_command_in_the_sandbox_image(home, monkeypatch):
    adapter = agents.get("claude-code")
    where = Path.home() / ".claude"
    monkeypatch.setattr(login, "image_with", lambda name: "openrua-sandbox-jazzy:latest")
    ran = []
    def run(argv, **kw):
        ran.append(argv)
        (where / ".credentials.json").write_text("{}")   # what the CLI writes when it is done
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(login.subprocess, "run", run)
    login.sign_in(home, adapter, where)
    argv = ran[0]
    assert argv[-4:] == ["openrua-sandbox-jazzy:latest", "claude", "auth", "login"]
    assert f"{where.resolve()}:/claude-config" in argv and "CLAUDE_CONFIG_DIR=/claude-config" in argv
    assert "-it" in argv and f"{__import__('os').getuid()}:{__import__('os').getgid()}" in argv


def test_an_unfinished_sign_in_and_no_image_yet(home, monkeypatch):
    adapter = agents.get("codex")
    built = []
    monkeypatch.setattr(login, "image_with", lambda name: None)
    from openrua.cli.commands import build
    monkeypatch.setattr(build, "_sandbox", lambda agent, *a: built.append(agent) or ("img", "sha"))
    monkeypatch.setattr(login.subprocess, "run", lambda argv, **kw: SimpleNamespace(returncode=1))
    with pytest.raises(AuthError, match="did not finish") as caught:
        login.sign_in(home, adapter, Path.home() / ".codex")
    assert built == [["codex"]] and caught.value.hint == "run openrua login codex to try again"


def test_ensure_offers_the_sign_in_only_when_it_is_needed(home, monkeypatch):
    signed = []
    monkeypatch.setattr(login, "sign_in", lambda home, adapter, where: signed.append((adapter.name, where)))
    login.ensure(home, "claude-code", "api", ask=lambda q: pytest.fail("asked"))
    with pytest.raises(AuthError) as caught:
        login.ensure(home, "claude-code", ask=lambda q: "n")
    assert caught.value.hint == "run openrua login claude-code"
    login.ensure(home, "codex", ask=lambda q: "")
    assert signed == [("codex", Path.home() / ".codex")]
    (Path.home() / ".claude").mkdir(parents=True)
    (Path.home() / ".claude" / ".credentials.json").write_text("{}")
    login.ensure(home, "claude-code", ask=lambda q: pytest.fail("asked when signed in"))
