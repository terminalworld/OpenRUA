"""Interactive resources have one owner and retain work across stop failures."""

from types import SimpleNamespace

import pytest

from openrua.errors import UnavailableError
from openrua.agents import PreparedProfile
from openrua.runner import live, live_state


@pytest.fixture
def resources(monkeypatch, tmp_path):
    calls = []
    cfg = {"machine": {"backend": {"kind": "real"}, "robot": {"model": "test"}},
           "agent": {"name": "fake"}}
    monkeypatch.setattr(live, "compose", lambda *a, **kw: SimpleNamespace(
        cfg=cfg, suite=None, task_id=None, robot="test", simulator=None, benchmark=None))
    monkeypatch.setattr(live, "normalize_arms", lambda _: None)
    monkeypatch.setattr(live, "ensure_internal_network", lambda: "network")
    monkeypatch.setattr(live, "ensure_proxy", lambda network, whitelist:
                        calls.append(("proxy", whitelist)) or "http://openrua-proxy-ab12cd34:8888")
    adapter = SimpleNamespace(name="fake", title="Fake", default_model="model", default_options={},
                              whitelist=("^api\\.fake\\.com$",), install="",
                              prepare_profile=lambda source, dest, limits=None:
                                  calls.append(("profile", limits)) or PreparedProfile(dest, ()))
    monkeypatch.setattr(live.agents, "get", lambda *a, **kw: adapter)
    machine = SimpleNamespace(shutdown=lambda: calls.append("robot stopped"))
    def bring_up(cfg, workdir, sim, sandbox, suite, task_id, network, internet, *a, **kw):
        calls.append(("internet", internet))
        return None, machine, 0
    monkeypatch.setattr(live, "bring_up", bring_up)
    monkeypatch.setattr(live, "sandbox_down", lambda _: calls.append("sandbox stopped"))
    return live.RobotRequest(home=tmp_path, name="robot", task="inspect"), calls, machine


def _stops(calls):
    return [c for c in calls if isinstance(c, str)]


def test_direct_resource_api_needs_no_cli_and_keeps_workspace(resources, proxy_address, agent_cli_ok):
    request, calls, _ = resources
    robot = live.open_robot(request)
    workspace = request.home / "sandboxes/robot/workspace/workspace"
    workspace.mkdir()
    (workspace / "notes.txt").write_text("observed state")
    robot.power_off()
    assert _stops(calls) == ["sandbox stopped", "robot stopped"]
    assert (workspace / "notes.txt").read_text() == "observed state"
    assert live_state.load(request.name, request.home, require_running=False)["status"] == "stopped"


def test_failed_state_save_closes_started_resources(resources, monkeypatch, proxy_address, agent_cli_ok):
    request, calls, _ = resources
    def fail(*a, **kw):
        raise OSError("disk full")
    monkeypatch.setattr(live_state, "save", fail)
    with pytest.raises(OSError, match="disk full"):
        live.open_robot(request)
    assert _stops(calls) == ["sandbox stopped", "robot stopped"]


def test_stop_attempts_robot_even_when_sandbox_stop_fails(resources, monkeypatch, proxy_address, agent_cli_ok):
    request, calls, _ = resources
    robot = live.open_robot(request)
    def fail(_):
        calls.append("sandbox failed")
        raise RuntimeError("daemon unavailable")
    monkeypatch.setattr(live, "sandbox_down", fail)
    with pytest.raises(UnavailableError, match="daemon unavailable"):
        robot.power_off()
    assert _stops(calls) == ["sandbox failed", "robot stopped"]
    assert live_state.load(request.name, request.home)["status"] == "running"
    monkeypatch.setattr(live, "sandbox_down", lambda _: None)
    robot.power_off()
    assert live_state.load(request.name, request.home, require_running=False)["status"] == "stopped"


def test_a_free_session_starts_no_proxy_and_goes_out_directly(resources, proxy_address,
                                                                agent_cli_ok):
    request, calls, _ = resources
    live.open_robot(request).power_off()
    assert not [c for c in calls if c[0] == "proxy"]
    assert ("internet", "open") in calls
    open_ = {"internet": "open", "web_search": True, "account_connectors": True}
    assert ("profile", open_) in calls
    facts = live_state.load(request.name, request.home, require_running=False)
    assert facts["proxy"] == "" and facts["limits"] == open_


def test_a_session_kept_to_its_model_service_gets_its_agents_proxy(resources, proxy_address,
                                                                    agent_cli_ok):
    request, calls, _ = resources
    from dataclasses import replace
    request = replace(request, limits={"internet": "model-only"})
    live.open_robot(request).power_off()
    assert ("proxy", "^api\\.fake\\.com$") in calls          # the agent's own hosts
    assert ("internet", "proxy:http://openrua-proxy-ab12cd34:8888") in calls
    facts = live_state.load(request.name, request.home, require_running=False)
    # a real robot's sandbox is on the host network: the agent gets the address
    assert facts["proxy"] == proxy_address
    assert facts["limits"] == {"internet": "model-only", "web_search": True,
                               "account_connectors": True}


def test_an_unknown_limit_is_refused(resources, proxy_address, agent_cli_ok):
    request, _, _ = resources
    from dataclasses import replace
    with pytest.raises(Exception, match="internet"):
        live.open_robot(replace(request, limits={"internet": "sometimes"}))


def test_provenance_names_the_agent_cli_and_the_limits(resources, monkeypatch, proxy_address):
    import json
    request, _, _ = resources
    monkeypatch.setattr(live, "check_agent_cli", lambda *a, **k: "2.1.306 (Claude Code)")
    live.open_robot(request).power_off()
    prov = json.loads((request.home / "sandboxes/robot/provenance.json").read_text())
    assert prov["agent_cli"] == {"name": "fake", "version": "2.1.306 (Claude Code)"}
    assert prov["limits"]["internet"] == "open"


def test_shutdown_reports_both_errors(monkeypatch):
    def fail_sandbox(_):
        raise RuntimeError("sandbox error")
    def fail_robot():
        raise RuntimeError("driver error")
    monkeypatch.setattr(live, "sandbox_down", fail_sandbox)
    with pytest.raises(UnavailableError, match="sandbox: sandbox error; robot: driver error"):
        live.stop_resources("box", fail_robot)


def test_the_agent_cli_check_reports_the_version_it_found(monkeypatch):
    # The session's provenance records the CLI the sandbox actually has.
    adapter = SimpleNamespace(name="fake", version=None, min_version="2.1.251",
                              version_probe=lambda: "HOME=/tmp fake --version",
                              sandbox_cli_check=lambda: ("floor", "test-version-floor"))
    codes = {"fake": 0}
    def run(argv, **kw):
        if argv[-1].endswith("--version"):
            return SimpleNamespace(returncode=0, stdout="2.1.306 (Fake)\n", stderr="")
        return SimpleNamespace(returncode=codes["fake"], stdout="", stderr="")
    monkeypatch.setattr(live.subprocess, "run", run)
    backend = {"kind": "sim", "sandbox_image": "openrua-sandbox-jazzy", "ros_distro": "jazzy"}
    assert live.check_agent_cli("box", adapter, backend, "panda") == "2.1.306 (Fake)"
    codes["fake"] = 1
    with pytest.raises(UnavailableError, match="2.1.306 \\(Fake\\); this session needs 2.1.251 or newer"):
        live.check_agent_cli("box", adapter, backend, "panda")
    no_check = SimpleNamespace(name="other", version_probe=lambda: None,
                               sandbox_cli_check=lambda: None)
    assert live.check_agent_cli("box", no_check, backend, "panda") is None


def test_missing_unpinned_codex_is_refused_before_starting_a_native_conversation(monkeypatch):
    from openrua import agents
    monkeypatch.setattr(live.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=127, stdout='', stderr='codex: not found'))
    backend = {'kind': 'real', 'sandbox_image': 'openrua-sandbox-humble', 'ros_distro': 'humble'}
    with pytest.raises(UnavailableError, match='installed, working CLI') as caught:
        live.check_agent_cli('box', agents.get('codex'), backend, 'robot.yaml')
    assert caught.value.hint == 'openrua build sandbox --distro humble --agent codex'


def test_interactive_environment_is_available_before_robot_startup_checks(resources, monkeypatch, agent_cli_ok):
    request, calls, _ = resources
    adapter = live.agents.get('fake')
    adapter.version = None
    monkeypatch.setattr(live, 'prepare_sandbox', lambda *a, **kw: (request.home / 'config.yaml', 7))
    def forbidden(_):
        pytest.fail('robot readiness must not prevent the interactive agent environment')
    monkeypatch.setattr(live, 'not_ready', forbidden)
    robot = live.open_robot(request, interactive=True)
    try:
        assert robot.connection is not None and robot.connection.machine is None
        assert robot.connection.request.robot == 'test'
        assert robot.connection.request.simulator == robot.connection.request.benchmark == ''
        assert not [call for call in calls if call == 'robot stopped']
        facts = live_state.load(request.name, request.home)
        assert facts['interactive'] and facts['status'] == 'running'
        assert any('/openrua-runtime' in mount for mount in facts['mounts'])
    finally:
        robot.power_off()
    assert _stops(calls) == ['sandbox stopped']
