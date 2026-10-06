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
    monkeypatch.setattr(live, "ensure_proxy", lambda _: "proxy")
    adapter = SimpleNamespace(name="fake", default_model="model", default_options={},
                              prepare_profile=lambda source, dest: PreparedProfile(dest, ()))
    monkeypatch.setattr(live.agents, "get", lambda *a, **kw: adapter)
    machine = SimpleNamespace(shutdown=lambda: calls.append("robot stopped"))
    monkeypatch.setattr(live, "bring_up", lambda *a, **kw: (None, machine, 0))
    monkeypatch.setattr(live, "sandbox_down", lambda _: calls.append("sandbox stopped"))
    return live.RobotRequest(home=tmp_path, name="robot", task="inspect"), calls, machine


def test_direct_resource_api_needs_no_cli_and_keeps_workspace(resources):
    request, calls, _ = resources
    robot = live.open_robot(request)
    workspace = request.home / "sandboxes/robot/workspace/workspace"
    workspace.mkdir()
    (workspace / "notes.txt").write_text("observed state")
    robot.power_off()
    assert calls == ["sandbox stopped", "robot stopped"]
    assert (workspace / "notes.txt").read_text() == "observed state"
    assert live_state.load(request.name, request.home, require_running=False)["status"] == "stopped"


def test_failed_state_save_closes_started_resources(resources, monkeypatch):
    request, calls, _ = resources
    def fail(*a, **kw):
        raise OSError("disk full")
    monkeypatch.setattr(live_state, "save", fail)
    with pytest.raises(OSError, match="disk full"):
        live.open_robot(request)
    assert calls == ["sandbox stopped", "robot stopped"]


def test_stop_attempts_robot_even_when_sandbox_stop_fails(resources, monkeypatch):
    request, calls, _ = resources
    robot = live.open_robot(request)
    def fail(_):
        calls.append("sandbox failed")
        raise RuntimeError("daemon unavailable")
    monkeypatch.setattr(live, "sandbox_down", fail)
    with pytest.raises(UnavailableError, match="daemon unavailable"):
        robot.power_off()
    assert calls == ["sandbox failed", "robot stopped"]
    assert live_state.load(request.name, request.home)["status"] == "running"
    monkeypatch.setattr(live, "sandbox_down", lambda _: None)
    robot.power_off()
    assert live_state.load(request.name, request.home, require_running=False)["status"] == "stopped"


def test_shutdown_reports_both_errors(monkeypatch):
    def fail_sandbox(_):
        raise RuntimeError("sandbox error")
    def fail_robot():
        raise RuntimeError("driver error")
    monkeypatch.setattr(live, "sandbox_down", fail_sandbox)
    with pytest.raises(UnavailableError, match="sandbox: sandbox error; robot: driver error"):
        live.stop_resources("box", fail_robot)
