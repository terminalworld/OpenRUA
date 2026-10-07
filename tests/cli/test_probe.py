"""Graph query failures must not produce a usable-looking profile draft."""

import subprocess
from types import SimpleNamespace

import pytest

from openrua.cli.commands import probe
from openrua.errors import UnavailableError


@pytest.mark.parametrize("command", ["ros2 topic list", "ros2 action list", "ros2 service list"])
@pytest.mark.parametrize("named", [False, True])
def test_query_failure_reports_native_error_without_a_draft(monkeypatch, capsys, command, named):
    stopped = []
    monkeypatch.setattr(probe.state, "load", lambda *args: {"sandbox": "existing-sandbox"})
    monkeypatch.setattr(probe, "sandbox_reachability", lambda *args: {})
    monkeypatch.setattr(probe, "sandbox_up", lambda *args, **kwargs: None)
    monkeypatch.setattr(probe, "sandbox_down", stopped.append)

    def execute(argv, **kwargs):
        status = 1 if argv[-1] == command else 0
        return subprocess.CompletedProcess(argv, status, "", "native discovery failed" if status else "")

    monkeypatch.setattr(probe.subprocess, "run", execute)
    args = SimpleNamespace(name="existing" if named else None, home=None, host=True,
                           static_peers=None, discovery_server=None,
                           image=None, distro="humble", ros_domain=7)
    with pytest.raises(UnavailableError) as caught:
        probe.run(args)
    assert command in str(caught.value)
    assert "native discovery failed" in str(caught.value)
    assert "docker info" in caught.value.hint
    assert capsys.readouterr().out == ""
    assert len(stopped) == (0 if named else 1)


def test_query_timeout_preserves_output_and_reports_a_fix(monkeypatch):
    def execute(argv, **kwargs):
        raise subprocess.TimeoutExpired(argv, kwargs["timeout"],
                                        output=b"partial output", stderr=b"discovery stalled")

    monkeypatch.setattr(probe.subprocess, "run", execute)
    with pytest.raises(UnavailableError) as caught:
        probe._exec_in("robot-sandbox")("ros2 topic list")
    assert "120" in str(caught.value)
    assert "partial output" in str(caught.value)
    assert "discovery stalled" in str(caught.value)
    assert "docker info" in caught.value.hint


def test_optional_urdf_failure_still_allows_a_draft(monkeypatch):
    def execute(argv, **kwargs):
        if argv[-1] == "ros2 topic list":
            return subprocess.CompletedProcess(argv, 0, "/joint_states\n", "")
        if argv[-1] in ("ros2 action list", "ros2 service list"):
            return subprocess.CompletedProcess(argv, 0, "", "")
        return subprocess.CompletedProcess(argv, 1, "", "no robot description available")

    monkeypatch.setattr(probe.subprocess, "run", execute)
    graph = probe.read_graph(probe._exec_in("robot-sandbox"))
    assert graph == {"topics": ["/joint_states"], "actions": [], "services": [], "urdf": None}
    assert "TODO: no revolute joints" in probe.draft_profile(graph, {"network": "host"})


@pytest.mark.parametrize("distro", ["humble", "jazzy"])
@pytest.mark.parametrize("discovery", [
    {"host": True, "static_peers": None, "discovery_server": None},
    {"host": False, "static_peers": "192.0.2.20,192.0.2.21", "discovery_server": None},
    {"host": False, "static_peers": None, "discovery_server": "192.0.2.20:11811"},
])
def test_standalone_probe_needs_no_model_proxy_and_preserves_distro(
        monkeypatch, capsys, distro, discovery):
    import yaml
    from openrua.runner import bringup

    def unexpected_proxy(*args):
        raise AssertionError("Robot graph probing must not depend on a model proxy")

    calls = []
    stopped = []
    monkeypatch.setattr(bringup, "proxy_url_from_network", unexpected_proxy)
    monkeypatch.setattr(probe, "sandbox_up", lambda *args, **kwargs: calls.append(kwargs))
    monkeypatch.setattr(probe, "sandbox_down", stopped.append)
    monkeypatch.setattr(probe, "read_graph", lambda run: {
        "topics": [], "actions": [], "services": [], "urdf": None})
    args = SimpleNamespace(name=None, home=None, image=None, distro=distro,
                           ros_domain=7, **discovery)
    assert probe.run(args) == 0
    backend = yaml.safe_load(capsys.readouterr().out)["machine"]["backend"]
    assert backend["ros_distro"] == distro
    assert backend["discovery"] == probe._discovery(args)
    assert calls[0]["image"] == f"openrua-sandbox-{distro}"
    assert calls[0]["ros_domain"] == 7
    assert calls[0]["internet"] == "none"
    assert calls[0]["network"] == "host"
    if discovery["static_peers"]:
        assert calls[0]["static_peer"] == discovery["static_peers"]
        assert "<address>192.0.2.20</address>" in calls[0]["peers_xml"]
    if discovery["discovery_server"]:
        assert calls[0]["env"] == ("ROS_DISCOVERY_SERVER=192.0.2.20:11811",)
    assert stopped == [calls[0]["name"]]
