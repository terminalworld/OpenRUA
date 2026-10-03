"""Robot package host-side contract: dispatch and the sim verbs' guards.

The bridge itself is container-side (rclpy) and is exercised by smokes;
what belongs here is the package shape and the data-consuming guards.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from openrua import robot

PKG = Path(__file__).resolve().parents[2] / "openrua" / "robot"


def test_dockerfiles_ship_with_the_package():
    assert sorted(p.name for p in (PKG / "sim").glob("Dockerfile.*")) == [
        "Dockerfile.humble", "Dockerfile.jazzy"]


def test_up_dispatches_on_backend_kind(tmp_path):
    from openrua.errors import ConfigError
    with pytest.raises(ConfigError, match="kind"):
        robot.up({"kind": "hover"}, name="x", config_path="c.yaml", task_suite="s",
                 task_id=0, log_path=tmp_path / "log")
    # a real robot with nothing to launch and nothing to probe is ready at once
    h = robot.up({"kind": "real", "discovery": {"network": "host"}}, name="r",
                 config_path="c.yaml", task_suite="s", task_id=0, log_path=tmp_path / "log")
    h.wait_ready()
    assert h.rpc({"cmd": "success"}) == {"ok": True, "not_applicable": True, "cmd": "success"}
    h.shutdown()
    with pytest.raises(ConfigError, match="owning handle"):
        robot.down("r", "real")


def test_real_probe_failure_is_a_timeout(tmp_path):
    h = robot.up({"kind": "real", "discovery": {"network": "host"}}, name="r2",
                 config_path="c.yaml", task_suite="s", task_id=0, log_path=tmp_path / "log",
                 probe_argv=["false"])
    with pytest.raises(TimeoutError, match="graph not visible"):
        h.wait_ready(timeout_s=0)


def test_sim_up_requires_rendered_peers_with_static_peer():
    # The runner renders the peers profile once and hands it in; a
    # static_peer without it would silently lose Humble DDS peering.
    from openrua.robot.sim.up import up

    with pytest.raises(ValueError, match="peers_xml"):
        up(name="x", image="img", config_path="c.yaml", task_suite="s",
           task_id=0, static_peer="peer")


def test_real_launch_runs_in_the_driver_image_when_one_is_named():
    from openrua.robot.real.up import launch_argv
    assert launch_argv("r", "ros2 launch x y.py", None) == ["bash", "-lc", "ros2 launch x y.py"]
    argv = launch_argv("r", "ros2 launch x y.py", "vendor/driver:foxy")
    assert argv[:2] == ["docker", "run"] and "--network" in argv and "host" in argv
    assert "r-driver" in argv and argv[-3:] == ["bash", "-lc", "ros2 launch x y.py"]


def test_real_launch_is_remembered_and_stopped(tmp_path):
    h = robot.up({"kind": "real", "discovery": {"network": "host"}, "launch": "sleep 30"},
                 name="r3", config_path="c.yaml", task_suite="s", task_id=0,
                 log_path=tmp_path / "log")
    assert h.proc is not None and h.proc.poll() is None
    h.shutdown()
    assert h.proc.poll() is not None


def test_real_driver_container_cleanup_reports_errors_and_can_retry(monkeypatch):
    import importlib
    from types import SimpleNamespace
    from openrua.robot.real.up import RealHandle
    module = importlib.import_module("openrua.robot.real.down")
    replies = iter([
        SimpleNamespace(returncode=1, stderr="daemon unavailable"),
        SimpleNamespace(returncode=0, stderr=""),
    ])
    monkeypatch.setattr(module.subprocess, "run", lambda *a, **k: next(replies))
    handle = RealHandle("real", None, None, "real-driver")
    with pytest.raises(RuntimeError, match="daemon unavailable"):
        handle.shutdown()
    handle.shutdown()


def test_real_handles_own_their_processes_even_with_the_same_display_name(tmp_path):
    handles = [robot.up({'kind': 'real', 'launch': 'sleep 120'}, name='same',
                        config_path='', task_suite='', task_id=0, log_path=tmp_path / f'log-{i}')
               for i in range(2)]
    try:
        handles[0].shutdown()
        assert handles[0].proc.poll() is not None
        assert handles[1].proc.poll() is None
    finally:
        for handle in handles:
            handle.shutdown()
