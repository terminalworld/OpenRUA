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
    argv = launch_argv("r", "ros2 launch x y.py", "vendor/driver:foxy", ros_domain=37)
    assert argv[argv.index("--env") + 1] == "ROS_DOMAIN_ID=37"
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


def test_real_readiness_does_not_accept_a_probe_that_finishes_after_deadline():
    import sys
    from openrua.robot.real.up import RealHandle
    handle = RealHandle('late-probe', None, [sys.executable, '-c',
        'import sys,time; print("waiting for graph", file=sys.stderr, flush=True); time.sleep(2)'])
    with pytest.raises(TimeoutError, match='waiting for graph'):
        handle.wait_ready(timeout_s=.2)


def test_real_readiness_failure_does_not_sleep_past_deadline():
    import sys
    import time
    from openrua.robot.real.up import RealHandle
    handle = RealHandle('missing-graph', None, [sys.executable, '-c',
        'import sys; sys.stderr.write("graph unavailable"); sys.exit(1)'])
    started = time.monotonic()
    with pytest.raises(TimeoutError, match='graph unavailable'):
        handle.wait_ready(timeout_s=.2)
    assert time.monotonic() - started < 2, 'polling delay exceeded the readiness budget'


def test_real_readiness_timeout_stops_probe_children(tmp_path):
    import sys
    import time
    from openrua.robot.real.up import RealHandle
    marker = tmp_path / 'probe-child-finished'
    child = 'import time; from pathlib import Path; time.sleep(1); Path(' + repr(str(marker)) + ').touch()'
    parent = 'import subprocess,sys,time; subprocess.Popen([sys.executable, "-c", ' + repr(child) + ']); time.sleep(2)'
    handle = RealHandle('probe-pipeline', None, [sys.executable, '-c', parent])
    with pytest.raises(TimeoutError, match='graph not visible'):
        handle.wait_ready(timeout_s=.2)
    time.sleep(1.1)
    assert not marker.exists(), 'a child of the timed-out probe kept running'


@pytest.mark.parametrize("domain", [0, 37])
def test_owned_real_driver_uses_explicit_domain_not_host_default(tmp_path, monkeypatch, domain):
    import shlex
    import sys
    monkeypatch.setenv("ROS_DOMAIN_ID", "12")
    log = tmp_path / "driver.log"
    command = shlex.join([sys.executable, "-c", "import os; print(os.environ.get('ROS_DOMAIN_ID'), flush=True)"])
    handle = robot.up({"kind": "real", "launch": command}, name="domain-probe",
                      config_path="", task_suite="", task_id=0, log_path=log,
                      ros_domain=domain)
    try:
        assert handle.proc.wait(timeout=5) == 0
        assert log.read_text().strip() == str(domain)
    finally:
        handle.shutdown()
