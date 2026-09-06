"""Robot package host-side contract: dispatch and the sim verbs' guards.

The bridge itself is container-side (rclpy) and is exercised by smokes;
what belongs here is the package shape and the data-consuming guards.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from robocli import robot

PKG = Path(__file__).resolve().parents[1] / "robocli" / "robot"


def test_dockerfiles_ship_with_the_package():
    assert sorted(p.name for p in (PKG / "sim").glob("Dockerfile.*")) == [
        "Dockerfile.humble", "Dockerfile.jazzy"]


def test_up_dispatches_on_backend_kind(tmp_path):
    from robocli.errors import ConfigError
    with pytest.raises(ConfigError, match="kind"):
        robot.up({"kind": "hover"}, name="x", config_path="c.yaml", task_suite="s",
                 task_id=0, log_path=tmp_path / "log")
    with pytest.raises(ValueError, match="venv"):
        robot.up({"kind": "sim"}, name="x", config_path="c.yaml", task_suite="s",
                 task_id=0, log_path=tmp_path / "log")
    # a real robot with nothing to launch and nothing to probe is ready at once
    h = robot.up({"kind": "real", "discovery": {"network": "host"}}, name="r",
                 config_path="c.yaml", task_suite="s", task_id=0, log_path=tmp_path / "log")
    h.wait_ready()
    assert h.rpc({"cmd": "success"}) == {"ok": True, "not_applicable": True, "cmd": "success"}
    h.shutdown()
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
    from robocli.robot.sim.up import up

    with pytest.raises(ValueError, match="peers_xml"):
        up(name="x", image="img", config_path="c.yaml", task_suite="s",
           task_id=0, simulator="/s", code_root="/r", static_peer="peer")
