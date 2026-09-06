"""Robot package host-side contract: front door + ground-verb guards.

Onboard itself is container-side (rclpy) and is exercised by smokes;
what belongs here is the package shape (same as sandbox/proxy/agents)
and the data-consuming guards of the ground verbs.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1] / "robocli" / "robot"


def test_package_front_door():
    h = subprocess.run([sys.executable, "-m", "robocli.robot", "--help"],
                       capture_output=True, text=True)
    assert h.returncode == 0
    for verb in ("build", "up", "down"):
        assert verb in h.stdout


def test_front_door_rejects_unknown_verb():
    h = subprocess.run([sys.executable, "-m", "robocli.robot", "fly"],
                       capture_output=True, text=True)
    assert h.returncode == 2 and "unknown verb" in h.stderr


def test_blueprints_ship_with_the_package():
    assert list(PKG.glob("*.Dockerfile")), "no body blueprints in robot/"


def test_up_requires_rendered_peers_with_static_peer():
    # The conductor renders the peers profile ONCE and hands it in; a
    # static_peer without it would silently lose Humble DDS peering.
    from robocli.robot.up import up

    with pytest.raises(ValueError, match="peers_xml"):
        up(name="x", image="img", config_path="c.yaml", task_suite="s",
           task_id=0, simulator="/s", code_root="/r", static_peer="peer")
