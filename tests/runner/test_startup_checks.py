"""A real robot's startup checks: run before every start, all of them, each failure with its fix."""

import pytest

from openrua.config import compose
from openrua.doctor import checks as doctor_checks
from openrua.errors import UnavailableError
from openrua.robot.real.up import failed_checks
from openrua.runner.live import RobotRequest, open_robot

ROBOT = """machine:
  backend:
    kind: real
    ros_distro: jazzy
    launch: python3 driver/up.py
    discovery:
      network: host
    startup_checks:
      - name: the controller answers
        run: test -e controller-up
        fix: power the controller on
      - name: the gripper's adapter is plugged in
        run: "true"
        fix: plug the USB adapter in
  robot:
    model: Fake arm
    description: two joints
  cameras:
    list: []
"""


def test_every_check_runs_in_the_robot_files_directory(tmp_path):
    checks = [{"name": "here", "run": "test -e marker", "fix": "make it"},
              {"name": "never", "run": "false", "fix": "it cannot"},
              {"name": "slow", "run": "sleep 5", "fix": "wait less"}]
    assert [c["name"] for c in failed_checks(checks, str(tmp_path), timeout_s=1)] \
        == ["here", "never", "slow"]
    (tmp_path / "marker").touch()
    assert [c["name"] for c in failed_checks(checks, str(tmp_path), timeout_s=1)] == ["never", "slow"]


def test_a_robot_that_is_not_ready_does_not_start_and_says_what_to_do(tmp_path):
    (tmp_path / "arm").mkdir()
    file = tmp_path / "arm" / "robot.yaml"
    file.write_text(ROBOT)
    with pytest.raises(UnavailableError, match="the controller answers") as caught:
        open_robot(RobotRequest(home=tmp_path / "home", name="arm", robot=str(file)))
    assert "power the controller on" in caught.value.hint
    assert f"openrua connect --dir {tmp_path / 'arm'}" in caught.value.hint
    assert "adapter" not in str(caught.value)
    # Nothing was started or reserved.
    assert not (tmp_path / "home" / "sandboxes" / "arm").exists()


def test_doctor_shows_each_startup_check(tmp_path):
    (tmp_path / "arm").mkdir()
    file = tmp_path / "arm" / "robot.yaml"
    file.write_text(ROBOT)
    ctx = doctor_checks.Context(home=tmp_path, agents=[], cfg=compose(str(file), "", "", tmp_path).cfg,
                                robot=str(file))
    results = doctor_checks.check_robot_ready(ctx)
    assert [(r.label, r.severity) for r in results] == [
        ("the controller answers", "error"), ("the gripper's adapter is plugged in", "ok")]
    assert results[0].hint == "power the controller on"
    (tmp_path / "arm" / "controller-up").touch()
    assert all(r.severity == "ok" for r in doctor_checks.check_robot_ready(ctx))
