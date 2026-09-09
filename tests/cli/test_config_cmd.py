"""``openrua config set/show`` and the defaults they feed."""

from __future__ import annotations

import pytest

from openrua.cli import build_parser
from openrua.config import compose
from openrua.errors import UsageError


def _run(argv):
    args = build_parser().parse_args(argv)
    return args.fn(args)


def test_set_writes_the_defaults_and_run_uses_them(tmp_path, capsys):
    home = str(tmp_path)
    assert _run(["--home", home, "config", "set", "--robot", "panda", "--sim", "robosuite",
                 "--agent", "claude-code"]) == 0
    assert (tmp_path / "config.yaml").read_text() == (
        "robot: panda\nsimulator: robosuite\nagent:\n  name: claude-code\n")
    c = compose(None, None, None, tmp_path)
    assert (c.robot, c.simulator, c.benchmark, c.suite) == ("panda", "robosuite", None, "Lift")
    _run(["--home", home, "config", "set", "--bench", "libero_pro", "--model", "m-1"])
    c = compose(None, None, None, tmp_path)
    assert (c.benchmark, c.suite) == ("libero_pro", "libero_goal_task")
    assert c.cfg["agent"]["model"] == "claude-opus-5"        # the benchmark's own wins
    c = compose(None, None, "capbench", tmp_path)               # a flag wins over the file
    assert c.benchmark == "capbench"
    _run(["--home", home, "config", "show"])
    out = capsys.readouterr().out
    assert "benchmark: libero_pro" in out and "agent:" in out


def test_set_needs_a_flag_and_keeps_other_keys(tmp_path):
    with pytest.raises(UsageError):
        _run(["--home", str(tmp_path), "config", "set"])
    assert not (tmp_path / "config.yaml").exists()
    (tmp_path / "config.yaml").write_text("sandbox: {run_args: ['--userns=keep-id']}\n")
    _run(["--home", str(tmp_path), "config", "set", "--robot", "panda"])
    text = (tmp_path / "config.yaml").read_text()
    assert "keep-id" in text and "robot: panda" in text


def test_a_defaulted_simulator_does_not_break_a_real_instance(tmp_path):
    lab = tmp_path / "lab.yaml"
    lab.write_text(
        "machine:\n  backend: {kind: real, discovery: {network: host}}\n"
        "  robot: {model: Lab arm}\n  arm: {joints: [j1], limits_rad: [[-1, 1]]}\n")
    (tmp_path / "config.yaml").write_text("simulator: robosuite\n")
    c = compose(str(lab), None, "libero_pro", tmp_path)
    assert c.simulator is None and c.cfg["machine"]["backend"]["kind"] == "real"
