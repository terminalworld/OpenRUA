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
        "robot: panda\nsimulator: robosuite\nagent: claude-code\n")
    c = compose(None, None, None, tmp_path)
    assert (c.robot, c.simulator, c.benchmark, c.suite) == ("panda", "robosuite", None, "Lift")
    # an agent's facts land under agents.<name>: the flagged agent, else the file's
    _run(["--home", home, "config", "set", "--bench", "libero_pro", "--model", "m-1"])
    _run(["--home", home, "config", "set", "--agent", "codex", "--credentials-dir", "/c",
          "--version", "1.2.3"])
    text = (tmp_path / "config.yaml").read_text()
    assert "agents:\n  claude-code:\n    model: m-1\n" in text
    assert "  codex:\n    credentials_dir: /c\n    version: 1.2.3\n" in text
    assert "agent: codex" in text                            # the default moved with --agent
    c = compose(None, None, None, tmp_path)
    assert (c.benchmark, c.suite) == ("libero_pro", "libero_goal_task")
    # the benchmark names its agent, and that wins over the file's default (as robot: does)
    assert c.cfg["agent"]["name"] == "claude-code"
    assert c.cfg["agent"]["model"] == "claude-opus-5"        # the benchmark's own wins over m-1
    c = compose(None, None, None, tmp_path, agent="codex")   # --agent wins over both
    assert c.cfg["agent"]["name"] == "codex" and c.cfg["agent"]["credentials_dir"] == "/c"
    assert "model" not in c.cfg["agent"]                    # the benchmark's model is claude-code's
    c = compose(None, None, "capbench", tmp_path)               # a flag wins over the file
    assert c.benchmark == "capbench"
    _run(["--home", home, "config", "show"])
    out = capsys.readouterr().out
    assert "benchmark: libero_pro" in out and "agents:" in out
    # the old shape is refused with the rewrite spelled out
    (tmp_path / "config.yaml").write_text("agent:\n  name: codex\n  credentials_dir: /c\n")
    import pytest
    from openrua.errors import ConfigError
    with pytest.raises(ConfigError) as e:
        _run(["--home", home, "config", "set", "--robot", "panda"])
    assert "agents.<name>" in str(e.value) and "agent: codex" in e.value.hint


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
        "machine:\n  backend: {kind: real, discovery: {network: host}}\n  cameras: {list: []}\n"
        "  robot: {model: Lab arm}\n  arm: {joints: [j1], limits_rad: [[-1, 1]]}\n"
        "  frames: {world: world, base: base_link, hand: tool0}\n")
    (tmp_path / "config.yaml").write_text("simulator: robosuite\n")
    c = compose(str(lab), None, "libero_pro", tmp_path)
    assert c.simulator is None and c.cfg["machine"]["backend"]["kind"] == "real"


def test_a_file_of_your_own_is_saved_by_its_full_path(tmp_path, monkeypatch):
    # A relative path would name a different file, or none, from another
    # directory; the defaults keep the one you meant.
    (tmp_path / "work").mkdir()
    (tmp_path / "work" / "my-arm.yaml").write_text("type: panda\n")
    monkeypatch.chdir(tmp_path / "work")
    assert _run(["--home", str(tmp_path), "config", "set", "--robot", "./my-arm.yaml",
                 "--agent", "claude-code"]) == 0
    text = (tmp_path / "config.yaml").read_text()
    assert f"robot: {tmp_path / 'work' / 'my-arm.yaml'}\n" in text and "agent: claude-code\n" in text


def test_robots_lists_the_ones_connected_on_this_computer(tmp_path, capsys):
    import json
    connected = tmp_path / "robots" / "my-arm"
    connected.mkdir(parents=True)
    (connected / "robot.yaml").write_text(
        "type: panda\nmachine:\n  robot: {model: My Arm, description: a test arm}\n"
        "  backend: {kind: real}\n")
    (tmp_path / "robots" / "half-done").mkdir()   # connecting not finished: no robot.yaml yet
    assert _run(["--home", str(tmp_path), "robots"]) == 0
    out = capsys.readouterr().out
    assert "connected on this computer" in out and "my-arm" in out and "half-done" not in out
    assert str(connected / "robot.yaml") in out
    assert _run(["--home", str(tmp_path), "robots", "--json"]) == 0
    rows = json.loads(capsys.readouterr().out)
    assert [r["name"] for r in rows if r.get("connected")] == ["my-arm"]
    assert "panda" in [r["name"] for r in rows if not r.get("connected")]
