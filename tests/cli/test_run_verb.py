"""``openrua run`` = up + agent + down in one process; bare ``openrua build``
builds all three images."""

from __future__ import annotations

import pytest

from openrua.cli import build_parser
from openrua.cli.commands import build as build_cmd
from openrua.cli.commands import run as run_cmd
from openrua.cli.commands import up as up_cmd


class _Adapter:
    name = "fake"

    def __init__(self, argv):
        self._argv = argv

    def interactive_argv(self, sandbox, model, proxy, options=None, prompt=None):
        return self._argv and [*self._argv, sandbox, model, prompt or ""]


def _session(calls):
    def open_session(args):
        calls.append(("up", args.robot))
        return up_cmd.Session(
            name="r", sim="sim", sandbox="box", robot=args.robot or "?",
            robot_model="Franka Emika Panda", backend="simulated by robosuite (ROS 2 jazzy)",
            benchmark="libero_pro", suite="libero_goal_task", task_id=0,
            task="open the bottom drawer of the cabinet", agent="fake", model="m",
            power_off=lambda: calls.append(("down",)))
    return open_session


def test_run_brings_up_opens_the_agent_and_powers_off(monkeypatch, tmp_path, capsys):
    calls = []
    monkeypatch.setattr(run_cmd.up, "open_session", _session(calls))
    monkeypatch.setattr(run_cmd.state, "load", lambda name, home: {
        "agent": "fake", "sandbox": "box", "model": "m", "proxy": "http://p", "options": {}})
    monkeypatch.setattr(run_cmd.agents, "get", lambda name, home: _Adapter(["agent-cli"]))
    monkeypatch.setattr(run_cmd.subprocess, "call",
                        lambda argv: calls.append(("agent", argv)) or 3)
    args = build_parser().parse_args(["--home", str(tmp_path), "run", "panda", "hello"])
    assert args.fn(args) == 3
    out = capsys.readouterr().out
    assert "[run] robot     panda: Franka Emika Panda, simulated by robosuite (ROS 2 jazzy)" in out
    assert '[run] scene     libero_pro / libero_goal_task #0: "open the bottom drawer' in out
    assert '[run] agent     fake (m), opening message: "hello"' in out
    assert calls == [("up", "panda"), ("agent", ["agent-cli", "box", "m", "hello"]),
                     ("down",)]


def test_run_powers_off_when_the_agent_cannot_open(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(run_cmd.up, "open_session", _session(calls))
    monkeypatch.setattr(run_cmd.state, "load", lambda name, home: {
        "agent": "fake", "sandbox": "box", "model": "m", "proxy": "http://p"})
    monkeypatch.setattr(run_cmd.agents, "get", lambda name, home: _Adapter(None))
    args = build_parser().parse_args(["--home", str(tmp_path), "run", "panda"])
    with pytest.raises(run_cmd.UnavailableError):
        args.fn(args)
    assert calls[-1] == ("down",)


def test_run_positionals_are_robot_then_prompt():
    args = build_parser().parse_args(["run", "panda"])
    assert (args.robot, args.prompt) == ("panda", None)
    args = build_parser().parse_args(["run", "panda", "pick up the bowl", "--model", "x"])
    assert (args.robot, args.prompt, args.model) == ("panda", "pick up the bowl", "x")


def test_bare_build_builds_the_agent_images_and_the_default_benchmarks(monkeypatch, tmp_path, capsys):
    built = []
    monkeypatch.setattr(build_cmd, "build_simulator",
                        lambda install, name, code_root, tag: built.append(("sim", name, tag)) or (tag, "d1"))
    monkeypatch.setattr(build_cmd, "_sandbox",
                        lambda *a: built.append(("sandbox", a[2])) or ("s", "d2"))
    monkeypatch.setattr(build_cmd, "_proxy",
                        lambda *a: built.append(("proxy",)) or ("p", "d3"))
    args = build_parser().parse_args(["--home", str(tmp_path), "build"])
    assert args.fn(args) == 0
    assert built == [("sandbox", "jazzy"), ("proxy",)]          # no default benchmark: a hint
    assert "openrua build --bench" in capsys.readouterr().out
    (tmp_path / "config.yaml").write_text("benchmark: libero_pro\n")
    built.clear()
    args = build_parser().parse_args(["--home", str(tmp_path), "build"])
    assert args.fn(args) == 0
    assert built == [("sandbox", "jazzy"), ("proxy",), ("sim", "libero_pro", "openrua-sim-libero_pro")]
    assert capsys.readouterr().out.splitlines()[-3:] == ["s d2", "p d3", "openrua-sim-libero_pro d1"]


def test_build_names_benchmarks_and_deduplicates_shared_images(monkeypatch, tmp_path):
    built = []
    monkeypatch.setattr(build_cmd, "build_simulator",
                        lambda install, name, code_root, tag: built.append(tag) or (tag, "d"))
    monkeypatch.setattr(build_cmd, "_sandbox", lambda *a: ("s", "d"))
    monkeypatch.setattr(build_cmd, "_proxy", lambda *a: ("p", "d"))
    args = build_parser().parse_args(["--home", str(tmp_path), "build", "--bench", "capbench",
                                      "--bench", "libero_pro", "--sim", "robosuite"])
    assert args.fn(args) == 0
    # capbench adds nothing to robosuite: one image for both, once
    assert built == ["openrua-sim-robosuite", "openrua-sim-libero_pro"]


def test_build_one_unit_still_works(monkeypatch, tmp_path):
    built = []
    monkeypatch.setattr(build_cmd, "build_base",
                        lambda distro, tag: built.append((distro, tag)) or ("r", "d"))
    args = build_parser().parse_args(["--home", str(tmp_path), "build", "base",
                                      "--distro", "humble"])
    assert args.fn(args) == 0 and built == [("humble", None)]
