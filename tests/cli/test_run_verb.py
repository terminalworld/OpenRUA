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
        return up_cmd.Session("r", "sim", "box", "suite", 0, "task",
                              power_off=lambda: calls.append(("down",)))
    return open_session


def test_run_brings_up_opens_the_agent_and_powers_off(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(run_cmd.up, "open_session", _session(calls))
    monkeypatch.setattr(run_cmd.state, "load", lambda name, home: {
        "agent": "fake", "sandbox": "box", "model": "m", "proxy": "http://p", "options": {}})
    monkeypatch.setattr(run_cmd.agents, "get", lambda name, home: _Adapter(["agent-cli"]))
    monkeypatch.setattr(run_cmd.subprocess, "call",
                        lambda argv: calls.append(("agent", argv)) or 3)
    args = build_parser().parse_args(["--home", str(tmp_path), "run", "panda-sim", "hello"])
    assert args.fn(args) == 3
    assert calls == [("up", "panda-sim"), ("agent", ["agent-cli", "box", "m", "hello"]),
                     ("down",)]


def test_run_powers_off_when_the_agent_cannot_open(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(run_cmd.up, "open_session", _session(calls))
    monkeypatch.setattr(run_cmd.state, "load", lambda name, home: {
        "agent": "fake", "sandbox": "box", "model": "m", "proxy": "http://p"})
    monkeypatch.setattr(run_cmd.agents, "get", lambda name, home: _Adapter(None))
    args = build_parser().parse_args(["--home", str(tmp_path), "run", "panda-sim"])
    with pytest.raises(run_cmd.UnavailableError):
        args.fn(args)
    assert calls[-1] == ("down",)


def test_run_positionals_are_robot_then_prompt():
    args = build_parser().parse_args(["run", "panda-sim"])
    assert (args.robot, args.prompt) == ("panda-sim", None)
    args = build_parser().parse_args(["run", "panda-sim", "pick up the bowl", "--model", "x"])
    assert (args.robot, args.prompt, args.model) == ("panda-sim", "pick up the bowl", "x")


def test_bare_build_builds_all_three(monkeypatch, tmp_path, capsys):
    built = []
    monkeypatch.setattr(build_cmd, "build_robot",
                        lambda distro, tag: built.append(("robot", distro)) or ("r", "d1"))
    monkeypatch.setattr(build_cmd, "_sandbox",
                        lambda *a: built.append(("sandbox", a[2])) or ("s", "d2"))
    monkeypatch.setattr(build_cmd, "_proxy",
                        lambda *a: built.append(("proxy",)) or ("p", "d3"))
    args = build_parser().parse_args(["--home", str(tmp_path), "build"])
    assert args.fn(args) == 0
    assert built == [("robot", "jazzy"), ("sandbox", "jazzy"), ("proxy",)]
    assert capsys.readouterr().out.splitlines() == ["r d1", "s d2", "p d3"]


def test_build_one_unit_still_works(monkeypatch, tmp_path):
    built = []
    monkeypatch.setattr(build_cmd, "build_robot",
                        lambda distro, tag: built.append((distro, tag)) or ("r", "d"))
    args = build_parser().parse_args(["--home", str(tmp_path), "build", "robot",
                                      "--distro", "humble"])
    assert args.fn(args) == 0 and built == [("humble", None)]
