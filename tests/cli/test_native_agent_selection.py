"""Native terminal entry points retain the plugin selected at robot startup."""

from types import SimpleNamespace

import pytest

from openrua.cli import build_parser
from openrua.cli.commands import agent
from openrua.runner import live_state


def plugin(tmp_path, filename="custom", version="1.0"):
    manifest = tmp_path / f"{filename}.yaml"
    manifest.write_text(f"name: {filename}-display\ndefault_model: model\nversion: '{version}'\nentry_point: ./terminal.py\n")
    (tmp_path / "terminal.py").write_text(
        "from openrua.agents import Agent\n"
        "class Terminal(Agent):\n"
        "    def interactive_argv(self, sandbox, model, proxy, options=None, prompt=None, **_):\n"
        "        return [self.name, self.version, sandbox, model, prompt or '', (options or {}).get('effort', '')]\n"
        "HOOKS = Terminal\n")
    return manifest


def save(tmp_path, manifest):
    live_state.save("robot", tmp_path, agent="custom-display", agent_spec=str(manifest),
                    agent_version="2.0", sandbox="sandbox", model="selected-model",
                    proxy="http://proxy", options={"effort": "high"})


def test_agent_reopens_the_recorded_manifest_and_version(tmp_path, monkeypatch):
    save(tmp_path, plugin(tmp_path))
    launched = []
    monkeypatch.setattr(agent.os, "execvp", lambda binary, argv: launched.append(argv))
    args = build_parser().parse_args(["--home", str(tmp_path), "agent", "--name", "robot", "Inspect"])
    assert agent.run(args) == 0
    assert launched == [["custom-display", "2.0", "sandbox", "selected-model", "Inspect", "high"]]


def test_explicit_agent_override_does_not_inherit_another_plugins_version(tmp_path, monkeypatch):
    save(tmp_path, plugin(tmp_path))
    alternative = plugin(tmp_path, "alternative", "3.0")
    launched = []
    monkeypatch.setattr(agent.os, "execvp", lambda binary, argv: launched.append(argv))
    args = build_parser().parse_args(["--home", str(tmp_path), "agent", "--name", "robot", "--agent", str(alternative), "--model", "another-model"])
    agent.run(args)
    assert launched[0][:4] == ["alternative-display", "3.0", "sandbox", "another-model"]


def test_legacy_state_without_plugin_spec_still_opens(tmp_path, monkeypatch):
    live_state.save("robot", tmp_path, agent="codex", sandbox="sandbox", model="model", proxy="http://proxy")
    launched = []
    monkeypatch.setattr(agent.os, "execvp", lambda binary, argv: launched.append(argv))
    agent.run(build_parser().parse_args(["--home", str(tmp_path), "agent", "--name", "robot"]))
    assert launched[0][0] == "docker" and "codex" in launched[0]


def test_agent_passes_the_sessions_limits(tmp_path, monkeypatch):
    manifest = plugin(tmp_path)
    (tmp_path / "terminal.py").write_text(
        "from openrua.agents import Agent\n"
        "class Terminal(Agent):\n"
        "    def interactive_argv(self, sandbox, model, proxy, options=None, prompt=None, limits=None, **_):\n"
        "        return [str(limits)]\n"
        "HOOKS = Terminal\n")
    live_state.save("robot", tmp_path, agent="custom-display", agent_spec=str(manifest),
                    agent_version="2.0", sandbox="sandbox", model="m", proxy="",
                    options={}, limits={"internet": "open", "web_search": False})
    launched = []
    monkeypatch.setattr(agent.os, "execvp", lambda binary, argv: launched.append(argv))
    args = build_parser().parse_args(["--home", str(tmp_path), "agent", "--name", "robot"])
    agent.run(args)
    assert launched == [[str({"internet": "open", "web_search": False})]]


def test_the_first_word_names_an_agent_when_it_is_one(tmp_path):
    manifest = plugin(tmp_path)
    assert agent._split([]) == (None, None)
    assert agent._split(["claude-code"]) == ("claude-code", None)
    assert agent._split(["claude-code", "open the drawer"]) == ("claude-code", "open the drawer")
    assert agent._split([str(manifest), "hi"]) == (str(manifest), "hi")
    assert agent._split(["open the drawer"]) == (None, "open the drawer")
    with pytest.raises(Exception, match="one opening message"):
        agent._split(["open", "the", "drawer"])


def test_without_a_name_the_newest_robot_up_is_opened(tmp_path, monkeypatch):
    import os
    import time
    manifest = plugin(tmp_path)
    for name, age in (("older", 100), ("newer", 0)):
        live_state.save(name, tmp_path, agent="custom-display", agent_spec=str(manifest),
                        agent_version="2.0", sandbox=f"{name}-sandbox", model="m", proxy="", project=os.getcwd())
        control = tmp_path / "sandboxes" / name / "control.json"
        control.write_text("{}")
        os.utime(control, (time.time() - age, time.time() - age))
    live_state.save("stopped", tmp_path, status="stopped", agent="x", sandbox="stopped-sandbox",
                    model="m", proxy="")
    monkeypatch.setattr(agent.subprocess, "run", lambda argv, **kw: SimpleNamespace(stdout="true\n"))
    assert agent._running(tmp_path) == ["newer", "older"]
    launched = []
    monkeypatch.setattr(agent.os, "execvp", lambda binary, argv: launched.append(argv))
    agent.run(build_parser().parse_args(["--home", str(tmp_path), "agent", "hello"]))
    assert launched[0][2] == "newer-sandbox" and launched[0][4] == "hello"


def test_with_none_up_it_starts_one_and_asks_to_turn_it_off(tmp_path, monkeypatch):
    manifest = plugin(tmp_path)
    started, ended, called = [], [], []

    def start_robot(args, name):
        started.append(name)
        live_state.save("fresh", tmp_path, agent="custom-display", agent_spec=str(manifest),
                        agent_version="2.0", sandbox="fresh-sandbox", model="m", proxy="",
                        backend="real")
        return "fresh", SimpleNamespace(wait=lambda timeout: 0)
    monkeypatch.setattr(agent, "_running", lambda home, project=None: [])
    monkeypatch.setattr(agent, "_start_robot", start_robot)
    monkeypatch.setattr(agent.sys, "stdin", SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr(agent.sys, "stdout", SimpleNamespace(isatty=lambda: True, write=lambda s: None, flush=lambda: None))
    monkeypatch.setattr(agent.subprocess, "call", lambda argv: called.append(argv) or 0)
    monkeypatch.setattr(agent.Client, "from_file", lambda path: SimpleNamespace(end=lambda: ended.append(path)))
    monkeypatch.setattr("builtins.input", lambda prompt: "")
    monkeypatch.setattr(agent.os, "execvp", lambda *a: pytest.fail("a robot it started is not exec'd into"))
    agent.run(build_parser().parse_args(["--home", str(tmp_path), "agent", str(manifest), "go"]))
    assert started == [str(manifest)] and called[0][2] == "fresh-sandbox" and called[0][4] == "go"
    assert ended and ended[0].name == "control.json"


def test_no_answer_to_the_turn_off_question_leaves_the_robot_running(tmp_path, monkeypatch, capsys):
    ended = []
    monkeypatch.setattr(agent.Client, "from_file", lambda path: SimpleNamespace(end=lambda: ended.append(path)))
    def interrupted(prompt):
        raise KeyboardInterrupt
    monkeypatch.setattr("builtins.input", interrupted)
    args = build_parser().parse_args(["--home", str(tmp_path), "agent"])
    agent._turn_off(args, "fresh", SimpleNamespace(wait=lambda timeout: 0))
    assert not ended and "openrua down --name fresh" in capsys.readouterr().out
