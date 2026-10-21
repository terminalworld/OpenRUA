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
