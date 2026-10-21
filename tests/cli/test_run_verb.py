"""``openrua run`` is gone and says so; bare ``openrua build`` builds all three images."""

from __future__ import annotations

import pytest

from openrua.cli import build_parser
from openrua.cli.commands import build as build_cmd
from openrua.cli.commands import up as up_cmd
from openrua.cli.commands import run as run_cmd
from openrua.runner import live, live_state


def test_run_is_removed_and_says_where_its_uses_went(tmp_path):
    args = build_parser().parse_args(["--home", str(tmp_path), "run", "panda", "hello"])
    with pytest.raises(run_cmd.UsageError) as caught:
        args.fn(args)
    assert "openrua exec" in caught.value.hint and "openrua agent" in caught.value.hint


def test_bare_build_builds_the_agent_images_and_the_default_benchmarks(monkeypatch, tmp_path, capsys):
    built = []
    monkeypatch.setattr(build_cmd, "build_simulator",
                        lambda install, name, code_root, tag: built.append(("sim", name, tag)) or (tag, "d1"))
    monkeypatch.setattr(build_cmd, "_sandbox",
                        lambda *a: built.append(("sandbox", a[2])) or ("s", "d2"))
    monkeypatch.setattr(build_cmd, "build_proxy",
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
    monkeypatch.setattr(build_cmd, "build_proxy", lambda *a: ("p", "d"))
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


def test_real_robot_opens_without_a_benchmark_or_reset(monkeypatch, tmp_path, proxy_address, agent_cli_ok):
    from types import SimpleNamespace

    profile = tmp_path / "robot.yaml"
    profile.write_text("type: panda\nmachine:\n  backend: {kind: real, discovery: {network: host}}\n  cameras: {list: []}\n"
                       "  gripper: {holds_partway: true, effort_honoured: true}\n")
    calls = []
    machine = SimpleNamespace(shutdown=lambda: calls.append("shutdown"))

    def bring_up(cfg, dest, sim, sandbox, suite, task_id, *args, **kwargs):
        assert cfg["machine"]["backend"]["kind"] == "real"
        assert "task" not in cfg
        assert (suite, task_id) == (None, None)
        return dest / "config.yaml", machine, 7

    monkeypatch.setattr(live, "bring_up", bring_up)
    monkeypatch.setattr(live, "ensure_internal_network", lambda: "internal")
    monkeypatch.setattr(live, "ensure_proxy", lambda *a, **k: "http://proxy")
    from openrua.agents import Agent, PreparedProfile
    monkeypatch.setattr(Agent, "prepare_profile", lambda self, source, dest, **kw: PreparedProfile(dest, ()))
    monkeypatch.setattr(live, "sandbox_down", lambda _: calls.append("sandbox down"))
    monkeypatch.setattr(live, "start_episode", lambda *a: pytest.fail("must not reset a real robot"))
    args = build_parser().parse_args(["--home", str(tmp_path / "home"), "up", str(profile),
                                      "--ros-domain", "7", "--task", "inspect the table"])
    session = up_cmd.open_session(args)
    assert session.task == "inspect the table"
    assert session.suite is None and session.scene() == "(none)"
    assert live_state.load(args.name, args.home)["task"] == "inspect the table"
    session.power_off()
    assert calls == ["sandbox down", "shutdown"]
    assert live_state.load(args.name, args.home, require_running=False)["status"] == "stopped"
    assert (args.home / "sandboxes" / args.name / "workspace").is_dir()
