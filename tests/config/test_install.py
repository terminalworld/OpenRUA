"""The declared install: bundled declarations are complete and point at
files that ship; the renderer turns one into a runnable script; doctor
checks the result against the declaration."""
from pathlib import Path

import pytest

from openrua import config
from openrua.config import paths
from openrua.robot.sim import install as installer


def test_every_bundled_benchmark_declares_a_complete_install():
    for e in paths.available("benchmarks"):
        _, b = config.load_benchmark(e.name)
        inst = config.install_for(b["simulator"], e.name)
        assert inst["python"] and inst["checkouts"] and inst["requirements"], e.name
        assert Path(inst["requirements"]).is_file(), e.name        # ships in the package
        for c in inst["checkouts"]:
            assert len(c["commit"]) == 40, (e.name, c)
            if c.get("patch"):
                assert Path(c["patch"]).is_file(), (e.name, c["patch"])
        assert "{here}" not in (inst.get("shell") or "")


def test_a_benchmark_writes_over_the_simulator_key_by_key(tmp_path):
    sim = config.install_for("robosuite")
    assert sim["venv"] == "cap-x/.venv-capbench" and sim["python"] == "3.10"
    lib = config.install_for("robosuite", "libero_pro")
    assert lib["venv"] == "cap-x/.venv-libero" and lib["python"] == "3.12"
    assert lib["checkouts"][0]["path"] == "cap-x"                 # the same checkout, declared again
    cap = config.install_for("robosuite", "capbench")
    assert cap["checkouts"] == sim["checkouts"]                   # nothing written: the simulator's


def test_relative_files_resolve_next_to_the_yaml_that_names_them(tmp_path):
    (tmp_path / "lock.txt").write_text("numpy\n")
    y = tmp_path / "mine.yaml"
    y.write_text("entry_point: libero\ntask: {benchmark: mine, suites: [s]}\n"
                 "robot: panda\nsimulator: robosuite\n"
                 "install: {venv: mine/.venv, python: '3.12', requirements: lock.txt,\n"
                 "  shell: 'echo {here}'}\n")
    inst = config.install_for("robosuite", str(y))
    assert inst["requirements"] == str(tmp_path / "lock.txt")
    assert inst["shell"] == f"echo {tmp_path}"
    assert inst["checkouts"][0]["patch"].endswith("capx-pyproject.patch")   # the simulator's, absolute


def test_render_is_one_rerunnable_script(tmp_path):
    inst = {"venv": "cap-x/.venv-x", "python": "3.12",
            "checkouts": [{"path": "cap-x", "repo": "https://x/y.git", "commit": "a" * 40,
                           "submodules": ["sub/one"], "patch": "/abs/p.patch"}],
            "requirements": "/abs/req.txt", "editable": ["cap-x"],
            "shell": "echo {root} {venv}"}
    s = installer.render(inst, "mine", tmp_path / "sims", tmp_path)   # tmp_path: no pyproject
    assert s.startswith("#!/usr/bin/env bash") and "set -euo pipefail" in s
    assert f'ROOT={tmp_path / "sims"}' in s and 'VENV="$ROOT/cap-x/.venv-x"' in s
    assert f'clone_at "$ROOT/cap-x" https://x/y.git {"a" * 40}' in s
    assert 'submodule update -q --init sub/one' in s and 'apply_patch "$ROOT/cap-x" /abs/p.patch' in s
    assert '-r /abs/req.txt' in s and '--no-deps -e "$ROOT/cap-x"' in s
    assert "openrua @ git+" in s                                    # not a checkout: the release
    assert 'echo "$ROOT" "$VENV"' in s
    (tmp_path / "pyproject.toml").write_text("[project]\nname='x'\n")
    assert f"-e {tmp_path}" in installer.render(inst, "mine", tmp_path / "sims", tmp_path)
    with pytest.raises(ValueError, match="python"):
        installer.render({"venv": "v"}, "mine", tmp_path, tmp_path)


def test_install_command_print_renders_the_defaults(tmp_path, capsys):
    from openrua.cli.commands import install as cmd
    import argparse
    (tmp_path / "config.yaml").write_text("benchmark: libero_pro\n")
    args = argparse.Namespace(home=tmp_path, sim=None, bench=None, print=True)
    assert cmd.run(args) == 0
    out = capsys.readouterr().out
    assert "cap-x/.venv-libero" in out and "PY=3.12" in out
    args = argparse.Namespace(home=tmp_path, sim="robosuite", bench=None, print=True)
    cmd.run(args)
    assert "cap-x/.venv-capbench" in capsys.readouterr().out
    (tmp_path / "config.yaml").write_text("{}\n")
    from openrua.errors import UsageError
    with pytest.raises(UsageError, match="--sim"):
        cmd.run(argparse.Namespace(home=tmp_path, sim=None, bench=None, print=True))
