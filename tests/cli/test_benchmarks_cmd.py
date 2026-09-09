"""``openrua benchmarks <name>``: suites, task ids and sentences to choose from."""

from __future__ import annotations

import json

from openrua.cli import build_parser
from openrua.cli.commands import list as listing


def _run(argv):
    args = build_parser().parse_args(argv)
    return args.fn(args)


def test_bare_listing_is_unchanged(capsys):
    assert _run(["benchmarks"]) == 0
    assert "libero_pro" in capsys.readouterr().out


def test_one_task_per_suite_and_the_seed_meaning(tmp_path, capsys, monkeypatch):
    monkeypatch.setattr(listing, "benchmark_catalog", lambda cfg, home: (
        {s: [{"task_id": 0, "language": f"do {s}"}] for s in cfg["task"]["suites"]}, ""))
    assert _run(["--home", str(tmp_path), "benchmarks", "capbench"]) == 0
    out = capsys.readouterr().out
    assert "capbench_lift        1 task: --task-ids 0" in out
    assert "do capbench_lift" in out
    assert "100 seeds per task" in out and "randomised reset" in out


def test_sentences_from_the_loader_are_asked_in_the_simulator_venv(tmp_path, capsys, monkeypatch):
    asked = {}

    def fake_catalog(cfg, home):
        asked["venv"] = cfg["machine"]["backend"]["simulator"]["venv"]
        return {s: [{"task_id": i, "language": f"{s} #{i}"} for i in range(3)]
                for s in cfg["task"]["suites"]}, ""

    monkeypatch.setattr(listing, "benchmark_catalog", fake_catalog)
    assert _run(["--home", str(tmp_path), "benchmarks", "libero_pro", "--json"]) == 0
    info = json.loads(capsys.readouterr().out)
    assert asked["venv"] == "cap-x/.venv-libero"
    assert info["init_states"] == "benchmark-files" and info["trials_per_task"] == 10
    assert info["suites"]["libero_goal_task"][2] == {"task_id": 2, "language": "libero_goal_task #2"}


def test_without_the_install_the_suites_still_list_and_the_note_says_how(tmp_path, capsys):
    assert _run(["--home", str(tmp_path), "benchmarks", "libero_pro"]) == 0
    out = capsys.readouterr().out
    assert "libero_goal_task\n" in out
    assert "openrua install --bench libero_pro" in out
