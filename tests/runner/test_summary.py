"""SUMMARY.md is a view over the run's artifacts, regenerated each time."""

import json

from robocli.runner.record import write_run_summary


def _trial(run_dir, suite, task, seed, **rec):
    d = run_dir / "trials" / f"{suite}-{task}" / f"seed{seed}"
    d.mkdir(parents=True)
    (d / "result.json").write_text(json.dumps(
        {"task_suite": suite, "task_id": task, "init_state_id": seed, **rec}))


def test_summary_lists_trials_and_totals(tmp_path):
    run = tmp_path / "libero_pro" / "demo"
    run.mkdir(parents=True)
    assert write_run_summary(run) is None          # nothing recorded yet
    (run / "config.json").write_text(json.dumps({
        "config_file": "benchmarks/libero_pro.yaml", "config_sha256": "abc123def456ff",
        "agent_cli": {"name": "claude-code", "version_pin": None}, "operator": "agent",
        "robocli_version": "0.1.0", "robocli_commit": "0123456789abcdef", "git_dirty": False,
        "simulator_commit": "fedcba9876543210", "sim_image_digest": "sha256:aaaa",
        "sandbox_image_digest": "sha256:bbbb", "proxy_image_digest": "sha256:cccc",
        "config": {"task": {"benchmark": "libero_pro"}, "agent": {}}}))
    _trial(run, "libero_goal_task", 0, 0, success=True, termination="self_finished",
           wall_seconds=100.5, operator_meta={"num_turns": 42, "model": "claude-opus-5"},
           agent_version="2.1.226 (Claude Code)", anomaly=None)
    _trial(run, "libero_goal_task", 0, 1, success=False, termination="anomaly",
           wall_seconds=3.0, operator_meta={}, anomaly="RuntimeError: preflight failed: tf_flow")
    _trial(run, "libero_goal_task", 1, 0, success=None, termination="operator_done",
           wall_seconds=50.0, operator_meta={}, anomaly=None, verdict="not_applicable")
    out = write_run_summary(run)
    text = out.read_text()
    assert out.name == "SUMMARY.md" and "# demo" in text
    assert "1/2 succeeded" in text and "1 anomalies" in text
    assert "| libero_goal_task | 0 | 0 | yes | self_finished | 100.5 | 42 |" in text
    assert "| libero_goal_task | 1 | 0 | n/a | operator_done | 50.0 |  |" in text
    assert "preflight failed: tf_flow" in text
    assert "claude-code 2.1.226 (Claude Code)" in text and "claude-opus-5" in text
    assert not (run / ".SUMMARY.md.tmp").exists()
