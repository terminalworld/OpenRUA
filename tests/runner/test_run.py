"""The command line and the config view a trial runs under."""

from __future__ import annotations

from robocli.config import load_config

import subprocess
import sys


def test_package_front_door_lists_run():
    h = subprocess.run([sys.executable, "-m", "robocli", "--help"],
                       capture_output=True, text=True)
    assert h.returncode == 0 and "run" in h.stdout


def test_front_door_rejects_unknown_verb():
    h = subprocess.run([sys.executable, "-m", "robocli", "walk"],
                       capture_output=True, text=True)
    assert h.returncode == 2 and "invalid choice" in h.stderr


def test_run_spelling_is_the_conductor():
    # python -m robocli run --help must reach run.py's own argparse.
    h = subprocess.run([sys.executable, "-m", "robocli", "run", "--help"],
                       capture_output=True, text=True)
    assert h.returncode == 0 and "--config" in h.stdout


# --------------------------------------------- suite view resolution

def test_suite_overrides_deep_merge_and_null_delete(tmp_path):
    from robocli.config import apply_suite_overrides, load_config
    cfg = load_config("libero_pro", home=tmp_path)
    cfg["machine"]["ports"].update({"twist": "/t", "gripper": "/g"})
    cfg["suite_overrides"] = {"wipe": {"machine": {
        "ports": {"gripper": None}, "gripper": None,
        "robot": {"model": "B"}}}}
    apply_suite_overrides(cfg, "wipe")
    m = cfg["machine"]
    assert "gripper" not in m["ports"] and "gripper" not in m
    assert m["ports"]["twist"] == "/t"  # siblings survive the merge
    assert m["robot"]["model"] == "B"


def test_unknown_suite_is_a_no_op():
    from robocli.config import apply_suite_overrides
    cfg = {"machine": {"ports": {"twist": "/t"}}}
    import copy
    assert apply_suite_overrides(copy.deepcopy(cfg), "nope") == cfg


def test_capbench_wipe_view_agrees_machine_and_manifest():
    # The shipped override chain end to end: wipe's view must drop the
    # gripper from ports and from the generated machine.yaml, while
    # preflight mints the negative check.
    from pathlib import Path

    import yaml

    from robocli.runner.preflight import build_checks
    from robocli.config import apply_suite_overrides
    cfg = load_config(Path(__file__).resolve().parents[2] / "robocli" / "configs" /
                          "benchmarks" / "capbench.yaml")
    apply_suite_overrides(cfg, "capbench_wipe")
    assert not cfg["machine"]["ports"].get("gripper")

    class _A:
        def credentials_check(self):
            return ("credentials_readable", "true")

        def sandbox_cli_check(self):
            return ("sandbox_cli_matches_pin", "true")

    names = [n for n, _ in build_checks(cfg, _A())]
    assert "gripper_absent" in names and "gripper_action" not in names


# --------------------------------------------- the per-arm machine view

def test_normalize_arms_synthesizes_from_flat_fields():
    from pathlib import Path

    import yaml

    from robocli.config import normalize_arms
    cfg = load_config(Path(__file__).resolve().parents[2] / "robocli" / "configs" /
                          "benchmarks" / "libero_pro.yaml")
    normalize_arms(cfg)
    arms = cfg["machine"]["arms"]
    assert len(arms) == 1
    a = arms[0]
    assert a["joints"] == cfg["machine"]["arm"]["joints"]
    assert a["ports"]["trajectory"] == cfg["machine"]["ports"]["trajectory"]
    assert a["ports"]["gripper"] == cfg["machine"]["ports"]["gripper"]
    assert a["label"] == ""
    assert a["hand_body"] == "robot0_right_hand"
    assert a["base_frame"] == cfg["machine"]["frames"]["base"]


def test_normalize_arms_keeps_explicit_arms_untouched():
    from robocli.config import normalize_arms
    explicit = [{"label": "left", "joints": ["j1"], "ports": {}},
                {"label": "right", "joints": ["j2"], "ports": {}}]
    cfg = {"machine": {"arms": [dict(a) for a in explicit]}}
    normalize_arms(cfg)
    assert cfg["machine"]["arms"] == explicit


def test_twoarm_suite_view_resolves_two_arms():
    # The shipped capbench twoarm override end to end: two labeled arms,
    # per-arm ports, no planner promised (MoveIt is single-arm only).
    from pathlib import Path

    import yaml

    from robocli.config import apply_suite_overrides, normalize_arms
    cfg = load_config(Path(__file__).resolve().parents[2] / "robocli" / "configs" /
                          "benchmarks" / "capbench.yaml")
    apply_suite_overrides(cfg, "capbench_twoarm_lift")
    normalize_arms(cfg)
    m = cfg["machine"]
    arms = m["arms"]
    assert [a["label"] for a in arms] == ["left", "right"]
    for a in arms:
        assert a["ports"]["trajectory"].startswith(f"/{a['label']}_")
        assert a["ports"]["gripper"] and a["ports"]["twist"]
        assert len(a["joints"]) == 7
        assert a["joints"][0].startswith(a["label"])
    assert "planning" not in m           # no promised planner
    assert m["joint_name_map"]["robot1_"] == "right_panda_"
    assert arms[1]["hand_body"] == "robot1_right_hand"


def test_trial_record_stamps_the_workspace_template_hash():
    # config.json carries this too, but the runner rewrites config.json on
    # every trial: after a mid-campaign template edit only a trial-level
    # stamp can still separate the two halves.
    import inspect
    from robocli.runner import trial as T
    src = inspect.getsource(T.run_trial)
    assert '"workspace_template_sha256": workspace.template_hash(cfg)' in src
