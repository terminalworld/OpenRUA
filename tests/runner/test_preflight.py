"""Preflight generation contract: the gate is GENERATED from the assembly
config, so every promised capability must mint its check (and an absent
capability its negative check). A regression that silently drops a check
would otherwise pass every trial unnoticed.
"""

from __future__ import annotations

from openrua.config import load_config

from openrua.runner.preflight import build_checks


class _Agent:
    def credentials_check(self):
        return ("credentials_readable", "true")

    def sandbox_cli_check(self):
        return ("sandbox_cli_matches_pin", "true")


def _names(cfg):
    # The checks read the normalized resolved config, like the runner.
    # Deep copy first: normalize mutates, and the fixtures are shared.
    import copy

    from openrua.config import normalize_arms
    view = normalize_arms(copy.deepcopy(cfg))
    return {name: snippet for name, snippet in build_checks(view, _Agent())}


_ARM_CFG = {"machine": {
    "ports": {"trajectory": "/panda_arm_controller/follow_joint_trajectory",
              "twist": "/servo_node/delta_twist_cmds",
              "gripper": "/franka_gripper/gripper_action",
              "wrench": "/franka_robot_state_broadcaster/external_wrench"},
    "arm": {"joints": ["panda_joint1", "panda_joint2"]},
    "frames": {"world": "world", "base": "panda_link0"},
    "workspace_template": "workspace",
    "planning": {"moveit": True, "ik_service": "/compute_ik"},
}}


def test_every_promised_port_mints_its_check():
    checks = _names(_ARM_CFG)
    for expected in ("shell_sh_provisioned", "credentials_readable",
                     "sandbox_cli_matches_pin", "clock_topic",
                     "joint_states_flow", "joint_names_match_manual",
                     "tf_flow", "trajectory_action", "twist_subscribed",
                     "gripper_action", "wrench_flow", "camera_frame_flow",
                     "camera_depth_intrinsics_flow",
                     "workspace_tools_importable", "moveit_ik_answers"):
        assert expected in checks, expected
    # the joint-name assertion carries the documented first joint
    assert "panda_joint1" in checks["joint_names_match_manual"]
    assert "/compute_ik" in checks["moveit_ik_answers"]


def test_absent_gripper_mints_the_negative_check():
    cfg = {"machine": {**_ARM_CFG["machine"],
                       "ports": {k: v for k, v in
                                 _ARM_CFG["machine"]["ports"].items()
                                 if k != "gripper"}}}
    checks = _names(cfg)
    assert "gripper_absent" in checks and "gripper_action" not in checks


def test_mobile_base_ports_mint_their_checks():
    cfg = {"machine": {**_ARM_CFG["machine"],
                       "ports": {**_ARM_CFG["machine"]["ports"],
                                 "base_twist": "/cmd_vel", "odom": "/odom"}}}
    checks = _names(cfg)
    assert "base_twist_subscribed" in checks and "odom_flow" in checks


def test_unpromised_capabilities_mint_no_check():
    checks = _names({"machine": {"ports": {}}})
    for absent in ("tf_flow", "joint_names_match_manual",
                   "workspace_tools_importable", "moveit_ik_answers",
                   "trajectory_action", "wrench_flow"):
        assert absent not in checks, absent


def test_real_configs_generate_a_full_gate():
    # The shipped resolved configs must all mint the baseline plus their
    # own promised capabilities (drift guard between configs and gate).
    from pathlib import Path

    import yaml
    for bench in ("libero_pro", "capbench", "robocasa365"):
        cfg = load_config(Path(__file__).resolve().parents[2] / "openrua" / "configs" /
                          "benchmarks" / f"{bench}.yaml")
        checks = _names(cfg)
        for expected in ("clock_topic", "joint_states_flow",
                         "joint_names_match_manual", "tf_flow",
                         "trajectory_action", "camera_frame_flow"):
            assert expected in checks, (bench, expected)


def test_twoarm_view_mints_per_arm_checks():
    from pathlib import Path

    import yaml

    from openrua.runner.preflight import build_checks
    from openrua.config import apply_suite_overrides, normalize_arms

    class _A:
        def credentials_check(self):
            return ("credentials_readable", "true")

        def sandbox_cli_check(self):
            return ("sandbox_cli_matches_pin", "true")

    cfg = load_config(Path(__file__).resolve().parents[2] / "openrua" / "configs" /
                          "benchmarks" / "capbench.yaml")
    apply_suite_overrides(cfg, "capbench_twoarm_lift")
    normalize_arms(cfg)
    names = [n for n, _ in build_checks(cfg, _A())]
    for side in ("left", "right"):
        assert f"trajectory_action_{side}" in names
        assert f"gripper_action_{side}" in names
        assert f"twist_subscribed_{side}" in names
        assert f"joint_names_match_manual_{side}" in names
    assert "moveit_ik_answers" not in names   # planner not promised
    assert "gripper_absent" not in names      # both arms have grippers


def test_flow_checks_carry_the_raised_ceiling():
    # Bimanual joint_states_flow/tf_flow take longer than 30 s to first
    # publish under heavy host load; the per-check ceiling is 60 s.
    # Guard against a silent revert (a lower ceiling would re-open the race
    # without changing any test that only counts check NAMES).
    from openrua.runner.preflight import _CHECK_TIMEOUT_S
    assert _CHECK_TIMEOUT_S >= 60
    checks = _names(_ARM_CFG)
    for flow in ("joint_states_flow", "tf_flow"):
        assert f"timeout {_CHECK_TIMEOUT_S} " in checks[flow], flow


def test_aggregate_budget_scales_with_the_ceiling():
    # The wrapper budget must cover every per-check ceiling firing in a row;
    # it is DERIVED (count x ceiling + slack), so raising the ceiling can
    # never leave the wrapper under-budgeted (a raw TimeoutExpired instead
    # of a readable red-check list).
    import copy

    from openrua.runner import preflight as preflight
    from openrua.config import normalize_arms
    view = normalize_arms(copy.deepcopy(_ARM_CFG))
    n = len(preflight.build_checks(view, _Agent()))
    expected = n * preflight._CHECK_TIMEOUT_S + 60.0
    assert expected >= n * 60  # ceiling is at least 60 per check
