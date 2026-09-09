"""Engines: the plug shape, machine-enforced, and the robosuite engine's
action assembly on a stand-in env (no simulator boots here)."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from openrua.config import paths
from openrua.robot.sim.bridge import engines


def _bundled_entry_points():
    for e in paths.available("simulators"):
        yield paths.entry_point("simulators", yaml.safe_load(e.path.read_text())["entry_point"], e.path)


def test_every_bundled_simulator_names_a_full_engine():
    specs = list(_bundled_entry_points())
    assert specs
    for spec in specs:
        engine = engines.load(spec)
        for name in engines.ENGINE_INTERFACE:
            assert hasattr(engine, name), f"{spec} missing {name}"


def test_an_engine_file_of_your_own_and_the_holes_it_can_have(tmp_path):
    good = tmp_path / "good.py"
    good.write_text("class E:\n    def __init__(self, env, cfg): pass\n" + "".join(
        f"    def {n}(self, *a, **k): pass\n" for n in engines.ENGINE_INTERFACE) + "ENGINE = E\n")
    assert hasattr(engines.bind(str(good), None, {}), "fk")
    (tmp_path / "none.py").write_text("x = 1\n")
    with pytest.raises(ValueError, match="no ENGINE"):
        engines.load(str(tmp_path / "none.py"))
    (tmp_path / "hole.py").write_text(
        "class E:\n    def __init__(self, env, cfg): pass\n    def joints(self): pass\nENGINE = E\n")
    with pytest.raises(ValueError, match="lacks"):
        engines.bind(str(tmp_path / "hole.py"), None, {})


# ------------------------------------------------- robosuite action assembly

def _robot(absolute: bool, action_dim: int, kp=50.0):
    ctrl = SimpleNamespace(output_max=np.array([0.5] * 2), kp=np.array([kp] * 2),
                           input_type="absolute" if absolute else "delta")
    return SimpleNamespace(part_controllers={"right": ctrl}, action_dim=action_dim)


def _bound(robots, qpos, kp_scale=10.0):
    from openrua.robot.sim.bridge.engines.robosuite import Robosuite

    raw = SimpleNamespace(robots=robots, sim=SimpleNamespace(data=SimpleNamespace(qpos=qpos)),
                          control_timestep=0.05, stepped=[])
    raw.step = raw.stepped.append
    engine = Robosuite(raw, {"machine": {"controller_kp_scale": kp_scale}})
    engine.bind_arms([(np.array([2 * i, 2 * i + 1]), f"robot{i}_right_hand")
                      for i in range(len(robots))])
    return engine, raw


def test_single_arm_absolute_action_is_current_plus_delta_with_gripper():
    engine, raw = _bound([_robot(absolute=True, action_dim=3)], np.array([0.1, 0.2]))
    assert engine.has_gripper(0) and not engine.mobile()
    a = engine.action(0, np.array([0.5, -0.5]), grips=[1.0], base_vel=None)
    assert np.allclose(a, [0.6, -0.3, 1.0])
    engine.step(a)
    assert raw.stepped == [a]  # step goes through env.step (the monitor's latch)


def test_single_arm_delta_action_clips_and_a_toolless_arm_has_no_gripper_column():
    engine, _ = _bound([_robot(absolute=False, action_dim=2)], np.zeros(2))
    assert not engine.has_gripper(0)
    a = engine.action(0, np.array([4.0, -0.125]), grips=[-1.0], base_vel=None)
    assert np.allclose(a, [1.0, -0.25])


def test_two_arms_commanded_moves_the_other_holds():
    engine, _ = _bound([_robot(True, 3), _robot(True, 3)], np.array([0.1, 0.2, 1.0, 2.0]))
    a = engine.action(1, np.array([0.5, -0.5]), grips=[-1.0, 1.0], base_vel=None)
    assert np.allclose(a, [0.1, 0.2, -1.0, 1.5, 1.5, 1.0])


def test_tuning_scales_kp_and_reapplies():
    engine, raw = _bound([_robot(True, 3, kp=50.0)], np.zeros(2), kp_scale=10.0)
    ctrl = raw.robots[0].part_controllers["right"]
    assert np.allclose(ctrl.kp, 500.0)
    ctrl.kp = np.array([50.0, 50.0])  # a reset reverted it
    engine.apply_tuning()
    assert np.allclose(ctrl.kp, 500.0)
