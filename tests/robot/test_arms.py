"""Pure multi-arm action math (no ROS imports): the commanded arm moves,
every other arm holds its current configuration."""

from __future__ import annotations

import numpy as np

from openrua.robot.sim.bridge.ros.arms import arm_targets


def test_absolute_mode_commanded_moves_others_hold():
    q = [np.array([0.1, 0.2]), np.array([1.0, 2.0])]
    dq = np.array([0.5, -0.5])
    t = arm_targets(q, dq, idx=1, absolute=True, out_max=[np.ones(2)] * 2)
    assert np.allclose(t[0], q[0])           # holds exactly where it is
    assert np.allclose(t[1], q[1] + dq)      # absolute target = q + delta


def test_delta_mode_commanded_clips_others_zero():
    q = [np.zeros(2), np.zeros(2)]
    dq = np.array([4.0, -0.25])
    out = [np.array([2.0, 1.0])] * 2
    t = arm_targets(q, dq, idx=0, absolute=False, out_max=out)
    assert np.allclose(t[0], [1.0, -0.25])   # clipped to [-1, 1] per joint
    assert np.allclose(t[1], [0.0, 0.0])     # zero delta = hold
