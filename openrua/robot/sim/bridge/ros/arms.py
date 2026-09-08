"""Per-arm machine view + multi-arm action math (numpy only, no ROS).

The bridge's command side loops ``machine.arms`` (materialized once by
normalize_arms into the resolved config); this module holds the pure parts
so they stay unit-testable outside a ROS environment.
"""

from __future__ import annotations

import numpy as np


def arm_specs(machine: dict) -> list[dict]:
    """The normalized per-arm list; loud when the config predates it."""
    arms = machine.get("arms")
    if not arms:
        raise KeyError(
            "machine.arms missing: the bridge boots from the RESOLVED "
            "resolved config (openrua run writes it via normalize_arms); "
            "re-run the trial through `python -m openrua run`")
    return arms


def arm_targets(q_current: list[np.ndarray], dq: np.ndarray, idx: int,
                absolute: bool, out_max: list[np.ndarray]) -> list[np.ndarray]:
    """Per-arm action columns when arm ``idx`` receives delta ``dq``.

    Every other arm HOLDS: absolute controllers get their current joint
    positions back, delta controllers get zeros. The commanded arm gets
    ``q + dq`` (absolute) or ``clip(dq / out_max)`` (delta) -- the same
    conversion the single-arm bridge always did at this boundary.
    """
    out = []
    for i, q in enumerate(q_current):
        if i == idx:
            out.append(q + dq if absolute
                       else np.clip(dq / out_max[i], -1.0, 1.0))
        else:
            out.append(q.copy() if absolute else np.zeros(len(q)))
    return out
