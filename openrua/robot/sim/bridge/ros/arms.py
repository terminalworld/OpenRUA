"""Per-arm machine view (no ROS).

The bridge's command side loops ``machine.arms`` (materialized once by
normalize_arms into the resolved config); this module reads that list.
"""

from __future__ import annotations


def arm_specs(machine: dict) -> list[dict]:
    """The normalized per-arm list; loud when the config predates it."""
    arms = machine.get("arms")
    if not arms:
        raise KeyError(
            "machine.arms missing: the bridge boots from the RESOLVED "
            "resolved config (openrua bench writes it via normalize_arms); "
            "re-run the trial through `python -m openrua bench`")
    return arms
