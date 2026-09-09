"""Joint naming: engine joint names -> the published franka names.

Graph knowledge (the prefix map mirrors franka_ros2 naming); shared by
the sensor publishers (forward direction) and the trajectory port
(reverse lookup) so both sides use identical naming. The engine arrives
as a parameter; this module knows nothing about how it was built.
"""

from __future__ import annotations


def build_joint_map(engine, cfg: dict) -> list[tuple[str, int, int]]:
    """[(published_name, qpos_index, qvel_index)] for robot joints only.

    Shared by the sensor publishers (forward direction) and the trajectory
    port (reverse lookup) so both sides use identical naming.
    """
    prefix_map = cfg.get("machine", {}).get("joint_name_map",
                                            {"robot0_": "panda_"})
    out = []
    for name, qadr, dadr in engine.joints():
        for old, new in prefix_map.items():
            if name.startswith(old):
                out.append((name.replace(old, new, 1), qadr, dadr))
                break
    return out
