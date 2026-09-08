"""Joint naming: sim joint names -> the published franka names.

Graph knowledge (the prefix map mirrors franka_ros2 naming); shared by
the sensor publishers (forward direction) and the trajectory port
(reverse lookup) so both sides use identical naming. The env arrives as
a parameter; this module knows nothing about how it was built.
"""

from __future__ import annotations


def build_joint_map(env, cfg: dict) -> list[tuple[str, int, int]]:
    """[(published_name, qpos_index, qvel_index)] for robot joints only.

    Shared by the sensor publishers (forward direction) and the trajectory
    port (reverse lookup) so both sides use identical naming.
    """
    sim = env.sim
    prefix_map = cfg.get("machine", {}).get("joint_name_map",
                                            {"robot0_": "panda_"})
    out = []
    for name in sim.model.joint_names:
        for old, new in prefix_map.items():
            if name.startswith(old):
                jid = sim.model.joint_name2id(name)
                out.append(
                    (
                        name.replace(old, new, 1),
                        sim.model.jnt_qposadr[jid],
                        sim.model.jnt_dofadr[jid],
                    )
                )
                break
    return out
