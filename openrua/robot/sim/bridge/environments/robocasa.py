"""RoboCasa365 loader: their registry, their split semantics, verbatim.

The task_suite string IS the env name (their registry, e.g.
"PickPlaceCounterToCabinet"); task_id unused (0). Their canonical
factory is robocasa.utils.env_utils.create_env, but it hardcodes the
default composite controller (OSC_POSE arm). We replicate its split
mapping + env kwargs VERBATIM and swap only controller_configs to our
JOINT_POSITION composite (as on LIBERO: OSC null-space bias fights
joint-space trajectory goals). Tasks,
scenes, predicates, split semantics stay theirs.
"""

from __future__ import annotations

import numpy as np


class RoboCasaLoader:
    def create(self, cfg: dict, task_suite: str, task_id: int):
        import json as _json
        from pathlib import Path as _P

        import robocasa  # noqa: F401; import registers the kitchen envs
        import robosuite

        env_name = task_suite
        split = cfg.get("task", {}).get("split", "target")
        # split mapping copied from robocasa.utils.env_utils.create_env
        if split == "target":
            obj_instance_split = "target"
            layout_ids = style_ids = None
            layout_and_style_ids = list(zip(range(1, 11), range(1, 11)))
        elif split == "pretrain":
            obj_instance_split = "pretrain"
            layout_ids = style_ids = -2
            layout_and_style_ids = None
        elif split == "all":
            obj_instance_split = None
            layout_ids = style_ids = -3
            layout_and_style_ids = None
        else:
            raise ValueError(f"unknown robocasa split: {split}")

        # Absolute: the runner resolved and copied it next to the config
        # (bringup.resolve_robot_files); the bridge never looks around.
        with open(cfg["machine"]["controller_config"]) as f:
            controller_config = _json.load(f)

        cam_cfg = cfg.get("machine", {}).get("cameras", {})
        res = cam_cfg.get("resolution", [640, 480])
        cams = cam_cfg.get("list") or [
            "robot0_agentview_left", "robot0_agentview_right",
            "robot0_eye_in_hand",
        ]
        env = robosuite.make(
            env_name=env_name,
            robots="PandaOmron",
            controller_configs=controller_config,
            camera_names=cams,
            camera_widths=res[0],
            camera_heights=res[1],
            has_renderer=False,
            has_offscreen_renderer=True,
            ignore_done=True,
            use_object_obs=True,
            use_camera_obs=True,
            camera_depths=False,
            seed=0,  # per-trial seeding happens at reset (their gym semantics)
            obj_instance_split=obj_instance_split,
            generative_textures=None,
            randomize_cameras=False,
            layout_and_style_ids=layout_and_style_ids,
            layout_ids=layout_ids,
            style_ids=style_ids,
            translucent_robot=False,
        )
        # Language is EPISODE metadata in robocasa (object instances vary per
        # reset): task_info reads env.get_ep_meta() after reset, never a
        # static sentence.
        return env, {"task_id": task_id, "robocasa_task": env_name,
                     "split": split}

    # Scene sampling is THEIRS, untouched: their kitchen env draws the
    # (layout, style) pair with rng.choice at every reset, out of a list it
    # already filtered down to the layouts/styles this task can legally run
    # in (kitchen.py EXCLUDE_LAYOUTS/EXCLUDE_STYLES). We only seed the rng
    # and reset -- byte for byte their gym wrapper's reset(seed=...).
    #
    # Drawing fewer rollouts than their fifty is the only protocol
    # deviation; the draw itself is theirs.

    def init_state(self, ctx: dict, seed: int):
        """Seed -> env.rng -> reset (their gym wrapper's semantics)."""
        return {"seed": seed}

    def reset(self, env, ctx: dict, state) -> None:
        # gym_wrapper.reset(seed): seed the env rng, then reset. The kitchen,
        # object instances, placements, robot base pose and textures all come
        # out of that rng, in their code, at their draw.
        env.rng = np.random.default_rng(state["seed"])
        env.reset()

    def success(self, env) -> bool:
        """The kitchen env's ORIGINAL _check_success, in place."""
        return bool(env._check_success())

    def task_info(self, env, ctx: dict) -> dict:
        # init_state: the optional slot every loader may fill with the
        # facts that identify THIS episode's world. Recorded verbatim, so
        # a result says which kitchen it ran in instead of leaving it to
        # be re-derived from the seed by whoever reads the code later.
        # int(): their rng.choice hands back numpy int64, which the trial
        # record cannot serialise.
        def _id(attr):
            v = getattr(env, attr, None)
            return None if v is None else int(v)

        return {"language": env.get_ep_meta().get("lang", ""),
                "name": ctx.get("robocasa_task", ""),
                "init_state": {"layout_id": _id("layout_id"),
                               "style_id": _id("style_id"),
                               "split": ctx.get("split")}}


LOADER = RoboCasaLoader()
