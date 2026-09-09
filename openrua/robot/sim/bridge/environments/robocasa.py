"""RoboCasa loader (the original release, robocasa v0.2 on robosuite 1.5.0).

Envs are built with robosuite.make and the kwargs their
``robocasa.utils.env_utils.create_env`` passes, verbatim; only
controller_configs is swapped to the JOINT_POSITION composite. The successor release, RoboCasa365, has its own loader:
its task names, action space and Python API are not compatible.

``task.split`` picks the distribution: ``eval`` (their held-out object
instances, split B, in the five fixed layout/style pairs of their
``eval_utils.create_eval_env``), ``train`` (split A, every scene) or
``all``.
"""

from __future__ import annotations

import numpy as np

# Their eval_utils.create_eval_env: held-out instances in five fixed scenes.
EVAL_LAYOUT_AND_STYLE_IDS = [(1, 1), (2, 2), (4, 4), (6, 9), (7, 10)]


class RoboCasaLoader:
    def tasks(self, cfg: dict, task_suite: str) -> list[dict]:
        # A suite IS one of their env names: one task, whose sentence
        # their reset writes per episode (task_info reads it).
        return [{"task_id": 0, "language": ""}]

    def create(self, cfg: dict, task_suite: str, task_id: int):
        import json as _json

        import robocasa  # noqa: F401; import registers the kitchen envs
        import robosuite

        split = cfg.get("task", {}).get("split", "eval")
        if split == "eval":
            obj_instance_split, layout_and_style_ids = "B", EVAL_LAYOUT_AND_STYLE_IDS
        elif split == "train":
            obj_instance_split, layout_and_style_ids = "A", None
        elif split == "all":
            obj_instance_split, layout_and_style_ids = None, None
        else:
            raise ValueError(f"unknown robocasa split: {split} (eval | train | all)")

        with open(cfg["machine"]["controller_config"]) as f:
            controller_config = _json.load(f)
        cam_cfg = cfg.get("machine", {}).get("cameras", {})
        res = cam_cfg.get("resolution", [640, 480])
        cams = cam_cfg.get("list") or [
            "robot0_agentview_left", "robot0_agentview_right", "robot0_eye_in_hand",
        ]
        env = robosuite.make(
            env_name=task_suite,
            robots=cfg.get("machine", {}).get("engine_model") or "PandaOmron",
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
            seed=0,  # per-trial seeding happens at reset
            obj_instance_split=obj_instance_split,
            generative_textures=None,
            randomize_cameras=False,
            layout_and_style_ids=layout_and_style_ids,
            layout_ids=None,
            style_ids=None,
            translucent_robot=False,
        )
        return env, {"task_id": task_id, "robocasa_task": task_suite, "split": split}

    def init_state(self, ctx: dict, seed: int):
        """Seed -> env.rng -> reset: layout, style, object instances and
        placements are drawn from that rng in their code."""
        return {"seed": seed}

    def reset(self, env, ctx: dict, state) -> None:
        env.rng = np.random.default_rng(state["seed"])
        env.reset()

    def success(self, env) -> bool:
        """The kitchen env's ORIGINAL _check_success, in place."""
        return bool(env._check_success())

    def task_info(self, env, ctx: dict) -> dict:
        def _id(attr):
            v = getattr(env, attr, None)
            return None if v is None else int(v)

        return {"language": env.get_ep_meta().get("lang", ""),
                "name": ctx.get("robocasa_task", ""),
                "init_state": {"layout_id": _id("layout_id"),
                               "style_id": _id("style_id"),
                               "split": ctx.get("split")}}


LOADER = RoboCasaLoader()
