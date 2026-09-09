"""robosuite loader: the engine's own scenes, no benchmark.

What ``openrua run <robot> --sim robosuite`` loads when no benchmark is
named: ``robosuite.make`` on one of robosuite's stock environments
(Lift, Stack, ...) with the robot the config names (``machine.
engine_model``) and a joint-position composite controller. Success is
the env's own ``_check_success``; there is no task sentence, the scene
name stands in for it.
"""

from __future__ import annotations

# robosuite 1.5's composite format, arm part set to JOINT_POSITION with
# absolute targets and the gains CaP-X tuned for the Panda (their
# panda_joint_ctrl.json); the bridge's kp scale stays 1 on top of it.
_ARM_JOINT_POSITION = {
    "type": "JOINT_POSITION",
    "input_max": 10, "input_min": -10,
    "output_max": 1.0, "output_min": -1.0,
    "kd": 400, "kv": 200, "kp": 1500, "kp_limits": [0, 1400],
    "interpolation": "linear", "ramp_ratio": 0.2,
    "input_type": "absolute",
    "gripper": {"type": "GRIP"},
}


class RobosuiteLoader:
    def tasks(self, cfg: dict, task_suite: str) -> list[dict]:
        # A native scene is one task; the sentence is whatever `openrua run` is given.
        return [{"task_id": 0, "language": ""}]

    def create(self, cfg: dict, task_suite: str, task_id: int):
        import json as _json

        import numpy as np
        import robosuite
        from robosuite.controllers import load_composite_controller_config

        machine = cfg.get("machine", {})
        model = machine.get("engine_model") or "Panda"
        if machine.get("controller_config"):
            # Absolute: the runner copied it next to the config.
            with open(machine["controller_config"]) as f:
                controller = _json.load(f)
        else:
            controller = load_composite_controller_config(controller="BASIC", robot=model)
            for arm in ("right", "left"):
                if arm in controller["body_parts"]:
                    controller["body_parts"][arm] = dict(_ARM_JOINT_POSITION)
        cam_cfg = machine.get("cameras", {})
        res = cam_cfg.get("resolution", [640, 480])
        kwargs = dict(
            env_name=task_suite, robots=model, controller_configs=controller,
            has_renderer=False, has_offscreen_renderer=True, use_camera_obs=False,
            camera_widths=res[0], camera_heights=res[1],
            # Termination belongs to the runner, never to the env.
            ignore_done=True, horizon=10**9,
            control_freq=20,
        )
        if cam_cfg.get("list"):
            kwargs["camera_names"] = list(cam_cfg["list"])
        env = robosuite.make(**kwargs)
        return env, {"task_id": task_id, "scene": task_suite, "np": np}

    def init_state(self, ctx: dict, seed: int):
        """No init files: a seeded reset (robosuite samples placements
        from numpy's global RNG)."""
        return seed

    def reset(self, env, ctx: dict, state) -> None:
        ctx["np"].random.seed(int(state))
        env.reset()

    def success(self, env) -> bool:
        """The env's own _check_success, in place."""
        return bool(env._check_success())

    def task_info(self, env, ctx: dict) -> dict:
        return {"language": "", "name": ctx.get("scene", "")}


LOADER = RobosuiteLoader()
