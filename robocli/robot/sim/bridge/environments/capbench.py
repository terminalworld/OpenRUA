"""CaP-Bench loader: cap-x env classes as factories, robosuite driven raw.

Single wrap: cap-x's env classes are used as factories only;
everything downstream drives the
inner robosuite env directly and scores with the env's own
_check_success. Task identity is byte-identical to their published
numbers because construction goes through their code.
"""

from __future__ import annotations


# CaP-Bench core tasks -> (cap-x env class, controller json). Their
# single-arm classes default to panda_joint_ctrl.json and the two-arm
# classes to panda_joint_ctrl2.json; passing each class its OWN default
# keeps task identity byte-identical to their published grid.
_CTRL1 = "panda_joint_ctrl.json"
_CTRL2 = "panda_joint_ctrl2.json"
_TASKS = {
    "lift": ("capx.envs.simulators.robosuite_cube_lift",
             "FrankaRobosuiteCubeLiftLowLevel", _CTRL1),
    "stack": ("capx.envs.simulators.robosuite_cubes",
              "FrankaRobosuiteCubesLowLevel", _CTRL1),
    "nut": ("capx.envs.simulators.robosuite_nut_assembly",
            "FrankaRobosuiteNutAssembly", _CTRL1),
    "restack": ("capx.envs.simulators.robosuite_cubes_restack",
                "FrankaRobosuiteCubesRestackLowLevel", _CTRL1),
    "wipe": ("capx.envs.simulators.robosuite_spill_wipe",
             "FrankaRobosuiteSpillWipeLowLevel", _CTRL1),
    "twoarm_lift": ("capx.envs.simulators.robosuite_two_arm_lift",
                    "RobosuiteTwoArmLiftEnv", _CTRL2),
    "handover": ("capx.envs.simulators.robosuite_handover",
                 "RobosuiteHandoverEnv", _CTRL2),
}


class CapBenchLoader:
    BENCHMARKS = ("capbench",)

    def create(self, cfg: dict, task_suite: str, task_id: int):
        import importlib

        task_name = task_suite.replace("capbench_", "")
        if task_name not in _TASKS:
            raise ValueError(f"unknown capbench task: {task_name} "
                             f"(have {sorted(_TASKS)})")
        mod_name, cls_name, ctrl_json = _TASKS[task_name]
        # Their default controller_cfg is cwd-relative; resolve it against the
        # installed capx package so the loader can run from any cwd.
        import capx
        from pathlib import Path as _P
        capx_root = _P(capx.__file__).resolve().parents[1]
        # cap-x's own robosuite fork adds a `skip_render_images` kwarg to
        # MujocoEnv.step (a render-skipping optimization); vanilla robosuite
        # 1.5 lacks it and their nut/handover classes pass it during their
        # OWN reset. Accept-and-ignore keeps their code path byte-identical
        # (we merely never skip renders).
        import inspect as _inspect
        from robosuite.environments.base import MujocoEnv as _ME
        if "skip_render_images" not in _inspect.signature(_ME.step).parameters:
            _orig_step = _ME.step

            def _step(self, action, skip_render_images=False):
                return _orig_step(self, action)

            _ME.step = _step
        low = getattr(importlib.import_module(mod_name), cls_name)(
            controller_cfg=str(capx_root / "capx/integrations/robosuite/"
                               f"controllers/config/robots/{ctrl_json}"),
            privileged=False, enable_render=False,
            # Termination belongs to the runner, never to the env (LIBERO
            # and robocasa are built the same way): cap-x's default
            # max_steps would freeze a mid-task agent.
            max_steps=10**9,
        )
        env = low.robosuite_env
        language = cfg.get("task", {}).get(
            "task_language", {}).get(task_name, task_name)
        return env, {"low": low, "task_id": task_id, "language": language,
                     "capbench_task": task_name}

    def init_state(self, ctx: dict, seed: int):
        """No fixed init files; the protocol is seeded resets, so the
        "state" is the seed itself."""
        return seed

    def reset(self, env, ctx: dict, state) -> None:
        # Seeded reset through their own reset path (placement sampling
        # parity with CaP-X).
        ctx["low"].reset(seed=state)

    def success(self, env) -> bool:
        """The raw robosuite env's ORIGINAL _check_success, in place."""
        return bool(env._check_success())

    def task_info(self, env, ctx: dict) -> dict:
        return {"language": ctx.get("language", ""),
                "name": ctx.get("capbench_task", "")}
