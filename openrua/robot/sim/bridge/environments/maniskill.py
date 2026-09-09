"""ManiSkill 3 loader: its table-top tasks, and the engine's own scenes.

Both the ``maniskill`` benchmark and ``openrua run <robot> --sim
maniskill`` land here: ``gym.make`` on a registered ManiSkill task with
the ``pd_joint_pos`` controller, the PhysX CPU backend and one
sub-environment, cameras sized from the config. Success is the task's
own ``evaluate()["success"]``. ManiSkill tasks carry no language; the
sentence for each is the one-line goal from the task's documentation,
kept in ``TASKS`` below, so the agent reads the same goal a person
would.

Assets ManiSkill downloads on demand land next to the venv
(``MS_ASSET_DIR``), never in the container's home.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

# (env id, the task's goal in one sentence) for the table-top suite:
# Panda tasks that ship with the package and need no downloaded asset.
TASKS = [
    ("PickCube-v1", "Pick up the red cube and move it to the green goal position."),
    ("StackCube-v1", "Pick up the red cube and stack it on top of the green cube; let go so it stays there on its own."),
    ("PushCube-v1", "Push the red cube to the goal region marked on the table."),
    ("PullCube-v1", "Pull the red cube toward the robot until it reaches the goal region."),
    ("LiftPegUpright-v1", "Move the peg so that it stands upright on the table."),
    ("PegInsertionSide-v1", "Pick up the peg and insert its head into the hole in the box from the side."),
    ("PlugCharger-v1", "Pick up the charger and plug it into the wall receptacle."),
    ("PokeCube-v1", "Use the peg to poke the cube so it reaches the goal region."),
    ("PullCubeTool-v1", "Use the L-shaped tool to pull the cube within reach, then bring it close to the robot."),
    ("PlaceSphere-v1", "Pick up the sphere and place it into the bin."),
    ("RollBall-v1", "Push the ball so it rolls to the goal region."),
]
SUITES = {"tabletop": TASKS}


def _assets_next_to_the_venv() -> None:
    root = Path(sys.prefix).parent / "data"
    os.environ.setdefault("MS_ASSET_DIR", str(root))


def make_kwargs(cfg: dict, env_id: str) -> dict:
    """gym.make arguments every ManiSkill-based loader shares: one
    sub-environment on the PhysX CPU backend, rendering on the GPU only
    when the install says there is one, cameras sized from the config,
    the lightest observation mode the task allows (the bridge reads
    cameras itself), no termination (that belongs to the runner)."""
    from mani_skill.utils.registration import REGISTERED_ENVS

    machine = cfg.get("machine", {})
    w, h = machine.get("cameras", {}).get("resolution", [640, 480])
    size = dict(width=int(w), height=int(h))
    supported = list(REGISTERED_ENVS[env_id].cls.SUPPORTED_OBS_MODES)
    kwargs = dict(
        obs_mode="none" if "none" in supported else supported[0],
        control_mode=machine.get("controller") or "pd_joint_pos",
        sim_backend="physx_cpu",
        render_backend="gpu" if machine.get("backend", {}).get("gpus") else "cpu",
        render_mode=None, num_envs=1,
        sensor_configs=size, human_render_camera_configs=size,
        max_episode_steps=10**9,
    )
    if machine.get("engine_model"):
        kwargs["robot_uids"] = machine["engine_model"]
    return kwargs


def _success(env) -> bool:
    v = env.unwrapped.evaluate()["success"]
    return bool(v.flatten()[0]) if hasattr(v, "flatten") else bool(v)


class ManiSkillLoader:
    def tasks(self, cfg: dict, task_suite: str) -> list[dict]:
        if task_suite in SUITES:
            return [{"task_id": i, "language": s} for i, (_, s) in enumerate(SUITES[task_suite])]
        return [{"task_id": 0, "language": ""}]  # a native scene: the env id itself

    def _task(self, task_suite: str, task_id: int) -> tuple[str, str]:
        if task_suite in SUITES:
            return SUITES[task_suite][task_id]
        return task_suite, ""

    def create(self, cfg: dict, task_suite: str, task_id: int):
        _assets_next_to_the_venv()
        import gymnasium as gym
        import mani_skill.envs  # noqa: F401  registers the tasks

        env_id, sentence = self._task(task_suite, task_id)
        env = gym.make(env_id, **make_kwargs(cfg, env_id))
        return env, {"task_id": task_id, "env_id": env_id, "language": sentence}

    def init_state(self, ctx: dict, seed: int):
        """No init files: the task's own seeded reset."""
        return seed

    def reset(self, env, ctx: dict, state) -> None:
        env.reset(seed=int(state))

    def success(self, env) -> bool:
        """The task's own evaluate(), in place."""
        return _success(env)

    def task_info(self, env, ctx: dict) -> dict:
        return {"language": ctx.get("language", ""), "name": ctx.get("env_id", "")}


LOADER = ManiSkillLoader()
