"""VLABench loader: its registered tasks, one env per task.

VLABench (OpenMOSS/VLABench, arXiv 2502.09858) registers its tasks in
``VLABench.utils.register`` when its task package is imported;
``load_env(task, robot=...)`` builds the dm_control env with the task's
own scene and the named robot, ``reset`` randomises the episode, the
task states its instruction per episode (``task.get_instruction``) and
ends the episode when its conditions hold (``should_terminate_episode``),
which is the success the benchmark's evaluator reads off
``timestep.last()``. The catalog lists task names with no sentence: the
instruction names the episode's objects, so it exists once the world
does.

Assets live in the checkout (``VLABENCH_ROOT``), which the loader
points at from the venv it runs in.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _root() -> Path:
    return Path(sys.prefix).parent / "VLABench" / "VLABench"


def _prepare() -> None:
    os.environ.setdefault("VLABENCH_ROOT", str(_root()))
    import VLABench  # noqa: F401
    import VLABench.robots  # noqa: F401  registers the robots
    import VLABench.tasks  # noqa: F401  registers the tasks


def _task_names() -> list[str]:
    _prepare()
    from VLABench.utils.register import register

    return list(register._tasks)


class VLABenchLoader:
    def tasks(self, cfg: dict, task_suite: str) -> list[dict]:
        return [{"task_id": i, "language": ""} for i in range(len(_task_names()))]

    def create(self, cfg: dict, task_suite: str, task_id: int):
        _prepare()
        from VLABench.envs import load_env

        name = _task_names()[task_id]
        robot = cfg.get("machine", {}).get("engine_model") or "franka"
        env = load_env(name, robot=robot)
        env.reset()
        return env, {"task_id": task_id, "name": name}

    def init_state(self, ctx: dict, seed: int):
        """No init files: a seeded reset (VLABench samples from numpy's
        global RNG)."""
        return seed

    def reset(self, env, ctx: dict, state) -> None:
        import numpy as np

        np.random.seed(int(state))
        env.reset()

    def success(self, env) -> bool:
        """The task's own termination condition, in place."""
        return bool(env.task.should_terminate_episode(env.physics))

    def task_info(self, env, ctx: dict) -> dict:
        return {"language": str(env.task.get_instruction()), "name": ctx.get("name", "")}


LOADER = VLABenchLoader()
