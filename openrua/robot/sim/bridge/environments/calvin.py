"""CALVIN loader: the long-horizon evaluation, one sequence per task.

CALVIN (mees/calvin, arXiv 2112.03227) evaluates a policy on 1000 chains
of five subtasks in its play table D: ``get_sequences`` draws the
chains and their initial conditions deterministically, ``get_env_state_
for_initial_condition`` turns a condition into the robot and scene state
the env resets to, and the task oracle (``calvin_env.envs.tasks.Tasks``)
decides when a subtask is done by comparing the scene before and after.
Each chain is a task here; the sentence lists its five instructions in
order (the benchmark gives them one at a time; an agent that reads the
list is told the same things). Success is the whole chain, judged
subtask by subtask with the oracle on every step, in order.

The env is built from the validation split's own recorded config
(``configs/benchmarks/calvin/merged_config.yaml``), cameras sized from
the config, EGL only when the install has a GPU (else PyBullet's CPU
renderer) and no tactile sensor. The
``Episode`` object drives control ticks through the robot's position
motors, the env's ``action_repeat`` physics steps per tick, and keeps
the chain's progress.
"""

from __future__ import annotations

from importlib import resources
from pathlib import Path

import numpy as np

NUM_SEQUENCES = 1000


def _stand_in_for_pyhash() -> None:
    """calvin_agent's evaluation utils hash an initial condition with
    pyhash's FNV-1 32-bit; pyhash no longer builds on current
    toolchains, and fnvhash computes the same function (the community
    harness makes the same substitution)."""
    import sys
    import types

    if "pyhash" in sys.modules:
        return
    try:
        import pyhash  # noqa: F401
        return
    except ImportError:
        pass
    import fnvhash

    module = types.ModuleType("pyhash")
    module.fnv1_32 = lambda: (lambda text: fnvhash.fnv1_32(str(text).encode()))
    sys.modules["pyhash"] = module


def _conf_dir() -> Path:
    import calvin_agent

    return Path(calvin_agent.__file__).resolve().parent.parent / "conf"


def _sequences():
    _stand_in_for_pyhash()
    from calvin_agent.evaluation.multistep_sequences import get_sequences

    return get_sequences(NUM_SEQUENCES)


def _annotations() -> dict:
    from omegaconf import OmegaConf

    ann = OmegaConf.load(_conf_dir() / "annotations/new_playtable_validation.yaml")
    return {k: list(v)[0] for k, v in ann.items()}


def _sentence(chain: list[str], ann: dict) -> str:
    return " ".join(f"{i + 1}. {ann[t]}" for i, t in enumerate(chain))


class Episode:
    """One CALVIN env with the chain it is evaluating."""

    def __init__(self, env, oracle, initial_condition: dict, chain: list[str]):
        self.env = env
        self.oracle = oracle
        self.initial_condition = initial_condition
        self.chain = list(chain)
        self.control_dt = 1.0 / env.control_freq
        self.physics_dt = self.control_dt / env.action_repeat
        self.steps = 0
        self.done = 0
        self._start_info = None

    def reset(self) -> None:
        _stand_in_for_pyhash()
        from calvin_agent.evaluation.utils import get_env_state_for_initial_condition

        robot_obs, scene_obs = get_env_state_for_initial_condition(self.initial_condition)
        self.env.reset(robot_obs=robot_obs, scene_obs=scene_obs)
        self.steps = 0
        self.done = 0
        self._start_info = self.env.get_info()

    def step(self, action: dict) -> None:
        import pybullet as pb

        env, robot = self.env, self.env.robot
        for j, q in zip(action["joints"], action["targets"]):
            env.p.setJointMotorControl2(bodyIndex=robot.robot_uid, jointIndex=j,
                                        controlMode=pb.POSITION_CONTROL, targetPosition=float(q),
                                        force=robot.max_joint_force, maxVelocity=robot.max_velocity,
                                        physicsClientId=env.cid)
        robot.gripper_action = int(action["gripper"])
        robot.control_gripper(robot.gripper_action)
        for _ in range(env.action_repeat):
            env.p.stepSimulation(physicsClientId=env.cid)
        env.scene.step()
        self.steps += env.action_repeat

    def progress(self) -> bool:
        """Advance the chain when the oracle sees the current subtask done;
        True once every subtask has been."""
        if self.done < len(self.chain):
            info = self.env.get_info()
            subtask = self.chain[self.done]
            if subtask in self.oracle.get_task_info_for_set(self._start_info, info, {subtask}):
                self.done += 1
                self._start_info = info
        return self.done == len(self.chain)

    def close(self) -> None:
        self.env.close()


class CalvinLoader:
    def tasks(self, cfg: dict, task_suite: str) -> list[dict]:
        ann = _annotations()
        return [{"task_id": i, "language": _sentence(chain, ann)}
                for i, (_, chain) in enumerate(_sequences())]

    def create(self, cfg: dict, task_suite: str, task_id: int):
        import hydra
        from omegaconf import OmegaConf

        merged = resources.files("openrua") / "configs/benchmarks/calvin/merged_config.yaml"
        conf = OmegaConf.load(str(merged))
        w, h = cfg.get("machine", {}).get("cameras", {}).get("resolution", [640, 480])
        conf.cameras.pop("tactile", None)
        for cam in conf.cameras.values():
            cam.width, cam.height = int(w), int(h)
        gpus = bool(cfg.get("machine", {}).get("backend", {}).get("gpus"))
        env = hydra.utils.instantiate(conf.env, show_gui=False, use_vr=False,
                                      use_egl=gpus,  # the EGL plugin needs a GPU; else TinyRenderer
                                      use_scene_info=True)
        oracle = hydra.utils.instantiate(
            OmegaConf.load(_conf_dir() / "callbacks/rollout/tasks/new_playtable_tasks.yaml"))
        initial_condition, chain = _sequences()[task_id]
        episode = Episode(env, oracle, initial_condition, chain)
        episode.reset()
        return episode, {"task_id": task_id, "chain": list(chain),
                         "language": _sentence(chain, _annotations())}

    def init_state(self, ctx: dict, seed: int):
        """A chain has one initial condition; the seed picks nothing."""
        return seed

    def reset(self, env, ctx: dict, state) -> None:
        env.reset()

    def success(self, env) -> bool:
        return env.progress()

    def task_info(self, env, ctx: dict) -> dict:
        return {"language": ctx.get("language", ""), "name": " > ".join(ctx.get("chain", []))}


LOADER = CalvinLoader()
