"""RoboTwin 2.0 loader: its fifty dual-arm tasks under one of its protocols.

RoboTwin (RoboTwin-Platform/RoboTwin, arXiv 2506.18088) builds a task
per episode: ``envs/<task>.py`` is a ``Base_Task`` subclass whose
``setup_demo(seed=...)`` places the objects, ``check_success`` is the
predicate, and ``task_config/<protocol>.yml`` (``demo_clean`` is the
"Easy" protocol, ``demo_randomized`` the "Hard" one) fixes the
embodiment, cameras and randomisation. Its evaluator creates a fresh
task for every episode; ``Episode`` below does the same on reset and
drives control ticks through the robot's own joint and gripper
setters, ten physics steps per tick, which is what its ``take_action``
control loop does between planned waypoints.

What this loader arranges around the checkout: its modules import by
path from the checkout root and read assets relative to it (the
process changes directory there, as its evaluator does). The declared
patch on the checkout (``eval-without-curobo.patch``) lets the robot
set up without CuRobo, the GPU motion planner its expert uses, and
reads the camera shader from ``ROBOTWIN_SHADER``: ray tracing when the
install has a GPU, SAPIEN's default otherwise. The sentence for a task
is its instruction template's ``full_description``.
"""

from __future__ import annotations

import importlib
import json
import os
import sys
from pathlib import Path

import numpy as np

PHYSICS_STEPS_PER_TICK = 10


def _root() -> Path:
    return Path(sys.prefix).parent / "RoboTwin"


def _task_names(root: Path) -> list[str]:
    return sorted(p.stem for p in (root / "envs").glob("*.py") if not p.stem.startswith("_"))


def _sentence(root: Path, task: str) -> str:
    f = root / "description" / "task_instruction" / f"{task}.json"
    if f.is_file():
        return json.loads(f.read_text()).get("full_description", "")
    return task.replace("_", " ")


def _import_robotwin(root: Path, gpus: bool) -> None:
    """Make the checkout importable the way its own scripts are run, and
    pick its camera shader: ray tracing on a GPU, SAPIEN's default
    otherwise (the declared patch reads ROBOTWIN_SHADER)."""
    for p in (str(root), str(root / "policy"), str(root / "description" / "utils")):
        if p not in sys.path:
            sys.path.insert(0, p)
    os.chdir(root)
    os.environ["ROBOTWIN_SHADER"] = "rt" if gpus else "default"


def _args(root: Path, protocol: str, task: str, cameras: dict) -> dict:
    import yaml

    args = yaml.safe_load((root / "task_config" / f"{protocol}.yml").read_text())
    args["task_name"], args["task_config"] = task, protocol
    kinds = yaml.safe_load((root / "task_config" / "_embodiment_config.yml").read_text())
    emb = args["embodiment"]
    files = [kinds[e]["file_path"] for e in emb[:2]] if len(emb) == 3 else [kinds[emb[0]]["file_path"]] * 2
    args["left_robot_file"], args["right_robot_file"] = files
    args["dual_arm_embodied"] = len(emb) == 1
    if len(emb) == 3:
        args["embodiment_dis"] = emb[2]
    for side, f in (("left", files[0]), ("right", files[1])):
        args[f"{side}_embodiment_config"] = yaml.safe_load((root / f / "config.yml").read_text())
    # The camera size comes from the config: RoboTwin's D435 is 320x240,
    # its Large_D435 640x480; both head and wrist cameras take the same type.
    w, h = cameras.get("resolution", [640, 480])
    kind = "Large_D435" if (int(w), int(h)) == (640, 480) else "D435"
    args["camera"]["head_camera_type"] = args["camera"]["wrist_camera_type"] = kind
    sizes = yaml.safe_load((root / "task_config" / "_camera_config.yml").read_text())
    args["head_camera_h"], args["head_camera_w"] = sizes[kind]["h"], sizes[kind]["w"]
    args["eval_mode"] = True
    return args


class Episode:
    """One RoboTwin task instance with the step the bridge expects."""

    def __init__(self, root: Path, protocol: str, task: str, cameras: dict):
        self._root, self._protocol, self._name = root, protocol, task
        self._args = _args(root, protocol, task, cameras)
        self._cls = getattr(importlib.import_module(f"envs.{task}"), task)
        self._seed = None
        self.task = None
        self.steps = 0
        self.physics_dt = 0.0
        self.control_dt = 0.0
        self._stable: list[int] = []  # RoboTwin seeds that placed the objects stably, in order

    def _setup(self, seed: int) -> bool:
        """Build the task at a RoboTwin seed; False when its placement was
        unstable (the seed its own evaluator skips)."""
        from envs.utils.create_actor import UnStableError

        self.close()
        self.task = self._cls()
        try:
            self.task.setup_demo(now_ep_num=0, seed=int(seed), is_test=True, **self._args)
        except UnStableError:
            self.close()
            return False
        return True

    def reset(self, episode: int = 0) -> None:
        """Episode n runs at the n-th stable seed counting up from 100000,
        as RoboTwin's evaluator numbers its episodes."""
        while len(self._stable) <= episode:
            seed = (self._stable[-1] + 1) if self._stable else 100000
            while not self._setup(seed):
                seed += 1
            self._stable.append(seed)
        if self.task is None or self._seed != self._stable[episode]:
            assert self._setup(self._stable[episode])
        self._seed = self._stable[episode]
        self.task.set_instruction(instruction=_sentence(self._root, self._name))
        self.physics_dt = float(self.task.scene.get_timestep())
        self.control_dt = self.physics_dt * PHYSICS_STEPS_PER_TICK
        self.steps = 0

    def step(self, action: dict) -> None:
        """One control tick: hold the per-arm targets for ten physics steps."""
        robot = self.task.robot
        for tag, (q, grip) in action.items():
            robot.set_arm_joints(np.asarray(q, dtype=float), np.zeros(len(q)), tag)
            robot.set_gripper(float(grip), tag)
        for _ in range(PHYSICS_STEPS_PER_TICK):
            self.task.scene.step()
        self.task._update_render()
        self.steps += PHYSICS_STEPS_PER_TICK

    def close(self) -> None:
        if self.task is not None:
            try:
                self.task.close_env(clear_cache=True)
            finally:
                self.task = None


class RoboTwinLoader:
    def tasks(self, cfg: dict, task_suite: str) -> list[dict]:
        root = _root()
        return [{"task_id": i, "language": _sentence(root, t)}
                for i, t in enumerate(_task_names(root))]

    def create(self, cfg: dict, task_suite: str, task_id: int):
        root = _root()
        _import_robotwin(root, bool(cfg.get("machine", {}).get("backend", {}).get("gpus")))
        name = _task_names(root)[task_id]
        episode = Episode(root, task_suite, name, cfg.get("machine", {}).get("cameras", {}))
        episode.reset(0)  # the bridge resets again with the trial's episode
        return episode, {"task_id": task_id, "name": name, "language": _sentence(root, name)}

    def init_state(self, ctx: dict, seed: int):
        """Episode number: the n-th stable RoboTwin seed from 100000 up."""
        return int(seed)

    def reset(self, env, ctx: dict, state) -> None:
        env.reset(int(state))

    def success(self, env) -> bool:
        return bool(env.task.check_success())

    def task_info(self, env, ctx: dict) -> dict:
        return {"language": env.task.get_instruction() or ctx.get("language", ""),
                "name": ctx.get("name", "")}


LOADER = RoboTwinLoader()
