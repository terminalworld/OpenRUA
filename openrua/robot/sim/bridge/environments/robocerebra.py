"""RoboCerebra loader: their LIBERO fork's env classes, their goal checker.

A task is a case directory of the RoboCerebra_Bench dataset
(``<root>/<task_type>/<case>/``): a bddl file (the scene), ``demo.hdf5``
(the initial state, first frame of their demonstration), ``goal.json``
(the success criteria their ``_check_success(monitor_dict)`` scores)
and ``task_description.txt`` (the instruction). The suite is the task
type (``Ideal``, ``Memory_Execution``, ...); the task id indexes the
cases in numeric order. The bench root is ``task.dataset_root``, or
``ROBOCEREBRA_BENCH`` in the environment, else ``RoboCerebra_Bench``
next to the checkout the venv was installed from.
"""

from __future__ import annotations

import json
import os
import re
from pathlib import Path


def _bench_root(cfg: dict) -> Path:
    spec = cfg.get("task", {}).get("dataset_root") or os.environ.get("ROBOCEREBRA_BENCH")
    if spec:
        return Path(spec).expanduser()
    # Their checkout: <RoboCerebra>/LIBERO/libero/libero/__init__.py; the
    # outer ``libero`` may resolve as a namespace package, the inner never.
    from libero import libero as inner
    return Path(inner.__file__).resolve().parents[3] / "RoboCerebra_Bench"


def _cases(root: Path, task_type: str) -> list[Path]:
    d = root / task_type
    if not d.is_dir():
        have = sorted(p.name for p in root.iterdir() if p.is_dir()) if root.is_dir() else []
        raise FileNotFoundError(f"RoboCerebra task type {task_type!r} not found under {root} "
                                f"(have: {', '.join(have) or 'nothing; run openrua install'})")

    def key(p: Path):
        m = re.search(r"(\d+)$", p.name)
        return (int(m.group(1)) if m else 10**9, p.name)

    return sorted((p for p in d.iterdir() if p.is_dir() and list(p.glob("*.bddl"))), key=key)


class RoboCerebraLoader:
    def create(self, cfg: dict, task_suite: str, task_id: int):
        import h5py
        import libero.libero.envs  # noqa: F401; registers TASK_MAPPING
        from libero.libero.envs import OffScreenRenderEnv

        cases = _cases(_bench_root(cfg), task_suite)
        if task_id >= len(cases):
            raise IndexError(f"{task_suite} has {len(cases)} cases; task id {task_id}")
        case = cases[task_id]
        bddl = next(case.glob("*.bddl"))
        res = cfg.get("machine", {}).get("cameras", {}).get("resolution", [640, 480])
        env = OffScreenRenderEnv(
            bddl_file_name=str(bddl),
            controller=cfg.get("machine", {}).get("controller", "JOINT_POSITION"),
            camera_heights=res[1],
            camera_widths=res[0],
            horizon=int(cfg.get("protocol", {}).get("horizon", 10**9)),
        )
        with h5py.File(case / "demo.hdf5", "r") as f:
            init_state = f["data"]["demo_1"]["states"][0][...]
        goal = None
        if (case / "goal.json").is_file():
            raw = json.loads((case / "goal.json").read_text())
            goal = {}
            for obj, relations in raw.items():
                triples = []
                for item in relations:
                    t = item["state_pair"] if isinstance(item, dict) and "state_pair" in item else item
                    if isinstance(t, list):
                        triples.append([x.lower() if i == 0 else x for i, x in enumerate(t)])
                goal[obj] = triples
        language = case.name
        desc = case / "task_description.txt"
        if desc.is_file():
            for line in desc.read_text().splitlines():
                if line.strip().startswith("Task:"):
                    language = line.split(":", 1)[1].strip()
                    break
        # success() receives the env only; one env per bridge process, so
        # the loader keeps this case's goal.
        self._goal = goal
        return env, {"case": case, "init_state": init_state, "goal": goal,
                     "language": language, "task_id": task_id}

    def init_state(self, ctx: dict, seed: int):
        """One initial state per case (their demonstration's first frame);
        every seed restores it."""
        return ctx["init_state"]

    def reset(self, env, ctx: dict, state) -> None:
        env.reset()
        env.env.sim.set_state_from_flattened(state)
        env.env.sim.forward()
        env.env._post_process()
        env.env._update_observables(force=True)

    def success(self, env) -> bool:
        """Their ``_check_success(monitor_dict)``: the third return is
        whether every subtask of every object is complete."""
        goal = getattr(self, "_goal", None)
        if goal is None:
            return False
        return bool(env.env._check_success(goal)[2])

    def task_info(self, env, ctx: dict) -> dict:
        return {"language": ctx.get("language", ""), "name": ctx["case"].name}


LOADER = RoboCerebraLoader()
