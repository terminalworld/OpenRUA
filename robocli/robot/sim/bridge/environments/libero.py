"""LIBERO / LIBERO-PRO loader: their fork's factory, their predicates.

Fork-style imports throughout (see notes/09 gotchas: the LIBERO-PRO fork
flattens the package; ``from libero import ...``, not
``from libero.libero import ...``).
"""

from __future__ import annotations

import importlib.util
import os
import re
from pathlib import Path

import yaml


def write_libero_settings() -> Path:
    """LIBERO reads ``~/.libero/config.yaml`` at import and, when the
    file is missing, asks on stdin where to keep datasets. Write the
    package's own default paths first so the import never blocks. The
    paths derive from the installed package (the fork's ``libero``
    top-level package is the inner ``libero/libero`` directory), not
    from any checkout layout."""
    root = Path(importlib.util.find_spec("libero").origin).parent
    settings_dir = Path(os.environ.get("LIBERO_CONFIG_PATH", "~/.libero")).expanduser()
    settings_dir.mkdir(parents=True, exist_ok=True)
    path = settings_dir / "config.yaml"
    path.write_text(yaml.safe_dump({
        "benchmark_root": str(root),
        "bddl_files": str(root / "bddl_files"),
        "init_states": str(root / "init_files"),
        "datasets": str(root.parent / "datasets"),
        "assets": str(root / "assets"),
    }))
    return path


class LiberoLoader:
    BENCHMARKS = ("libero_pro", "libero")

    def create(self, cfg: dict, task_suite: str, task_id: int):
        write_libero_settings()
        from libero import benchmark, get_libero_path
        from libero.envs import OffScreenRenderEnv

        suite = benchmark.get_benchmark_dict()[task_suite]()
        task = suite.get_task(task_id)
        res = cfg.get("machine", {}).get("cameras", {}).get(
            "resolution", [640, 480])
        env = OffScreenRenderEnv(
            bddl_file_name=os.path.join(
                get_libero_path("bddl_files"), task.problem_folder, task.bddl_file
            ),
            # JOINT_POSITION is the config's base action space (real-robot
            # shape: both graph ports sit above a joint layer; FJT tracks
            # joints natively, twist runs differential IK like moveit_servo).
            # OSC was rejected after measurement: its null-space bias fights
            # joint-space goals (0.23 rad residual, see plan P1 log).
            controller=cfg.get("machine", {}).get("controller",
                                                  "JOINT_POSITION"),
            camera_heights=res[1],
            camera_widths=res[0],
            # Termination belongs to the evaluator (triple-OR rule), never to
            # the env: the default horizon=1000 made the env refuse stepping
            # mid-trial (smoke seed1 hit "executing action in terminated
            # episode" and the agent inferred a phantom time budget). The
            # motion-step budget question vs the VLA protocol's max_steps is a
            # disclosed protocol asymmetry (plan P5 ledger).
            horizon=int(cfg.get("protocol", {}).get("horizon", 10**9)),
        )
        # The task sentence's AUTHORITATIVE source is the bddl (:language ...)
        # line: perturbed variants (the *_task cells!) change goal AND language
        # in the bddl while the fork's static task map still returns the
        # original sentence; reading task.language would hand the agent the
        # WRONG instruction for an entire protocol column.
        bddl_path = os.path.join(
            get_libero_path("bddl_files"), task.problem_folder, task.bddl_file
        )
        m = re.search(r"\(:language\s+(.+?)\)\s*$",
                      open(bddl_path).read(), re.MULTILINE)
        language = m.group(1).strip() if m else getattr(task, "language", "")
        return env, {
            "suite": suite,
            "task": task,
            "task_id": task_id,
            "language": language,
        }

    def init_state(self, ctx: dict, seed: int):
        """Fixed benchmark init-state files; the seed indexes into them."""
        states = ctx["suite"].get_task_init_states(ctx["task_id"])
        return states[seed]

    def reset(self, env, ctx: dict, state) -> None:
        env.reset()
        env.set_init_state(state)

    def success(self, env) -> bool:
        """The benchmark's ORIGINAL BDDL goal predicate, in place."""
        return bool(env.check_success())

    def task_info(self, env, ctx: dict) -> dict:
        return {
            "language": ctx.get("language")
            or getattr(ctx.get("task"), "language", ""),
            "name": getattr(ctx.get("task"), "name", ""),
        }
