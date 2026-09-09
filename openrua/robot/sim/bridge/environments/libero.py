"""LIBERO family loader: the fork's factory, the fork's predicates.

Serves LIBERO, LIBERO-PRO, LIBERO-Plus and LIBERO-Mem, each from its
own fork installed as ``libero``. Two package layouts exist: the
LIBERO-PRO fork flattens the package (``from libero import benchmark``),
the others keep upstream's nesting (``from libero.libero import
benchmark``); ``_libero()`` finds whichever is installed. A task whose
init-state file the fork does not ship (LIBERO-Mem ships four of ten)
starts from a seeded ``env.reset()`` instead, and a fork whose success
predicate advances a subgoal state machine (LIBERO-Mem's ``inc=``) is
called that way on every step.
"""

from __future__ import annotations

import importlib
import importlib.util
import os
import re
from pathlib import Path

import yaml


def _libero():
    """The fork's inner package (``benchmark``, ``envs``, ``get_libero_path``
    live there), whichever layout is installed."""
    import libero
    try:
        from libero import libero as inner   # upstream layout: libero.libero
        if hasattr(inner, "get_libero_path"):
            return inner
    except ImportError:
        pass
    return libero                            # LIBERO-PRO's flattened layout


def write_libero_settings() -> Path:
    """LIBERO reads ``~/.libero/config.yaml`` at import and, when the
    file is missing, asks on stdin where to keep datasets. Write the
    package's own default paths first so the import never blocks. The
    paths derive from the installed package (the inner ``libero``
    package directory), not from any checkout layout."""
    spec = importlib.util.find_spec("libero.libero") or importlib.util.find_spec("libero")
    root = Path(spec.origin).parent
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


def _bddl_language(get_libero_path, task) -> str:
    """The task sentence from its bddl ``(:language ...)`` line, the
    AUTHORITATIVE source: perturbed variants (the *_task cells) change
    goal AND language in the bddl while the fork's static task map still
    returns the original sentence; reading task.language would hand the
    agent the WRONG instruction for an entire protocol column."""
    bddl_path = os.path.join(get_libero_path("bddl_files"), task.problem_folder, task.bddl_file)
    m = re.search(r"\(:language\s+(.+?)\)\s*$", open(bddl_path).read(), re.MULTILINE)
    return m.group(1).strip() if m else getattr(task, "language", "")


class LiberoLoader:
    def tasks(self, cfg: dict, task_suite: str) -> list[dict]:
        """Every task of a suite with its sentence, without building an
        env (the catalog ``openrua benchmarks <name>`` prints)."""
        write_libero_settings()
        lib = _libero()
        benchmark = importlib.import_module(lib.__name__ + ".benchmark")
        suite = benchmark.get_benchmark_dict()[task_suite]()
        return [{"task_id": i, "language": _bddl_language(lib.get_libero_path, suite.get_task(i))}
                for i in range(suite.n_tasks)]

    def create(self, cfg: dict, task_suite: str, task_id: int):
        import inspect

        write_libero_settings()
        lib = _libero()
        benchmark = importlib.import_module(lib.__name__ + ".benchmark")
        OffScreenRenderEnv = importlib.import_module(lib.__name__ + ".envs").OffScreenRenderEnv
        get_libero_path = lib.get_libero_path

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
        language = _bddl_language(get_libero_path, task)
        # LIBERO-Mem: its predicate advances a subgoal state machine only
        # when asked to (inc=True); asked on every step, as their loop does.
        params = inspect.signature(env.env._check_success).parameters
        self._success_inc = "inc" in params
        return env, {
            "suite": suite,
            "task": task,
            "task_id": task_id,
            "language": language,
        }

    def init_state(self, ctx: dict, seed: int):
        """Fixed benchmark init-state files, the seed indexing into them;
        a task without a file (LIBERO-Mem ships four of ten) starts from
        a seeded reset instead."""
        try:
            states = ctx["suite"].get_task_init_states(ctx["task_id"])
        except FileNotFoundError:
            return {"seed": seed}
        return states[seed]

    def reset(self, env, ctx: dict, state) -> None:
        if isinstance(state, dict) and "seed" in state:
            env.seed(state["seed"])
            env.reset()
        else:
            env.reset()
            env.set_init_state(state)
        if hasattr(env.env, "reset_subgoal_progress"):
            env.env.reset_subgoal_progress()

    def success(self, env) -> bool:
        """The benchmark's ORIGINAL BDDL goal predicate, in place."""
        if getattr(self, "_success_inc", False):
            return bool(env.env._check_success(inc=True))
        return bool(env.check_success())

    def task_info(self, env, ctx: dict) -> dict:
        return {
            "language": ctx.get("language")
            or getattr(ctx.get("task"), "language", ""),
            "name": getattr(ctx.get("task"), "name", ""),
        }


LOADER = LiberoLoader()
