"""Environment: everything serving the simulated world's env object.

Two kinds of service live here: the per-world loaders (one per benchmark, plus the engine's native scenes) (the env's
construction and semantics: build, initial state, original predicate, task
sentence) and the worker (the env's access discipline: one owner thread,
everyone else queues). Consumers receive env and Worker as a pair from
main.py; nothing here knows the sibling packages.

Adding a benchmark = one module implementing every name in
``LOADER_INTERFACE`` (test-enforced) and exposing an instance as
``LOADER``; the benchmark's yaml names it under ``entry_point``, and the
resolved config carries the result as ``task.loader`` (a module path or
an absolute file path), which ``load()`` imports. No list here to edit.
Each loader wraps NOTHING: ``create`` goes through the benchmark's own
factory (single-wrap discipline; the env object is their class, their
instance), ``success`` calls their ORIGINAL predicate in place (scoring
identity; zero reimplementation), ``init_state``/``reset`` speak their
protocol, ``task_info`` reads their task sentence. The state returned by
``init_state`` is opaque to everyone but the same loader's ``reset``.

``tasks`` answers without an env: the suite's task ids and sentences,
which ``openrua benchmarks <name>`` prints (through ``catalog.py``, run
in the simulator's venv); an empty sentence means the benchmark writes
it at reset.

Zero ROS imports; simulator imports live inside ``create`` (the registry
imports cheaply anywhere). Zero knowledge of the sibling ros package and
rpc module: the env and the loader travel onward as parameters (main.py
passes them).
"""

from __future__ import annotations

import importlib
import importlib.util
import sys
from pathlib import Path

# The plug shape every loader implements (a duck-typed contract, like agents.Agent).
LOADER_INTERFACE = (
    "create",       # cfg, task_suite, task_id -> (env, ctx)
    "init_state",   # ctx, seed -> opaque state (its own reset understands it)
    "reset",        # env, ctx, state -> None (restore the world)
    "success",      # env -> bool (the benchmark's ORIGINAL predicate)
    "task_info",    # env, ctx -> {language, name, [init_state]}
    "tasks",        # cfg, task_suite -> [{task_id, language}] without an env (the catalog)
)


def load(spec: str):
    """The ``LOADER`` object of a resolved ``task.loader``: a module path
    (``openrua.robot.sim.bridge.environments.libero``) or an absolute
    ``.py`` file (a benchmark of your own, copied next to the config)."""
    if spec.endswith(".py"):
        module = _import_file(Path(spec))
    else:
        module = importlib.import_module(spec)
    loader = getattr(module, "LOADER", None)
    if loader is None:
        raise ValueError(f"{spec} exposes no LOADER")
    missing = [n for n in LOADER_INTERFACE if not hasattr(loader, n)]
    if missing:
        raise ValueError(f"{spec}: LOADER lacks {', '.join(missing)}")
    return loader


def _import_file(path: Path):
    """Import one module by file path under a private name; a failure
    removes the half-initialised module so a retry starts clean."""
    modname = f"_openrua_loader_{path.stem}_{abs(hash(str(path)))}"
    if modname in sys.modules:
        return sys.modules[modname]
    spec = importlib.util.spec_from_file_location(modname, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[modname] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(modname, None)
        raise
    return module
