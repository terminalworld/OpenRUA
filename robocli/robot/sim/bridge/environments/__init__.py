"""Environment: everything serving the simulated world's env object.

Two kinds of service live here: the per-benchmark loaders (the env's
construction and semantics: build, initial state, original predicate, task
sentence) and the worker (the env's access discipline: one owner thread,
everyone else queues). Consumers receive env and Worker as a pair from
main.py; nothing here knows the sibling packages.

Adding a benchmark = adding one module here implementing every name in
``LOADER_INTERFACE`` (test-enforced) and registering it in ``get()``.
Each loader wraps NOTHING: ``create`` goes through the benchmark's own
factory (single-wrap discipline; the env object is their class, their
instance), ``success`` calls their ORIGINAL predicate in place (scoring
identity; zero reimplementation), ``init_state``/``reset`` speak their
protocol, ``task_info`` reads their task sentence. The state returned by
``init_state`` is opaque to everyone but the same loader's ``reset``.

Zero ROS imports; simulator imports live inside ``create`` (the registry
imports cheaply anywhere). Zero knowledge of the sibling ros package and
rpc module: the env and the loader travel onward as parameters (main.py
passes them).
"""

from __future__ import annotations

from .capbench import CapBenchLoader
from .libero import LiberoLoader
from .robocasa import RoboCasaLoader

# The plug shape every loader implements (a duck-typed contract, like agents.Agent).
LOADER_INTERFACE = (
    "BENCHMARKS",   # config names this loader answers to
    "create",       # cfg, task_suite, task_id -> (env, ctx)
    "init_state",   # ctx, seed -> opaque state (its own reset understands it)
    "reset",        # env, ctx, state -> None (restore the world)
    "success",      # env -> bool (the benchmark's ORIGINAL predicate)
    "task_info",    # env, ctx -> {language, name, [init_state]}
)

LOADERS = (LiberoLoader(), CapBenchLoader(), RoboCasaLoader())


def get(benchmark: str):
    """The loader for a config's ``benchmark`` name."""
    for loader in LOADERS:
        if benchmark in loader.BENCHMARKS:
            return loader
    known = sorted(b for ld in LOADERS for b in ld.BENCHMARKS)
    raise ValueError(f"unknown benchmark: {benchmark} (have {known})")
