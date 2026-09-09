"""The tasks a benchmark's suites hold, from the loader, as JSON.

Run in the simulator's venv by ``openrua benchmarks <name>`` (host
side, no container): the loader is the one party that knows how a
benchmark enumerates its tasks, and its imports live in that venv.
Writes ``{suite: [{task_id, language}, ...]}`` to ``--out`` (a file: a
benchmark's imports may write anything to stdout and stderr).

    python -m openrua.robot.sim.bridge.catalog --config <resolved.yaml> --out tasks.json [--task-suite S ...]
"""

from __future__ import annotations

import argparse
import json

import yaml

from . import environments


def catalog(cfg: dict, suites: list[str]) -> dict:
    loader = environments.load(cfg["task"]["loader"])
    return {s: loader.tasks(cfg, s) for s in suites}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True, help="a resolved benchmark config")
    ap.add_argument("--out", required=True, help="where the JSON goes")
    ap.add_argument("--task-suite", action="append", default=None,
                    help="suite to list (repeatable; default: every suite)")
    args = ap.parse_args()
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    result = catalog(cfg, args.task_suite or cfg["task"]["suites"])
    with open(args.out, "w") as f:
        json.dump(result, f)


if __name__ == "__main__":
    main()
