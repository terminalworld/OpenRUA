"""``openrua robots | simulators | benchmarks | agents``: what ships in the package."""

from __future__ import annotations

import json
import subprocess
import tempfile
from pathlib import Path

import yaml

from openrua import agents
from openrua.cli.output import add_json, print_listing
from openrua.config import compose, paths


def _describe(kind: str, path: Path) -> str:
    """One line about a robot profile or a benchmark config, or why it
    could not be read (a bad user file must not hide the rest)."""
    try:
        d = yaml.safe_load(path.read_text()) or {}
    except Exception as exc:  # noqa: BLE001
        return f"(unreadable: {exc})"
    if kind == "robots":
        if "machine" in d:      # an instance: a particular (usually real) robot
            r = d["machine"].get("robot", {})
            kind_ = d["machine"].get("backend", {}).get("kind", "?")
            return f"{r.get('model', '?'):<28} {kind_}: {r.get('description', '')}"
        r = d.get("robot", {})
        return f"{r.get('model', '?'):<28} {r.get('description', '')}"
    if kind == "simulators":
        robots = ", ".join(sorted(d.get("robots", {}))) or "-"
        native = (d.get("native") or {}).get("scene", "-")
        return f"{d.get('engine', '?'):<28} robots: {robots}; native scene: {native}"
    t = d.get("task", {})
    suites = t.get("suites", [])
    return (f"{t.get('benchmark', '?'):<14} {len(suites)} suites; robot: "
            f"{d.get('robot', '(own machine:)')} on {d.get('simulator', '-')}")


def _list_kind(kind: str, args) -> int:
    rows = [{"name": e.name, "path": str(e.path), "description": _describe(kind, e.path)}
            for e in paths.available(kind)]
    print_listing(rows, args.json)
    return 0


def run_robots(args) -> int:
    return _list_kind("robots", args)


def run_simulators(args) -> int:
    return _list_kind("simulators", args)


def run_benchmarks(args) -> int:
    if args.name:
        return _show_benchmark(args)
    return _list_kind("benchmarks", args)


_EPISODES = {
    "benchmark-files": "seed N starts episode N of the benchmark's fixed initial-state files",
    "seeded-reset": "seed N seeds the benchmark's own randomised reset",
    "random-reset": ("the benchmark's reset draws fresh randomness every time; "
                     "seed N only numbers the episode"),
}


def benchmark_catalog(cfg: dict, home: Path | None) -> tuple[dict | None, str]:
    """``{suite: [{task_id, language}] | None}`` from the loader, run in the
    simulator's venv, and a note; (None, why) when it could not be asked."""
    venv = paths.simulator_venv(cfg["machine"]["backend"]["simulator"]["venv"], home)
    python = venv / "bin" / "python"
    if not python.exists():
        return None, (f"task sentences need the simulator install: "
                      f"openrua install --bench {cfg['task']['benchmark']}")
    with tempfile.TemporaryDirectory() as tmp:
        cfg_file, out = Path(tmp) / "config.yaml", Path(tmp) / "tasks.json"
        cfg_file.write_text(yaml.safe_dump(cfg))
        # The answer travels by file: a benchmark's imports print freely.
        r = subprocess.run([str(python), "-m", "openrua.robot.sim.bridge.catalog",
                            "--config", str(cfg_file), "--out", str(out)],
                           capture_output=True, text=True, timeout=600)
        if r.returncode != 0:
            tail = (r.stderr.strip().splitlines() or ["(no output)"])[-1]
            return None, f"the loader could not list tasks: {tail}"
        return json.loads(out.read_text()), ""


def _show_benchmark(args) -> int:
    """One benchmark: its suites, and each suite's tasks with their
    sentences, so --task-suite / --task-ids / --seeds can be chosen by
    reading rather than by trial."""
    composed = compose(None, None, args.name, args.home)
    cfg = composed.cfg
    task, proto = cfg["task"], cfg.get("protocol", {})
    catalog, note = benchmark_catalog(cfg, args.home)
    suites = {suite: (catalog or {}).get(suite) for suite in task["suites"]}
    info = {"name": args.name, "benchmark": task["benchmark"], "robot": composed.robot,
            "simulator": composed.simulator, "init_states": task.get("init_states"),
            "trials_per_task": proto.get("trials_per_task"), "suites": suites, "note": note}
    if args.json:
        print(json.dumps(info, indent=2))
        return 0
    print(f"{args.name:<20} robot {info['robot']} on {info['simulator']}; "
          f"{len(suites)} suites")
    how = _EPISODES.get(info["init_states"] or "", info["init_states"] or "loader-defined")
    n = info["trials_per_task"]
    depth = f"; the reported runs use {n} seed{'s' if n != 1 else ''} per task" if n else ""
    print(f"{'episodes':<20} {how}{depth}")
    for suite, rows in suites.items():
        if rows is None:            # the loader could not be asked; the note says why
            print(suite)
            continue
        print(f"{suite:<20} {len(rows)} task{'s' if len(rows) != 1 else ''}: --task-ids "
              f"{'0' if len(rows) == 1 else f'0-{len(rows) - 1}'}")
        for r in rows:
            print(f"{'':<20}   {r['task_id']:>3}  {r['language'] or '(sentence written at reset)'}")
    if note:
        print(f"{'note':<20} {note}")
    return 0


def run_agents(args) -> int:
    rows = []
    for a in agents.available():
        desc = (f"FAILED: {a.error}" if a.agent is None else
                f"{a.agent.default_model:<24} {' '.join(sorted(a.agent.capabilities))}")
        rows.append({"name": a.name, "path": str(a.path), "description": desc,
                     "capabilities": sorted(a.agent.capabilities) if a.agent else None})
    print_listing(rows, args.json)
    return 0


def add_parser(sub) -> None:
    for kind, fn in (("robots", run_robots), ("simulators", run_simulators),
                     ("benchmarks", run_benchmarks), ("agents", run_agents)):
        p = sub.add_parser(kind, help=f"list the bundled {kind}"
                           + (" (or one benchmark's suites and tasks)" if kind == "benchmarks" else ""),
                           description=f"Every {kind[:-1]} shipped in the package. A file "
                           "of your own is not listed; pass it as a path where a name "
                           "is expected."
                           + (" With a name: that benchmark's suites, each suite's task ids "
                              "and sentences, and what a seed means, for choosing "
                              "--task-suite, --task-ids and --seeds." if kind == "benchmarks" else ""))
        if kind == "benchmarks":
            p.add_argument("name", nargs="?", default=None,
                           help="a benchmark (name or path) to show in full")
        add_json(p)
        p.set_defaults(fn=fn)
