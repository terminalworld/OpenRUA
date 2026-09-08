"""``openrua robots | benchmarks | agents``: what is available, bundled and yours."""

from __future__ import annotations

from pathlib import Path

import yaml

from openrua import agents
from openrua.cli.output import add_json, print_listing
from openrua.config import paths


def _describe(kind: str, path: Path) -> str:
    """One line about a robot profile or a benchmark config, or why it
    could not be read (a bad user file must not hide the rest)."""
    try:
        d = yaml.safe_load(path.read_text()) or {}
    except Exception as exc:  # noqa: BLE001
        return f"(unreadable: {exc})"
    if kind == "robots":
        r = d.get("machine", {}).get("robot", {})
        return f"{r.get('model', '?'):<28} {r.get('description', '')}"
    t = d.get("task", {})
    suites = t.get("suites", [])
    return f"{t.get('benchmark', '?'):<14} {len(suites)} suites; robot: {d.get('robot', '(own machine:)')}"


def _list_kind(kind: str, args) -> int:
    rows = [{"name": e.name, "source": e.source, "path": str(e.path),
             "description": _describe(kind, e.path),
             "shadowed_by": str(e.shadowed_by) if e.shadowed_by else None}
            for e in paths.available(kind, args.home)]
    print_listing(rows, args.json)
    return 0


def run_robots(args) -> int:
    return _list_kind("robots", args)


def run_benchmarks(args) -> int:
    return _list_kind("benchmarks", args)


def run_agents(args) -> int:
    rows = []
    for a in agents.available(args.home):
        desc = (f"FAILED: {a.error}" if a.agent is None else
                f"{a.agent.default_model:<24} {' '.join(sorted(a.agent.capabilities))}")
        rows.append({"name": a.name, "source": a.source, "path": str(a.path),
                     "description": desc,
                     "shadowed_by": str(a.shadowed_by) if a.shadowed_by else None,
                     "capabilities": sorted(a.agent.capabilities) if a.agent else None})
    print_listing(rows, args.json)
    return 0


def add_parser(sub) -> None:
    for kind, fn in (("robots", run_robots), ("benchmarks", run_benchmarks),
                     ("agents", run_agents)):
        p = sub.add_parser(kind, help=f"list the {kind}: bundled, then ~/.openrua/{kind}/",
                           description=f"Every {kind[:-1]} OpenRUA can find: the bundled "
                           f"ones, then yours under <home>/{kind}/. A user file that "
                           "carries a bundled name is reported and not used.")
        add_json(p)
        p.set_defaults(fn=fn)
