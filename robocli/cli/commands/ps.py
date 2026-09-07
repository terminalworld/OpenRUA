"""``robocli ps``: the attempts claimed under a runs root, live or stale."""

from __future__ import annotations

import json
import time
from pathlib import Path

from robocli.cli.output import add_json
from robocli.runner import lock
from robocli.runner.main import RUNS_ROOT


def run(args) -> int:
    root = Path(args.runs_root).expanduser() if args.runs_root else RUNS_ROOT
    rows = lock.holders(root) if root.is_dir() else []
    if not args.all:
        rows = [r for r in rows if r["live"]]
    if args.json:
        print(json.dumps(rows, indent=2))
        return 0
    if not rows:
        print(f"no {'claims' if args.all else 'live attempts'} under {root}")
        return 0
    now = time.time()
    print(f"{'TRIAL':<60} {'PID':>8} {'AGE':>7}  STATE     CONTAINERS")
    for r in rows:
        age = f"{(now - r['since']) / 60:.0f}m" if r.get("since") else "?"
        state = "live" if r["live"] else "stale"
        print(f"{r['trial']:<60} {str(r.get('pid') or '?'):>8} {age:>7}  {state:<9} {r.get('stem') or ''}")
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser(
        "ps", help="list the attempts running under a runs root (--all: stale claims too)",
        description="Every trial directory that carries an attempt's claim: the "
        "process, when it started, whether it is still working (process alive, or "
        "its containers running), and its container name stem. Check this before "
        "restarting anything that launches trials; a live attempt is run over if a "
        "second writer starts on its directory. A stale claim is a crashed attempt "
        "and is taken over by the next run.")
    p.add_argument("--all", "-a", action="store_true",
                   help="also list stale claims (crashed attempts, and claims archived under attempts/)")
    p.add_argument("--runs-root", default=None,
                   help=f"where runs live (default: {RUNS_ROOT}, as for robocli run)")
    add_json(p)
    p.set_defaults(fn=run)
