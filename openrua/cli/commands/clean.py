"""``openrua clean [--all]``: sweep what sandboxes left in the user directory.

Every sandbox writes under ``<home>/sandboxes/<name>/`` (workspace,
profile copy, state) and removes it when it goes down; a sandbox that
crashed, or a bring-up that failed, leaves its directory behind. This
verb removes every such directory whose containers are not running;
``--all`` powers the running ones off first. ``config.yaml`` and
``credentials/`` are yours and never touched.
"""

from __future__ import annotations

import shutil
import subprocess

from openrua import robot
from openrua.cli import state
from openrua.config import paths
from openrua.sandbox.down import down as sandbox_down


def running(name: str) -> bool:
    """Whether either container of a named sandbox is up."""
    for c in state.container_names(name):
        r = subprocess.run(["docker", "inspect", "--format", "{{.State.Running}}", c],
                           capture_output=True, text=True)
        if r.returncode == 0 and r.stdout.strip() == "true":
            return True
    return False


def run(args) -> int:
    root = paths.sandboxes_dir(args.home)
    if not root.is_dir():
        print(f"nothing under {root}")
        return 0
    kept = 0
    for d in sorted(p for p in root.iterdir() if p.is_dir()):
        if running(d.name):
            if not args.all:
                print(f"[clean] {d.name}: containers running, kept (--all powers them off)")
                kept += 1
                continue
            sim_name, sandbox_name = state.container_names(d.name)
            sandbox_down(sandbox_name)
            robot.down(sim_name, (state.load(d.name, args.home).get("backend", "sim")
                                  if state.path(d.name, args.home).is_file() else "sim"))
        shutil.rmtree(d, ignore_errors=True)
        print(f"[clean] removed {d}")
    if not kept and not any(root.iterdir()):
        root.rmdir()
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("clean", help="remove what sandboxes left under <home>/sandboxes",
                       description=__doc__.split("\n\n")[1])
    p.add_argument("--all", action="store_true",
                   help="power running sandboxes off and remove theirs too")
    p.set_defaults(fn=run)
