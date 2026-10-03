"""``openrua clean [--name <name>] [--all]``: explicitly delete session files.

Sessions retain their workspace, native agent profile and state after
shutdown. This command deletes those materials under ``<home>/sandboxes/``:
by default all stopped sessions, or just ``--name``. ``--all`` also stops
running containers before deletion. User-supplied workspace directories
outside the session directory, ``config.yaml`` and ``credentials/`` are
never deleted.
"""

from __future__ import annotations

import shutil
import subprocess

from openrua import robot
from openrua.cli import state
from openrua.config import paths
from openrua.errors import UnavailableError, UsageError
from openrua.sandbox.down import down as sandbox_down


def running(name: str) -> bool:
    """Whether either session container is up; an unreachable daemon is an error."""
    result = subprocess.run(["docker", "ps", "--format", "{{.Names}}"],
                            capture_output=True, text=True)
    if result.returncode:
        raise UnavailableError(result.stderr.strip() or "cannot inspect running containers",
                               hint="check docker info before deleting session files")
    return bool(set(state.container_names(name)) & set(result.stdout.splitlines()))


def run(args) -> int:
    root = paths.sandboxes_dir(args.home)
    selected = paths.sandbox_dir(args.name, args.home) if args.name else None
    if not root.is_dir():
        print(f"nothing under {root}")
        return 0
    directories = ([selected] if selected else sorted(p for p in root.iterdir() if p.is_dir()))
    for directory in directories:
        if directory.is_symlink():
            raise UsageError(f"refusing to delete a session symlink: {directory}",
                             hint="inspect the link and remove it explicitly if intended")
        if not directory.exists():
            print(f"nothing at {directory}")
            continue
        if (directory / "endpoint.json").exists():
            raise UnavailableError(
                f"session {directory.name!r} has a shared service endpoint; files kept",
                hint=f"openrua session --name {directory.name} end; if the host crashed, inspect its resources before removing the stale endpoint")
        if running(directory.name):
            if not args.all:
                print(f"[clean] {directory.name}: containers running, kept (--all powers them off)")
                continue
            sim_name, sandbox_name = state.container_names(directory.name)
            sandbox_down(sandbox_name)
            kind = (state.load(directory.name, args.home, require_running=False).get("backend", "sim")
                    if state.path(directory.name, args.home).is_file() else "sim")
            robot.down(sim_name, kind)
            if running(directory.name):
                raise UnavailableError(f"session {directory.name!r} still has running containers",
                                       hint=f"openrua down --name {directory.name}")
        shutil.rmtree(directory)
        print(f"[clean] removed {directory}")
    if not any(root.iterdir()):
        root.rmdir()
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("clean", help="explicitly delete retained session files",
                       description=__doc__.split("\n\n")[1])
    p.add_argument("--name", default=None,
                   help="delete only this session (default: all stopped sessions)")
    p.add_argument("--all", action="store_true",
                   help="also power running containers off and delete their session files")
    p.set_defaults(fn=run)
