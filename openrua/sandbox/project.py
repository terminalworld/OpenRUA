"""Persistent project files and fresh, separately mounted robot materials."""

from __future__ import annotations

import fcntl
from pathlib import Path

from openrua.sandbox import SandboxError
from openrua.sandbox.workspace import seed

GUIDANCE = """\
## OpenRUA robot project

This directory is a persistent robot coding project, shared by its sessions.
The current robot's documentation, starter tools and machine.yaml are under
/robot. Start with /robot/README.md. These files describe this connection;
files from earlier sessions describe earlier observations and machines.

Before starting work, read this project's existing instructions, code and
notes. Organize files as you work so a new conversation can continue without
your chat history. A useful structure is scripts/ for reusable project code,
notes/ for findings and progress, and observations/ for saved sensor data.
Use an existing project structure when there is one; these names are suggestions.
Record program entry points, dependencies, relevant machine assumptions,
what was checked, and what remains to do. Keep observations identifiable by
their acquisition time. Get current observations before relying on old state.
Use the project's Git repository for maintained code and notes; keep generated
data separate and choose which artifacts belong in version control.

.openrua/runs/ contains OpenRUA's per-run configuration and robot logs;
keep generated run data separate from maintained project code. Credentials
and private conversation state remain in OpenRUA's user directory. Ending
a session stops its runtime resources and retains this project's files.
Declare dependencies needed to rebuild your programs' environment; a package
installed only inside a container disappears when that container stops.
"""


def prepare(cfg: dict, root: Path, materials: Path) -> None:
    """Preserve project contents; seed current robot materials outside them."""
    root.mkdir(parents=True, exist_ok=True)
    metadata = root / ".openrua"
    if metadata.is_symlink() or (metadata.exists() and not metadata.is_dir()):
        raise SandboxError(f"project metadata is not a plain directory: {metadata}")
    metadata.mkdir(exist_ok=True)
    for path in (metadata / "initialize.lock", root / "AGENTS.md"):
        if path.is_symlink() or (path.exists() and not path.is_file()):
            raise SandboxError(f"project instructions must be regular files: {path}")
    with (metadata / "initialize.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        marker = metadata / "project"
        if marker.is_symlink():
            raise SandboxError(f"project marker must not be a symlink: {marker}")
        if not marker.exists():
            instructions = root / "AGENTS.md"
            with instructions.open("a") as stream:
                stream.write(("\n\n" if instructions.stat().st_size else "") + GUIDANCE)
            marker.write_text("OpenRUA robot project\n")
    seed(cfg, materials)
    for document in materials.rglob('*.md'):
        document.write_text(document.read_text().replace('python3 tools/', 'python3 /robot/tools/'))


def run_directory(root: Path, name: str) -> Path:
    """Allocate a new run directory without replacing previous run records."""
    runs = root / ".openrua" / "runs"
    if runs.is_symlink() or (runs.exists() and not runs.is_dir()):
        raise SandboxError(f"project run records must be a plain directory: {runs}")
    runs.mkdir(parents=True, exist_ok=True)
    for number in range(10000):
        directory = runs / (name if number == 0 else f"{name}-{number}")
        try:
            directory.mkdir()
        except FileExistsError:
            continue
        return directory
    raise SandboxError(f"too many runs for {name!r} under {runs}")
