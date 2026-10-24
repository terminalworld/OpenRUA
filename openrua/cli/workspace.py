"""Select a project directory and obtain consent before writing into it."""

from pathlib import Path
import sys

from openrua.config import projects
from openrua.errors import UsageError


def choose(directory: str | Path, home: Path, *, accept: bool = False, screen=None) -> Path | None:
    root = projects.resolve(directory)
    if projects.trusted(root, home):
        return root
    if accept:
        root.mkdir(parents=True, exist_ok=True)
        projects.trust(root, home)
        return root
    if not (sys.stdin.isatty() and sys.stdout.isatty()):
        raise UsageError(f"workspace has not been trusted: {root}",
                         hint="open openrua interactively to choose and trust a workspace, "
                              "or pass --workspace <directory> --trust-workspace explicitly")
    from openrua.terminal.launcher import workspace
    selected = workspace(str(root), **({'screen': screen} if screen else {}))
    if selected is None:
        return None
    root = projects.resolve(selected)
    root.mkdir(parents=True, exist_ok=True)
    projects.trust(root, home)
    return root
