"""What ``robocli up`` remembers about a live robot, for ``agent`` and ``down``."""

from __future__ import annotations

from pathlib import Path

import yaml

from robocli.config import paths
from robocli.errors import NotFound

DEFAULT_NAME = "robocli"


def container_names(name: str) -> tuple[str, str]:
    """(robot container, sandbox container) for a handle."""
    return f"{name}-sim", f"{name}-sandbox"


def path(name: str, home: Path) -> Path:
    return paths.state_dir(home) / f"{name}.yaml"


def save(name: str, home: Path, **facts) -> None:
    paths.state_dir(home).mkdir(parents=True, exist_ok=True)
    path(name, home).write_text(yaml.safe_dump(facts, sort_keys=False))


def load(name: str, home: Path) -> dict:
    p = path(name, home)
    if not p.is_file():
        raise NotFound(f"no live robot named {name!r} (nothing at {p})",
                       hint=f"robocli up <robot> --name {name}")
    return yaml.safe_load(p.read_text()) or {}


def forget(name: str, home: Path) -> None:
    path(name, home).unlink(missing_ok=True)
