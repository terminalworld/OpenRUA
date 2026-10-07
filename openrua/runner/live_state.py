"""Retained session facts, separate from whether its robot is running."""

from __future__ import annotations

import os
from pathlib import Path
import tempfile

import yaml

from openrua.config import paths
from openrua.errors import NotFound, UsageError

DEFAULT_NAME = "openrua"


def container_names(name: str) -> tuple[str, str]:
    """(robot container, sandbox container) for a handle."""
    return f"{name}-sim", f"{name}-sandbox"


def path(name: str, home: Path) -> Path:
    return paths.sandbox_dir(name, home) / "state.yaml"


def save(name: str, home: Path, **facts) -> None:
    """Replace retained facts only after the complete new snapshot is written."""
    target = path(name, home)
    target.parent.mkdir(parents=True, exist_ok=True)
    content = yaml.safe_dump(facts, sort_keys=False)
    fd, temporary = tempfile.mkstemp(prefix=".state-", dir=target.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(content)
        os.replace(temporary, target)
    finally:
        Path(temporary).unlink(missing_ok=True)


def load(name: str, home: Path, *, require_running: bool = True) -> dict:
    p = path(name, home)
    if not p.is_file():
        raise NotFound(f"no live robot named {name!r} (nothing at {p})",
                       hint=f"openrua up <robot> --name {name}")
    facts = yaml.safe_load(p.read_text()) or {}
    if require_running and facts.get("status", "running") != "running":
        raise NotFound(f"session {name!r} is stopped; its files remain at {p.parent}",
                       hint="start a new session with openrua up <robot> --name <new-name>")
    return facts



def reserve(name: str, home: Path) -> None:
    """Claim a new handle without overwriting retained or active materials."""
    directory = paths.sandbox_dir(name, home)
    directory.parent.mkdir(parents=True, exist_ok=True)
    try:
        directory.mkdir()
    except FileExistsError as exc:
        raise UsageError(
            f"session directory already exists: {directory}",
            hint=f"use --name <new-name> to keep it, or explicitly delete it with "
                 f"openrua --home {home} clean --name {name}") from exc


def stopped(name: str, home: Path) -> None:
    """Mark a stopped session while retaining its files and native agent profile."""
    if path(name, home).is_file():
        facts = load(name, home, require_running=False)
        facts["status"] = "stopped"
        save(name, home, **facts)
