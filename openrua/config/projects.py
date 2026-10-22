"""Workspace identity and the user's explicit trust decisions."""

from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
import tempfile


def resolve(directory: str | Path) -> Path:
    """Resolve this directory, without searching for a parent repository."""
    root = Path(directory).expanduser().resolve()
    if root.exists() and not root.is_dir():
        raise ValueError(f"workspace is not a directory: {root}")
    return root


def _read(path: Path) -> list[str]:
    if not path.exists():
        return []
    value = json.loads(path.read_text())
    if not isinstance(value, list) or not all(isinstance(p, str) for p in value):
        raise ValueError(f"invalid workspace trust file: {path}")
    return value


def trusted(directory: Path, home: Path) -> bool:
    """Trust applies to exactly one canonical directory, never its children."""
    return str(resolve(directory)) in _read(home / "trusted-workspaces.json")


def trust(directory: Path, home: Path) -> None:
    """Remember consent atomically, retaining concurrent trust decisions."""
    root = resolve(directory)
    home.mkdir(parents=True, exist_ok=True)
    target = home / "trusted-workspaces.json"
    with (home / "workspace-trust.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        values = sorted(set(_read(target)) | {str(root)})
        fd, temporary = tempfile.mkstemp(prefix=".workspace-trust-", dir=home)
        try:
            with os.fdopen(fd, "w") as stream:
                json.dump(values, stream, indent=2)
                stream.write("\n")
            os.replace(temporary, target)
        finally:
            Path(temporary).unlink(missing_ok=True)


FIELDS = ('robot', 'sim', 'bench', 'agent', 'model')


def selection(directory: Path) -> dict[str, str]:
    """Non-secret project defaults, read only after the workspace is trusted."""
    path = directory / '.openrua' / 'settings.json'
    if not path.exists():
        return {}
    value = json.loads(path.read_text())
    if not isinstance(value, dict) or set(value) - set(FIELDS) or not all(isinstance(v, str) for v in value.values()):
        raise ValueError(f'invalid project settings: {path}')
    return value


def save_selection(directory: Path, values: dict[str, str]) -> None:
    """Persist the project's environment choice, keeping credentials outside it."""
    metadata = directory / '.openrua'
    if metadata.is_symlink() or (metadata.exists() and not metadata.is_dir()):
        raise ValueError(f'project metadata is not a plain directory: {metadata}')
    metadata.mkdir(parents=True, exist_ok=True)
    target = metadata / 'settings.json'
    if target.is_symlink():
        raise ValueError(f'project settings must not be a symlink: {target}')
    chosen = {key: values[key] for key in FIELDS}
    for key in ('robot', 'sim', 'bench', 'agent'):
        value = chosen[key]
        if value and ('/' in value or (Path(value).suffix and not (key == 'agent' and '@' in value))):
            chosen[key] = str(Path(value).expanduser().resolve())
    fd, temporary = tempfile.mkstemp(prefix='.settings-', dir=metadata)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(chosen, stream, indent=2)
            stream.write('\n')
        os.replace(temporary, target)
    finally:
        Path(temporary).unlink(missing_ok=True)
