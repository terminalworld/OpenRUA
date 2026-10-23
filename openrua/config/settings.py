"""Update shared user defaults without replacing unrelated configuration."""

from __future__ import annotations

import fcntl
import os
from pathlib import Path
import tempfile

import yaml

from openrua.config import paths
from openrua.config.loader import load_user_config, load_yaml, validate
from openrua.config.schema import UserConfig


def update(home: Path, names: dict, facts: dict | None = None) -> dict:
    """Merge selected names and agent facts into the validated defaults file."""
    path = paths.config_path(home)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        load_user_config(path)
        data = (load_yaml(path) if path.is_file() else {}) or {}
        # A file of your own is saved as a full path: a relative one would
        # name a different file, or none, from another directory.
        data.update({key: str(Path(value).expanduser().resolve())
                     if isinstance(value, str) and paths.is_path(value) else value
                     for key, value in names.items()})
        if facts:
            agent = data.get('agent') or load_user_config(paths.package_config_path()).agent
            data.setdefault('agents', {}).setdefault(agent, {}).update(facts)
        validate(UserConfig, data, path)
        fd, temporary = tempfile.mkstemp(prefix='.config-', dir=path.parent)
        try:
            with os.fdopen(fd, 'w') as stream:
                yaml.safe_dump(data, stream, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            Path(temporary).unlink(missing_ok=True)
    return data
