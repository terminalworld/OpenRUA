"""Select only completed work recorded for one exact project directory."""

from dataclasses import dataclass
import json
from pathlib import Path
import os
import sqlite3
from typing import Callable

import yaml

from openrua.artifacts import WorkspaceFiles
from openrua.config import paths
from openrua.errors import NotFound
from openrua.runner import live_state
from openrua.sessions.store import SQLiteStore


@dataclass
class Source:
    name: str
    updated: float
    load: Callable[[], dict[str, bytes]]


def _conversation(path: Path, max_bytes: int, max_events: int, project: Path, home: Path, name: str) -> dict[str, bytes]:
    def owner():
        try:
            value = live_state.load(name, home, require_running=False).get('project')
        except yaml.YAMLError as exc:
            raise ValueError('conversation metadata changed or became unreadable') from exc
        return Path(value).resolve() if isinstance(value, str) and Path(value).is_absolute() else None
    if owner() != project:
        raise ValueError('conversation project ownership changed during selection')
    store = SQLiteStore.open_readonly(path)
    try:
        value = store.export(max_bytes=max_bytes, max_events=max_events)
    finally:
        store.close()
    if not value['state']['closed']:
        raise ValueError('conversation is active; only closed records are eligible')
    if owner() != project:
        raise ValueError('conversation project ownership changed during capture')
    content = json.dumps(value, ensure_ascii=False, separators=(',', ':')).encode()
    return {'conversation.json': content}


def select(project: Path, home: Path, record_roots: tuple[Path, ...], *, since: float | None,
           limit: int, max_file_bytes: int, max_events: int) -> tuple[list[Source], list[dict]]:
    """Selection reads metadata and readonly SQLite; it never contacts owners."""
    sources, skipped = [], []
    sandboxes = paths.sandboxes_dir(home)
    if sandboxes.is_dir():
        for directory in sandboxes.iterdir():
            if directory.is_symlink() or not directory.is_dir():
                continue
            try:
                facts = live_state.load(directory.name, home, require_running=False)
            except (NotFound, OSError, ValueError, yaml.YAMLError):
                continue
            owner = facts.get('project')
            if not isinstance(owner, str) or not Path(owner).is_absolute() or Path(owner).resolve() != project:
                continue
            database = directory / 'conversation.sqlite'
            if not database.is_file() or database.is_symlink():
                continue
            updated = max(path.stat().st_mtime for path in (database, directory / 'conversation.sqlite-wal') if path.exists())
            if since is not None and updated < since:
                continue
            try:
                store = SQLiteStore.open_readonly(database)
                try:
                    closed = store.export(max_bytes=max_file_bytes, max_events=1)['state']['closed']
                finally:
                    store.close()
                if not closed:
                    skipped.append({'source': 'sessions/' + directory.name, 'reason': 'conversation is active'})
                    continue
            except (OSError, ValueError, sqlite3.Error, KeyError) as exc:
                skipped.append({'source': 'sessions/' + directory.name, 'reason': str(exc)})
                continue
            source = Source('sessions/' + directory.name, updated,
                            lambda database=database, name=directory.name: _conversation(database, max_file_bytes, max_events, project, home, name))
            sources.append(source)
    seen = set()
    for root_index, root in enumerate(record_roots):
        if not root.is_dir():
            continue
        reader = WorkspaceFiles(root, max_file_bytes)
        for folder, directories, files in os.walk(root, followlinks=False):
            directories[:] = [name for name in directories if name not in {'frames', 'workspace', 'profile'}
                               and not (Path(folder) / name).is_symlink()]
            if 'result.json' not in files:
                continue
            result = Path(folder) / 'result.json'
            if result.resolve() in seen:
                continue
            seen.add(result.resolve())
            relative = result.relative_to(root).as_posix()
            try:
                item = reader.read(relative)
                value = json.loads(item.get('text', ''))
                owner = value.get('project')
                if not isinstance(owner, str) or not Path(owner).is_absolute() or Path(owner).resolve() != project:
                    continue
                updated = result.stat().st_mtime
                if since is not None and updated < since:
                    continue
                parent = result.parent.relative_to(root).as_posix()
                name = f'exec/{root_index}/' + (parent if parent != '.' else 'root')
                def load(folder=Path(folder), reader=reader, root=root, relative=relative, expected=item['text']):
                    if reader.read(relative).get('text') != expected:
                        raise ValueError('exec record changed during selection')
                    output = {}
                    # Native transcript contents stay data; no vendor parsing here.
                    for name in ('result.json', 'transcript.jsonl', 'commands.sh', 'ops.jsonl', 'prompt.txt'):
                        path = folder / name
                        if path.is_file():
                            item = reader.read(path.relative_to(root).as_posix())
                            if item['kind'] == 'text':
                                output[name] = item['text'].encode()
                    if reader.read(relative).get('text') != expected:
                        raise ValueError('exec record changed during capture')
                    return output
                sources.append(Source(name, updated, load))
            except (OSError, ValueError, sqlite3.Error) as exc:
                skipped.append({'source': str(result), 'reason': str(exc)})
    ordered = sorted(sources, key=lambda source: source.updated, reverse=True)
    skipped.extend({'source': source.name, 'reason': 'completed-record limit reached'} for source in ordered[limit:])
    return ordered[:limit], skipped
