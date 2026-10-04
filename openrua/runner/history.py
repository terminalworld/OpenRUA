"""Discover retained conversations without moving files or restarting execution."""

from __future__ import annotations

import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

from openrua.config import paths
from openrua.sessions.client import Client
from openrua.sessions.store import SQLiteStore


def new_id() -> str:
    return uuid4().hex


def _read(path: Path, operation: str, *args):
    store = SQLiteStore.open_readonly(path)
    try:
        return getattr(store, operation)(*args)
    finally:
        store.close()


def describe(directory: Path) -> dict:
    database = directory / 'conversation.sqlite'
    snapshot = _read(database, 'snapshot')
    state = snapshot['state']
    messages = state['messages']
    title = ' '.join(messages[0]['text'].split())[:80] if messages else 'New robot conversation'
    updated = database.stat().st_mtime
    wal = directory / 'conversation.sqlite-wal'
    if wal.is_file():
        updated = max(updated, wal.stat().st_mtime)
    return dict(id=directory.name, title=title, updated=updated,
                time=datetime.fromtimestamp(updated, timezone.utc).strftime('%Y-%m-%d %H:%M UTC'),
                status='Ended' if state['closed'] else 'Service recorded; verify on open' if (directory / 'endpoint.json').exists() else 'Execution status unknown')


def entries(home: Path) -> list[dict]:
    root = paths.sandboxes_dir(home)
    if not root.exists():
        return []
    records = []
    for directory in root.iterdir():
        if directory.is_symlink() or not directory.is_dir() or not (directory / 'conversation.sqlite').is_file():
            continue
        try:
            records.append(describe(directory))
        except (OSError, sqlite3.Error, ValueError, KeyError, TypeError) as exc:
            records.append(dict(id=directory.name, title='Unreadable retained conversation',
                                updated=0, time='', status=f'Cannot read: {exc}'))
    return sorted(records, key=lambda row: row['updated'], reverse=True)


class ArchiveClient:
    """Read retained conversation state; never imply that its execution is live."""

    read_only = True

    def __init__(self, database: Path, reason: str):
        self.database, self.reason = database, reason
        self.snapshot()

    def snapshot(self) -> dict:
        return _read(self.database, 'snapshot')

    def events(self, after: int = 0, limit: int = 1000) -> list[dict]:
        return _read(self.database, 'events', after, limit)

    def command(self, operation: str, **params):
        raise RuntimeError('retained conversation is read-only; no execution has been restarted')

    def end(self):
        raise RuntimeError('cannot stop an execution owner through retained records')


def open_session(home: Path, name: str):
    directory = paths.sandbox_dir(name, home)
    endpoint = directory / 'endpoint.json'
    try:
        closed = _read(directory / 'conversation.sqlite', 'snapshot')['state']['closed']
    except (sqlite3.Error, OSError, ValueError, KeyError, TypeError) as exc:
        raise RuntimeError(f'cannot read retained conversation {name!r}: {exc}') from exc
    reason = 'Ended conversation' if closed else 'Execution status unknown'
    if endpoint.exists():
        try:
            client = Client.from_file(endpoint)
            client.timeout = 3
            if not client.snapshot()['state']['closed']:
                return client, name
        except (OSError, RuntimeError, ValueError, KeyError):
            reason = 'Service unavailable; execution status unknown'
    return ArchiveClient(directory / 'conversation.sqlite', reason), name
