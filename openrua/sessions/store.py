"""Transactional session state and an ordered, reconnectable event journal.

Storage is supplied to the session through the Store protocol. SQLiteStore is
the local implementation; clients never access its SQL or database connection.
"""

from __future__ import annotations

import json
import os
import sqlite3
import threading
import time
from pathlib import Path
from typing import Callable, Protocol, Any

Change = Callable[[dict], tuple[Any, list[dict]]]


class Store(Protocol):
    def update(self, change: Change) -> Any:
        """Commit state and emitted events together, or change neither."""
        ...

    def snapshot(self) -> dict: ...

    def events(self, after: int = 0, limit: int = 1000) -> list[dict]: ...


class SQLiteStore:
    def __init__(self, path: Path, initial: dict):
        path.parent.mkdir(parents=True, exist_ok=True)
        # Create privately before SQLite opens it; journals inherit its mode.
        fd = os.open(path, os.O_WRONLY | os.O_CREAT, 0o600)
        os.close(fd)
        path.chmod(0o600)
        self._lock = threading.RLock()
        self._db = sqlite3.connect(path, isolation_level=None, timeout=30, check_same_thread=False)
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA synchronous=FULL")
        self._db.execute("CREATE TABLE IF NOT EXISTS state (id INTEGER PRIMARY KEY, body TEXT NOT NULL)")
        self._db.execute("CREATE TABLE IF NOT EXISTS events (seq INTEGER PRIMARY KEY AUTOINCREMENT, "
                         "at REAL NOT NULL, body TEXT NOT NULL)")
        self._db.execute("INSERT OR IGNORE INTO state VALUES (1, ?)", (json.dumps(initial),))

    @classmethod
    def open_readonly(cls, path: Path) -> SQLiteStore:
        """Read retained records without creating a database or modifying its schema."""
        instance = cls.__new__(cls)
        instance._lock = threading.RLock()
        instance._db = sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True,
                                       isolation_level=None, timeout=30, check_same_thread=False)
        return instance

    def update(self, change: Change) -> Any:
        with self._lock:
            self._db.execute("BEGIN IMMEDIATE")
            try:
                state = json.loads(self._db.execute("SELECT body FROM state WHERE id=1").fetchone()[0])
                result, events = change(state)
                self._db.execute("UPDATE state SET body=? WHERE id=1", (json.dumps(state),))
                self._db.executemany("INSERT INTO events(at,body) VALUES (?,?)",
                                     [(time.time(), json.dumps(event)) for event in events])
                self._db.execute("COMMIT")
                return result
            except BaseException:
                self._db.execute("ROLLBACK")
                raise

    def snapshot(self) -> dict:
        with self._lock:
            self._db.execute("BEGIN")
            try:
                state = json.loads(self._db.execute("SELECT body FROM state WHERE id=1").fetchone()[0])
                cursor = self._db.execute("SELECT COALESCE(MAX(seq),0) FROM events").fetchone()[0]
                self._db.execute("COMMIT")
                return {"state": state, "cursor": cursor}
            except BaseException:
                self._db.execute("ROLLBACK")
                raise

    def events(self, after: int = 0, limit: int = 1000) -> list[dict]:
        if after < 0 or not 1 <= limit <= 1000:
            raise ValueError("cursor must be nonnegative and limit must be 1..1000")
        with self._lock:
            rows = self._db.execute("SELECT seq,at,body FROM events WHERE seq>? ORDER BY seq LIMIT ?",
                                    (after, limit)).fetchall()
            return [{"seq": seq, "at": at, **json.loads(body)} for seq, at, body in rows]

    def export(self, *, max_bytes: int, max_events: int) -> dict:
        """Export one bounded, consistent state and recent event history.

        A readonly caller can inspect closed records without opening their
        execution endpoint. Truncation is explicit; event order stays ascending.
        """
        if min(max_bytes, max_events) <= 0:
            raise ValueError('export limits must be positive')
        with self._lock:
            self._db.execute('BEGIN')
            try:
                size = self._db.execute('SELECT length(CAST(body AS BLOB)) FROM state WHERE id=1').fetchone()[0]
                if size > max_bytes:
                    raise ValueError('conversation state exceeds the per-record input limit')
                state = json.loads(self._db.execute('SELECT body FROM state WHERE id=1').fetchone()[0])
                cursor = self._db.execute('SELECT COALESCE(MAX(seq),0) FROM events').fetchone()[0]
                events, truncated = [], False
                used = len(json.dumps({'state': state, 'cursor': cursor, 'events': [], 'truncated': False},
                                      ensure_ascii=False, separators=(',', ':')).encode())
                if used > max_bytes:
                    raise ValueError('conversation state exceeds the per-record input limit')
                rows = self._db.execute('SELECT seq,at,length(CAST(body AS BLOB)) FROM events '
                                        'WHERE seq<=? ORDER BY seq DESC LIMIT ?', (cursor, max_events + 1)).fetchall()
                for seq, at, length in rows:
                    if len(events) >= max_events or length > max_bytes - used:
                        truncated = True
                        break
                    body = self._db.execute('SELECT body FROM events WHERE seq=?', (seq,)).fetchone()[0]
                    event = {'seq': seq, 'at': at, **json.loads(body)}
                    cost = len(json.dumps(event, ensure_ascii=False, separators=(',', ':')).encode()) + bool(events)
                    if used + cost > max_bytes:
                        truncated = True
                        break
                    events.append(event)
                    used += cost
                self._db.execute('COMMIT')
                return {'state': state, 'cursor': cursor, 'events': events[::-1], 'truncated': truncated}
            except BaseException:
                self._db.execute('ROLLBACK')
                raise

    def close(self) -> None:
        with self._lock:
            self._db.close()
