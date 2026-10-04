"""Start a local service process and wait for its existing session API."""

from __future__ import annotations

import fcntl
import os
from pathlib import Path
import subprocess
import time
import threading
from collections.abc import Sequence

from openrua.errors import UnavailableError
from openrua.sessions.client import Client


def connect(endpoint: Path) -> Client:
    """Check liveness using the API, not just the endpoint file."""
    client = Client.from_file(endpoint)
    client.timeout = 3
    snapshot = client.snapshot()
    if snapshot['state']['closed']:
        raise RuntimeError('session is closed; start with a new name')
    return client


def start(argv: Sequence[str], endpoint: Path, log: Path, lock: Path,
          timeout: float = 180) -> Client:
    """Detach a service with explicit arguments; retain logs on startup failure."""
    lock.parent.mkdir(parents=True, exist_ok=True)
    with lock.open('a') as owner:
        try:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise UnavailableError('another client is starting this session',
                                   hint='wait for startup, then reconnect using the same --name') from exc
        # Never create another execution process after an uncertain old startup.
        if endpoint.exists():
            try:
                return connect(endpoint)
            except (OSError, RuntimeError, ValueError, KeyError) as exc:
                raise UnavailableError(str(exc), hint=f'inspect {log}; do not replay tasks automatically') from exc
        log.parent.mkdir(parents=True, exist_ok=True)
        with os.fdopen(os.open(log, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600), 'ab') as output:
            try:
                process = subprocess.Popen(list(argv), stdin=subprocess.DEVNULL, stdout=output,
                                           stderr=subprocess.STDOUT, start_new_session=True)
            except OSError as exc:
                raise UnavailableError(str(exc), hint=f'inspect {log} and repair the OpenRUA installation') from exc
        threading.Thread(target=process.wait, daemon=True).start()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                detail = log.read_text(errors='replace')[-6000:]
                raise UnavailableError(f'session startup exited with status {process.returncode}\n{detail}',
                                       hint=f'inspect {log}; retained session files are not overwritten')
            if endpoint.exists():
                try:
                    return connect(endpoint)
                except (OSError, RuntimeError, ValueError, KeyError):
                    pass
            time.sleep(0.1)
        # A timeout is not proof of failure. Leave the owner to finish or report
        # its own error; the resource reservation prevents duplicate startup.
        raise UnavailableError('session startup is still unresolved; it may still be running',
                               hint=f'inspect {log}; reconnect with the same --name when ready')
