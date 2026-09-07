"""One writer per trial directory.

Nothing used to record that a trial key was being worked on: the batch
master kept that only in memory, so a master that was killed left its
trials running as orphans, and the next master relaunched the same keys.
Two processes then shared one directory -- and the second one's first act
is ``record.archive_prior_attempt()``, which moves every artifact aside,
pulling the running attempt's transcript out from under it.

The fact therefore lives on disk, next to the thing it protects. Creation
is O_EXCL, so it is the runner's FIRST act and there is no window in which
two starters both see a free directory. It guards against any second
writer, not just a restarted master: a hand-run ``robocli run`` on a
directory the master owns is the same collision and the same corruption.

Staleness needs care -- a lock nobody can reclaim wedges a trial forever.
A holder counts as gone only when its process is dead AND none of its
containers are running. Container names carry a random per-attempt tag, so
unlike a pid they cannot be confused with something else that came later.
"""
from __future__ import annotations

import json
import os
import socket
import subprocess
import time
from pathlib import Path

LOCK_NAME = ".running"


def holder(trial_dir: Path) -> dict | None:
    """Who claims this trial directory, or None. A lock too damaged to read
    is reported as an unknown holder rather than as no holder: refusing to
    start is recoverable, two writers are not."""
    path = Path(trial_dir) / LOCK_NAME
    try:
        return json.loads(path.read_text())
    except FileNotFoundError:
        return None
    except (OSError, ValueError):
        return {"pid": None, "stem": None, "unreadable": True}


def is_live(info: dict) -> bool:
    """Whether a lock's holder is still working. Live if its process is
    alive, or if any of its containers are still up (a container name is
    attempt-unique, so it cannot be mistaken for a later attempt's)."""
    pid = info.get("pid")
    if pid and _pid_alive(int(pid)) and info.get("host") == socket.gethostname():
        return True
    stem = info.get("stem")
    return bool(stem) and bool(running_containers(stem))


def acquire(trial_dir: Path, stem: str) -> "Lock | None":
    """Claim the directory, or None when a live attempt already holds it.

    A dead holder's lock is taken over: a killed runner (or a rebooted
    host) must not bench its trial permanently.
    """
    path = Path(trial_dir) / LOCK_NAME
    payload = json.dumps({"pid": os.getpid(), "host": socket.gethostname(),
                          "stem": stem, "since": round(time.time(), 1)})
    while True:
        try:
            fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError:
            info = holder(trial_dir)
            if info is None:
                continue          # released between our attempt and the read
            if is_live(info):
                return None
            # Dead holder. Remove ITS lock and retry; whoever wins the next
            # O_EXCL is the single writer.
            try:
                path.unlink()
            except FileNotFoundError:
                pass
            continue
        with os.fdopen(fd, "w") as f:
            f.write(payload)
        return Lock(path)


class Lock:
    """Released by the runner that took it, and by nobody else."""

    def __init__(self, path: Path):
        self.path = path

    def release(self) -> None:
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass


def running_containers(stem: str) -> list[str]:
    """Running containers belonging to one attempt (or to one trial key,
    given its prefix). Empty when docker cannot answer -- the caller pairs
    this with a process check, and treating an unreachable daemon as proof
    of death would be the one mistake that lets two writers in."""
    try:
        out = subprocess.run(
            ["docker", "ps", "--format", "{{.Names}}",
             "--filter", f"name=^{stem}"],
            capture_output=True, text=True, timeout=30)
    except (subprocess.TimeoutExpired, OSError):
        return []
    return [n for n in out.stdout.split() if n]


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True          # someone else's process, but alive
    return True


class TrialLocked(RuntimeError):
    """Raised when a live attempt already owns the trial directory."""
