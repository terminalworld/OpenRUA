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
writer, not just a restarted master: a hand-run ``openrua run`` on a
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
from typing import Iterable

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


def holders(root: Path) -> list[dict]:
    """Every claim under ``root``, live or stale, as ``holder()`` reports
    it plus ``trial`` (the directory, relative to root) and ``live``.
    This is what a person checks before restarting anything that
    launches trials: a stale claim is a crashed attempt, a live one is
    an attempt that will be run over if a second writer starts."""
    running = running_containers()       # one question to docker for all rows
    rows = []
    for path in sorted(Path(root).rglob(LOCK_NAME)):
        info = holder(path.parent) or {}
        rows.append({"trial": str(path.parent.relative_to(root)), **info,
                     "live": is_live(info, running)})
    return rows


def is_live(info: dict, running: Iterable[str] | None = None) -> bool:
    """Whether a lock's holder is still working. Live if its process is
    alive, or if any of its containers are still up (a container name is
    attempt-unique, so it cannot be mistaken for a later attempt's).
    ``running`` is the machine's running container names when the caller
    already holds them; otherwise docker is asked for this stem."""
    pid = info.get("pid")
    if pid and info.get("host") == socket.gethostname() and _pid_alive(int(pid), info.get("since")):
        return True
    stem = info.get("stem")
    if not stem:
        return False
    if running is None:
        return bool(running_containers(stem))
    return any(n.startswith(stem) for n in running)


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


def running_containers(stem: str = "") -> list[str]:
    """Running containers belonging to one attempt (or to one trial key,
    given its prefix; every container when no stem is given). Empty when
    docker cannot answer -- the caller pairs this with a process check,
    and treating an unreachable daemon as proof of death would be the
    one mistake that lets two writers in."""
    try:
        out = subprocess.run(
            ["docker", "ps", "--format", "{{.Names}}",
             "--filter", f"name=^{stem}"],
            capture_output=True, text=True, timeout=30)
    except (subprocess.TimeoutExpired, OSError):
        return []
    return [n for n in out.stdout.split() if n]


def _pid_alive(pid: int, since: float | None = None) -> bool:
    """Whether the process a claim names is still the one that wrote it.
    Pids are reused and a claim can outlive its writer by days, so an
    existing pid holds the claim only if it started before the claim was
    written. Where that cannot be told (no /proc, no ``since``) the claim
    is trusted: treating an unknown process as dead would be the one
    mistake that lets a second writer in."""
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        pass                 # someone else's process, but alive
    started = _process_start(pid)
    if since is None or started is None:
        return True
    return started <= since


def _process_start(pid: int) -> float | None:
    """Unix time the process started, from /proc (None elsewhere)."""
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
        boot = next(int(l.split()[1]) for l in Path("/proc/stat").read_text().splitlines()
                    if l.startswith("btime "))
    except (OSError, StopIteration, ValueError):
        return None
    ticks = int(stat.rsplit(")", 1)[1].split()[19])
    return boot + ticks / os.sysconf("SC_CLK_TCK")


class TrialLocked(RuntimeError):
    """Raised when a live attempt already owns the trial directory."""
