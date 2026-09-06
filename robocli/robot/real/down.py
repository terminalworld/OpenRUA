"""Stop a real robot's launch process started by ``up``.

The process group is remembered by name in this process; ``down(name)``
terminates it. A graph that was already running is left alone.
"""

from __future__ import annotations

import os
import signal
import subprocess

_LAUNCHED: dict[str, subprocess.Popen] = {}


def remember(name: str, proc: subprocess.Popen) -> None:
    _LAUNCHED[name] = proc


def down(name: str) -> None:
    proc = _LAUNCHED.pop(name, None)
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=30)
    except Exception:  # noqa: BLE001
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except OSError:
            pass
