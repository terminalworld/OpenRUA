"""Stop a real robot's launch process started by ``up``.

The process (and its driver container, when the launch ran in one) is
remembered by name in this process; ``down(name)`` stops it. A graph that
was already running is left alone.
"""

from __future__ import annotations

import os
import signal
import subprocess

_LAUNCHED: dict[str, tuple[subprocess.Popen, str | None]] = {}


def container_name(name: str) -> str:
    return f"{name}-driver"


def remember(name: str, proc: subprocess.Popen, container: str | None = None) -> None:
    _LAUNCHED[name] = (proc, container)


def down(name: str) -> None:
    proc, container = _LAUNCHED.pop(name, (None, None))
    if container:
        subprocess.run(["docker", "rm", "-f", container], capture_output=True)
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
