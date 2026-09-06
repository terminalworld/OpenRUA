"""Bring a real robot's graph up, or wait for one already running."""

from __future__ import annotations

import subprocess
import time
from pathlib import Path

from robocli.robot.base import Handle
from robocli.robot.real.down import down, remember


class RealHandle(Handle):
    def __init__(self, name: str, proc: subprocess.Popen | None,
                 probe_argv: list[str] | None):
        self.name = name
        self.proc = proc
        self._probe = probe_argv

    def wait_ready(self, timeout_s: float = 300.0) -> None:
        """Poll ``probe_argv`` until it exits 0 (the graph is visible), or
        raise TimeoutError. Without a probe, return at once."""
        if not self._probe:
            return
        deadline = time.time() + timeout_s
        while True:
            if self.proc is not None and self.proc.poll() is not None:
                raise TimeoutError(f"launch command exited with {self.proc.returncode} "
                                   "before the graph came up")
            r = subprocess.run(self._probe, capture_output=True, text=True)
            if r.returncode == 0:
                return
            if time.time() >= deadline:
                raise TimeoutError(f"graph not visible after {timeout_s:.0f}s: "
                                   f"{(r.stderr or r.stdout).strip()[-300:]}")
            time.sleep(5)

    def rpc(self, obj: dict, timeout_note: str = "", timeout_s: float = 900.0) -> dict:
        return {"ok": True, "not_applicable": True, "cmd": obj.get("cmd")}

    def shutdown(self) -> None:
        down(self.name)


def up(name: str, launch: str | None, log_path: Path | None = None,
       probe_argv: list[str] | None = None) -> RealHandle:
    """Start ``launch`` (a shell command) if given, its output streaming
    into ``log_path``; return the handle. ``probe_argv`` is what
    ``wait_ready`` polls."""
    proc = None
    if launch:
        proc = subprocess.Popen(
            ["bash", "-lc", launch], stdin=subprocess.DEVNULL,
            stdout=(open(log_path, "w") if log_path else subprocess.DEVNULL),
            stderr=subprocess.STDOUT, start_new_session=True)
        remember(name, proc)
    return RealHandle(name, proc, probe_argv)
