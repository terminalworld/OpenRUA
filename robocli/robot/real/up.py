"""Bring a real robot's graph up, or wait for one already running."""

from __future__ import annotations

import subprocess
import time
from pathlib import Path

from robocli.robot.base import Handle
from robocli.robot.real.down import container_name, down, remember


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


def launch_argv(name: str, launch: str, image: str | None) -> list[str]:
    """The process that runs the launch command: on the host, or inside
    ``image`` on the host network so the driver's ROS release need not
    match the sandbox's."""
    if image:
        return ["docker", "run", "--rm", "--network", "host",
                "--name", container_name(name), image, "bash", "-lc", launch]
    return ["bash", "-lc", launch]


def up(name: str, launch: str | None, log_path: Path | None = None,
       probe_argv: list[str] | None = None, image: str | None = None) -> RealHandle:
    """Start ``launch`` if given (on the host, or in ``image``), its output
    streaming into ``log_path``; return the handle. ``probe_argv`` is what
    ``wait_ready`` polls."""
    proc = None
    if launch:
        if image:
            subprocess.run(["docker", "rm", "-f", container_name(name)], capture_output=True)
        proc = subprocess.Popen(
            launch_argv(name, launch, image), stdin=subprocess.DEVNULL,
            stdout=(open(log_path, "w") if log_path else subprocess.DEVNULL),
            stderr=subprocess.STDOUT, start_new_session=True)
        remember(name, proc, container=container_name(name) if image else None)
    return RealHandle(name, proc, probe_argv)
