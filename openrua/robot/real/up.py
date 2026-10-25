"""Bring a real robot's graph up, or wait for one already running."""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path

from openrua.robot.base import Handle
from openrua.robot.real.down import container_name, down


class RealHandle(Handle):
    def __init__(self, name: str, proc: subprocess.Popen | None,
                 probe_argv: list[str] | None, container: str | None = None,
                 stop_timeout_s: float = 30.0):
        self.name = name
        self.proc = proc
        self._probe = probe_argv
        self._container = container
        self._stop_timeout_s = stop_timeout_s

    def wait_ready(self, timeout_s: float = 300.0, *, cancelled=None) -> None:
        """Poll ``probe_argv`` until it exits 0 (the graph is visible), or
        raise TimeoutError. Without a probe, return at once."""
        if not self._probe:
            return
        deadline = time.monotonic() + timeout_s
        diagnostic = ""
        while True:
            if cancelled is not None and cancelled():
                raise InterruptedError('robot startup cancelled')
            if self.proc is not None and self.proc.poll() is not None:
                raise TimeoutError(f"launch command exited with {self.proc.returncode} "
                                   "before the graph came up")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"graph not visible after {timeout_s:g}s: {diagnostic}")
            # The probe owns its process group, including a shell's pipeline.
            # A stalled check must not outlive the graph-readiness deadline.
            with subprocess.Popen(self._probe, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                  text=True, start_new_session=True) as probe:
                try:
                    while True:
                        if cancelled is not None and cancelled():
                            raise InterruptedError('robot startup cancelled')
                        remaining = max(0, deadline - time.monotonic())
                        try:
                            stdout, stderr = probe.communicate(timeout=min(.1, remaining) if cancelled else remaining)
                            break
                        except subprocess.TimeoutExpired:
                            if time.monotonic() >= deadline:
                                raise
                except BaseException as exc:
                    try:
                        os.killpg(probe.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    probe.wait(timeout=5)
                    if isinstance(exc, subprocess.TimeoutExpired):
                        output = exc.stderr or exc.output or "readiness probe did not finish"
                        if isinstance(output, bytes):
                            output = output.decode(errors="replace")
                        raise TimeoutError(f"graph not visible after {timeout_s:g}s: "
                                           f"{output.strip()[-300:]}") from exc
                    raise
                # A pre-existing graph can answer while this launch fails.
                # Its response cannot establish that our driver started.
                if self.proc is not None and self.proc.poll() is not None:
                    raise TimeoutError(f"launch command exited with {self.proc.returncode} "
                                       "before the graph came up")
                if probe.returncode == 0:
                    return
                diagnostic = (stderr or stdout).strip()[-300:]
            retry_at = min(deadline, time.monotonic() + 5)
            while time.monotonic() < retry_at:
                if cancelled is not None and cancelled():
                    raise InterruptedError('robot startup cancelled')
                time.sleep(min(.1 if cancelled else 5, max(0, retry_at - time.monotonic())))

    def rpc(self, obj: dict, timeout_note: str = "", timeout_s: float = 900.0, *, cancelled=None) -> dict:
        if cancelled is not None and cancelled():
            raise InterruptedError('robot startup cancelled')
        return {"ok": True, "not_applicable": True, "cmd": obj.get("cmd")}

    def shutdown(self) -> None:
        down(self.proc, self._container, self._stop_timeout_s)

    def stopped(self) -> str | None:
        code = None if self.proc is None else self.proc.poll()
        return None if code is None else f"the launch command exited with {code}"


def launch_argv(name: str, launch: str, image: str | None, ros_domain: int = 0) -> list[str]:
    """The process that runs the launch command: on the host, or inside
    ``image`` on the host network so the driver's ROS release need not
    match the sandbox's."""
    if image:
        return ["docker", "run", "--rm", "--network", "host",
                "--name", container_name(name), "--env", f"ROS_DOMAIN_ID={ros_domain}",
                image, "bash", "-lc", launch]
    return ["bash", "-lc", launch]


def failed_checks(checks: list[dict], cwd: str | None = None,
                  timeout_s: float = 30.0) -> list[dict]:
    """The startup checks that do not hold (each ``{name, run, fix}``),
    every one run, so a person sees all there is to do at once."""
    failed = []
    for check in checks:
        try:
            ok = subprocess.run(["bash", "-c", check["run"]], cwd=cwd, capture_output=True,
                                timeout=timeout_s).returncode == 0
        except subprocess.TimeoutExpired:
            ok = False
        if not ok:
            failed.append(check)
    return failed


def up(name: str, launch: str | None, log_path: Path | None = None,
       probe_argv: list[str] | None = None, image: str | None = None,
       ros_domain: int = 0, cwd: str | None = None,
       stop_timeout_s: float = 30.0) -> RealHandle:
    """Start ``launch`` if given (on the host in ``cwd``, or in ``image``),
    its output streaming into ``log_path``; return the handle.
    ``probe_argv`` is what ``wait_ready`` polls; ``stop_timeout_s`` is how
    long shutdown waits after each signal."""
    proc = None
    if launch:
        if image:
            subprocess.run(["docker", "rm", "-f", container_name(name)], capture_output=True)
        proc = subprocess.Popen(
            launch_argv(name, launch, image, ros_domain), stdin=subprocess.DEVNULL,
            env={**os.environ, "ROS_DOMAIN_ID": str(ros_domain)},
            stdout=(open(log_path, "w") if log_path else subprocess.DEVNULL),
            stderr=subprocess.STDOUT, start_new_session=True,
            cwd=None if image else cwd)
    return RealHandle(name, proc, probe_argv, container_name(name) if launch and image else None,
                      stop_timeout_s)
