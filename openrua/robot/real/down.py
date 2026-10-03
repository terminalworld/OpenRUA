"""Stop the launch resources held by a real robot handle.

A graph that was already running has no owned process and is left alone.
No process identity is looked up by name or stored in module globals.
"""

from __future__ import annotations

import os
import signal
import subprocess


def container_name(name: str) -> str:
    return f"{name}-driver"


def down(proc: subprocess.Popen | None, container: str | None = None) -> None:
    failures = []
    if container:
        result = subprocess.run(["docker", "rm", "-f", container], capture_output=True, text=True)
        if result.returncode and "No such container" not in result.stderr:
            failures.append(f"could not remove {container}: {result.stderr.strip()}")
    if proc is not None and proc.poll() is None:
        try:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired) as exc:
            failures.append(f"launch process {proc.pid}: {exc}")
    if failures:
        raise RuntimeError("; ".join(failures) + "; inspect the driver log and retry shutdown")
