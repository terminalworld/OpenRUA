"""The host end of the bridge's control line: one question, one answer.

The bridge process runs in the container with stdin/stdout piped to the
host. The runner asks on stdin (one JSON object per line) and reads the
answer on stdout; stderr streams into the trial's bridge.log. The pipe
is held by the parent alone: no network endpoint exists for the agent to
find, and when the parent exits the bridge sees EOF and shuts itself
down.
"""

from __future__ import annotations

import json
import os
import select
import subprocess
import sys
import threading
import time
import uuid

from robocli.robot.base import Handle
from robocli.robot.sim.down import down


class BridgeClient(Handle):
    def __init__(self, proc: subprocess.Popen, name: str):
        self.name = name
        self.proc = proc
        # The watchdog probe thread and the main thread share one pipe;
        # write+read must be atomic or a stale probe could take the
        # final verdict.
        self._rpc_lock = threading.Lock()
        self._rx = b""

    def _read_line(self, deadline: float, note: str) -> str:
        """One line via raw-fd reads (a TextIO buffer would hide bytes
        from select and fake a timeout)."""
        fd = self.proc.stdout.fileno()
        while b"\n" not in self._rx:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise RuntimeError(f"bridge rpc timeout {note}")
            ready, _, _ = select.select([fd], [], [], remaining)
            if not ready:
                raise RuntimeError(f"bridge rpc timeout {note}")
            chunk = os.read(fd, 65536)
            if not chunk:
                raise RuntimeError(f"bridge died mid-rpc {note}")
            self._rx += chunk
        line, _, self._rx = self._rx.partition(b"\n")
        return line.decode()

    def rpc(self, obj: dict, timeout_note: str = "", timeout_s: float = 900.0) -> dict:
        """One request/response, serialized, bounded, id-matched: a late
        answer to an abandoned (timed-out) request is discarded instead
        of being taken for the next reply."""
        assert self.proc.stdin and self.proc.stdout
        rid = uuid.uuid4().hex[:12]
        deadline = time.time() + timeout_s
        note = f"{obj.get('cmd')} {timeout_note}"
        with self._rpc_lock:
            self.proc.stdin.write(json.dumps({**obj, "id": rid}) + "\n")
            self.proc.stdin.flush()
            while True:
                resp = json.loads(self._read_line(deadline, note))
                if resp.get("id") in (rid, None):
                    return resp
                print(f"[rpc] drained stale answer (id {resp.get('id')}) "
                      f"while waiting for {note}", file=sys.stderr, flush=True)

    def wait_ready(self, timeout_s: float = 1800.0) -> None:
        """The control line answers once the whole robot is up (env,
        graph and MoveIt). The bound is generous: concurrent software
        renders legitimately take minutes; it turns a dead boot into a
        clean error, not a speed limit."""
        try:
            if self.rpc({"cmd": "success"}, "boot", timeout_s=timeout_s).get("ok"):
                return
        except (RuntimeError, json.JSONDecodeError) as e:
            raise TimeoutError(f"bridge not ready: {e}") from e
        raise TimeoutError("bridge not ready: control line answered not-ok")

    def shutdown(self) -> None:
        """Ask the bridge to stop; if it does not answer in seconds,
        remove the container."""
        try:
            self.rpc({"cmd": "shutdown"}, timeout_s=15.0)
            self.proc.wait(timeout=30)
        except Exception:  # noqa: BLE001
            down(self.name)
