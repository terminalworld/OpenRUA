"""Single sim-owner thread: every env/sim access funnels through here.

Two reasons this exists (both bitten in practice during P1):

1. mujoco data structures are not thread-safe (a reset racing the
   republish timer segfaulted);
2. the EGL render context is thread-affine; rendering from an rclpy
   executor thread silently returns garbage (exposed by robosuite's
   depth-map assert).

So the bridge runs ONE owner thread (the main thread, which also created
the env and its GL context); ROS callbacks and the control channel submit
jobs instead of touching the env. Jobs run strictly in submission order;
which is also exactly the paused-clock contract (commands are serialized;
one command, one step).
"""

from __future__ import annotations

import queue
import threading
from concurrent.futures import Future
from typing import Any, Callable


class SimJobRunner:
    def __init__(self) -> None:
        self._q: queue.SimpleQueue = queue.SimpleQueue()
        self._owner: threading.Thread | None = None
        self.dropped_errors = 0

    def bind_current_thread(self) -> None:
        """Declare the calling thread as the sim owner (call once, from the
        thread that created the env / GL context)."""
        self._owner = threading.current_thread()

    def submit(self, fn: Callable[[], Any], wait: bool = True):
        """Run ``fn`` on the sim thread. Same-thread calls execute directly
        (jobs may nest); cross-thread calls enqueue. ``wait=True`` blocks
        for (and re-raises from) the result."""
        if threading.current_thread() is self._owner:
            return fn()
        fut: Future = Future()
        self._q.put((fn, fut, wait))
        return fut.result() if wait else fut

    def backlog(self) -> int:
        """Approximate queued-job count (cross-thread jobs not yet started).
        Used by camera publishing to yield to pending commands under load."""
        return self._q.qsize()

    def run_pending(self, timeout: float = 0.1) -> None:
        """Execute one pending job (sim thread's main loop body)."""
        try:
            fn, fut, waited = self._q.get(timeout=timeout)
        except queue.Empty:
            return
        try:
            fut.set_result(fn())
        except BaseException as exc:  # noqa: BLE001; deliver to submitter
            fut.set_exception(exc)
            if not waited:
                # Nobody will read this Future (audit 2026-08-14 F8: the
                # wipe canary's per-step assert died in silence exactly
                # here); surface it in bridge.log instead of vanishing.
                self.dropped_errors += 1
                import sys
                print(f"[simthread] fire-and-forget job failed "
                      f"({self.dropped_errors} total): "
                      f"{type(exc).__name__}: {exc}",
                      file=sys.stderr, flush=True)
