"""Private control channel: evaluator <-> bridge, never on the graph.

The runner talks to the bridge process over its stdio: one JSON object per
line on stdin, one JSON reply per line on stdout (logs go to stderr).
This channel carries exactly the truth-side verbs; reset to a fixed init
state, ask the original predicate, shutdown; and is physically
unreachable from the agent sandbox (no exec path into the sim container, and
none of this touches DDS).

Protocol:
    {"cmd": "reset", "init_state_id": 3}   -> {"ok": true}
    {"cmd": "success"}                     -> {"ok": true, "success": false}
    {"cmd": "shutdown"}                    -> {"ok": true}  (then exit)
"""

from __future__ import annotations

import json
import sys
import threading
from typing import Callable


class ControlChannel:
    def __init__(
        self,
        handlers: dict[str, Callable[[dict], dict]],
        on_eof: Callable[[], None],
        out=None,
    ):
        self._handlers = handlers
        self._on_eof = on_eof  # runner gone -> bridge exits cleanly (no orphans)
        # `out` is the PRIVATE handle to the real stdout (see boot.main's fd
        # redirection); simulator prints can never pollute the channel.
        self._out = out if out is not None else sys.stdout
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _loop(self) -> None:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            rid = None
            try:
                req = json.loads(line)
                rid = req.get("id")
                handler = self._handlers[req["cmd"]]
                resp = {"ok": True, **(handler(req) or {})}
            except Exception as exc:  # noqa: BLE001; report, never die
                resp = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
            if rid is not None:
                # Echoed request id (2026-08-14 diff-review F-A): lets the
                # evaluator discard late answers to abandoned (timed-out)
                # requests instead of mistaking them for the next reply.
                resp["id"] = rid
            try:
                self._out.write(json.dumps(resp) + "\n")
                self._out.flush()
            except (OSError, ValueError):
                # Runner died mid-rpc: the answer pipe broke before stdin
                # reported EOF. Same meaning, same exit; an unhandled
                # BrokenPipeError here would kill this thread and skip
                # the no-orphans shutdown below.
                break
        self._on_eof()
