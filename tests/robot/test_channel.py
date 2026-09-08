"""ControlChannel unit contract (pure python; no rclpy, no container).

The private stdio line's survival rules: report-not-die on handler
errors, request-id echo, and EVERY exit path ending in on_eof (the
no-orphans guarantee) -- including the answer pipe breaking mid-rpc
(a hard-killed runner must not strand the sim).
"""

from __future__ import annotations

import io
import json
import sys

from openrua.robot.sim.bridge.rpc import ControlChannel


class _Out:
    def __init__(self, fail_after=None):
        self.lines = []
        self._fail_after = fail_after

    def write(self, s):
        if self._fail_after is not None and len(self.lines) >= self._fail_after:
            raise BrokenPipeError("runner died")
        self.lines.append(s)

    def flush(self):
        pass


def _run(lines, handlers, out=None):
    out = out or _Out()
    eof = []
    ch = ControlChannel(handlers, on_eof=lambda: eof.append(True), out=out)
    old = sys.stdin
    sys.stdin = io.StringIO("".join(l + "\n" for l in lines))
    try:
        ch._loop()
    finally:
        sys.stdin = old
    return [json.loads(s) for s in out.lines], eof


def test_dispatch_answers_and_echoes_request_id():
    answers, eof = _run(
        [json.dumps({"cmd": "success", "id": 7}), ""],
        {"success": lambda req: {"success": True}})
    assert answers == [{"ok": True, "success": True, "id": 7}]
    assert eof == [True]  # stdin EOF -> no-orphans shutdown


def test_handler_error_reports_and_loop_survives():
    def boom(req):
        raise RuntimeError("sim wedged")

    answers, eof = _run(
        [json.dumps({"cmd": "reset"}), json.dumps({"cmd": "ok"})],
        {"reset": boom, "ok": lambda req: {}})
    assert answers[0]["ok"] is False and "sim wedged" in answers[0]["error"]
    assert answers[1] == {"ok": True}  # next rpc still served
    assert eof == [True]


def test_unknown_cmd_reports_not_dies():
    answers, eof = _run([json.dumps({"cmd": "nope", "id": 1})], {})
    assert answers[0]["ok"] is False and answers[0]["id"] == 1
    assert eof == [True]


def test_broken_answer_pipe_still_reaches_on_eof():
    # Runner hard-killed mid-rpc: writing the answer raises. The loop
    # must break to the SAME shutdown path, never strand the sim.
    out = _Out(fail_after=0)
    answers, eof = _run(
        [json.dumps({"cmd": "ok"}), json.dumps({"cmd": "ok"})],
        {"ok": lambda req: {}}, out=out)
    assert answers == [] and eof == [True]
