"""Who acts on the robot during a trial.

- ``none``: nobody acts (plumbing tests; trials score false).
- ``script``: a caller-supplied command file runs inside the sandbox on
  the native surface (the same shell the agent gets; no privileged
  verbs), one operation at a time. Canonical source: a trial's
  commands.sh. Open-loop replay.
- ``agent``: the headless agent (``session.agent_operator``).

An operator takes the trial context dict and returns the metadata
recorded under ``operator_meta``.
"""

from __future__ import annotations

import re
from typing import NamedTuple
import subprocess
import threading
import time
from pathlib import Path

from openrua.runner import record
from openrua.runner.session import agent_operator


def none_operator(ctx: dict) -> dict:
    return {"operator": "none"}


_MARKER = re.compile(rf"^{re.escape(record.OP_MARKER)}\d+(?:{re.escape(record.RAN_MARK)}([0-9.]+)s)?$")


class Op(NamedTuple):
    command: str
    ran_s: float | None      # how long it took the agent, when the marker says


def blocks(script: str) -> list[Op]:
    """The operations of a command file: the text between consecutive
    ``# openrua op N`` lines (what commands.sh extraction writes), each
    with the duration its marker carries. A file without markers is one
    operation; text that is only comments and blank lines (the file's
    header) is none."""
    out: list[Op] = []
    current: list[str] = []
    ran: float | None = None

    def flush(next_ran: float | None) -> None:
        nonlocal ran
        if any(l.strip() and not l.lstrip().startswith("#") for l in current):
            out.append(Op("\n".join(current).strip("\n"), ran))
        current.clear()
        ran = next_ran

    for line in script.splitlines():
        m = _MARKER.match(line)
        if m:
            flush(float(m.group(1)) if m.group(1) else None)
        else:
            current.append(line)
    flush(None)
    return out


def op_timeout_s(ran_s: float | None, remaining_s: float) -> float:
    """The bound for one replayed operation: three times what it took
    the agent plus half a minute (a replay's simulator may be slower),
    never under a minute, never past the trial's wall clock; without a
    recorded duration, the wall clock alone."""
    if not ran_s:
        return remaining_s
    return min(remaining_s, max(60.0, 3.0 * ran_s + 30.0))


# The persistent shell's prelude: a state directory and the runner of one
# operation (bash -c is quoted once, as a whole).
_PRELUDE = r"""
_openrua_state=$(mktemp -d)
_openrua_run() {
  timeout --foreground -k 5 "$1" bash -c '
    shopt -s expand_aliases; alias exit="return"
    [ -f "$1/env" ] && source "$1/env"
    [ -f "$1/cwd" ] && cd "$(cat "$1/cwd")" 2>/dev/null
    source "$1/op"; _op </dev/null; _rc=$?
    export -p > "$1/env"; pwd > "$1/cwd"
    builtin exit $_rc' _ "$_openrua_state"
  _rc=$?
  [ "$_rc" -eq 124 ] && echo "[openrua] operation killed after $1s (bounded as the recorded run was; see commands.sh)"
  return $_rc
}
"""


class Shell:
    """One bash inside the sandbox, kept open across operations the way
    the agent's own shell is: the working directory and exported
    variables carry over. Each operation runs under its own time bound
    (the agent's tool cut commands that never returned; see
    ``op_timeout_s``), which means in a child bash: the persistent
    shell hands it the directory and the environment and takes them
    back afterwards. Each operation is followed by a sentinel echo; the
    output is everything up to that sentinel."""

    def __init__(self, sandbox: str):
        self._proc = subprocess.Popen(
            ["docker", "exec", "-i", "-w", "/workspace", sandbox, "bash"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1)
        self._n = 0
        # The operation runs in a child bash under `timeout`, from a file
        # the persistent shell writes. In the child, `exit` is aliased to
        # `return`: the agent's tool ran each command in a shell of its
        # own, where `exit` ended only that command; here it ends the
        # operation's function and the child still hands the directory
        # and the exported variables back (aliases expand at parse time;
        # a quoted `exit`, in a python -c or bash -c string, is untouched).
        # An empty stdin, so a command that reads input cannot swallow
        # the operations queued behind it.
        self._proc.stdin.write(_PRELUDE)
        self._proc.stdin.flush()

    def run(self, command: str, timeout_s: float,
            op_timeout_s: float | None = None) -> tuple[str, bool]:
        """Run one operation; returns (output, finished). Not finished
        means the wall clock (``timeout_s``) ran out while it was still
        running, and the shell is unusable afterwards. ``op_timeout_s``
        bounds the operation itself: past it the operation is killed, a
        note is appended to its output, and the shell goes on to the
        next one."""
        self._n += 1
        sentinel = f"__openrua_op_done_{self._n}__"
        marker = f"__openrua_op_text_{self._n}__"
        bound = int(op_timeout_s if op_timeout_s else timeout_s) or 1
        self._proc.stdin.write(f"cat > \"$_openrua_state/op\" <<'{marker}'\n"
                               f"_op() {{\n{command}\n}}\n{marker}\n"
                               f"_openrua_run {bound}\necho {sentinel}\n")
        self._proc.stdin.flush()
        lines: list[str] = []
        deadline = time.monotonic() + timeout_s
        reader = threading.Thread(target=self._read_until, args=(sentinel, lines),
                                  daemon=True)
        reader.start()
        reader.join(max(0.0, deadline - time.monotonic()))
        if reader.is_alive():
            self.close()
            return "".join(lines), False
        return "".join(lines), True

    def _read_until(self, sentinel: str, lines: list[str]) -> None:
        for line in self._proc.stdout:
            if line.strip() == sentinel:
                return
            lines.append(line)

    def close(self) -> None:
        for stream in (self._proc.stdin, self._proc.stdout):
            try:
                stream.close()
            except OSError:
                pass
        try:
            self._proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._proc.kill()


def script_operator(ctx: dict) -> dict:
    """Replay a command file in the sandbox, one operation at a time.

    Surface-only by construction: the operations run in a bash inside
    the sandbox as the sandbox user with ROS sourced, the same shell the
    agent gets; no control-line verb is touched. Every operation's
    command, output head and wall times land in the trial's
    ``ops.jsonl`` (what a demo is rendered from). The wall-clock cap is
    enforced here like the agent's; the verdict comes from the ordinary
    blind ask afterwards."""
    path = ctx.get("script")
    if not path:
        raise ValueError("--operator script requires --script <file>")
    script = Path(path).read_text()
    # The trial's own copy of what ran, as an agent trial has one
    # extracted from its transcript.
    (ctx["trial_dir"] / "commands.sh").write_text(script)
    ops = blocks(script)
    meta: dict = {"operator": "script", "script": str(path), "ops": len(ops)}
    deadline = time.monotonic() + ctx["active_wall_clock_min"] * 60
    shell = Shell(ctx["sandbox"])
    timed: list[dict] = []
    finished = True
    try:
        for i, op in enumerate(ops):
            t0 = time.time()
            remaining = deadline - time.monotonic()
            output, finished = shell.run(op.command, remaining,
                                         op_timeout_s=op_timeout_s(op.ran_s, remaining))
            timed.append({"kind": "shell", "command": op.command, "output": output,
                          "t0": t0, "t1": time.time()})
            if not finished:
                break
    finally:
        shell.close()
        record.write_ops(ctx["trial_dir"], timed)
    meta["termination"] = "self_finished" if finished else "wall_clock_cap"
    meta["output_tail"] = timed[-1]["output"][-2000:] if timed else ""
    return meta


OPERATORS = {
    "none": none_operator,
    "script": script_operator,
    "agent": agent_operator,
}
