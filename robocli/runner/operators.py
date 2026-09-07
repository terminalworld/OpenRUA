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
import subprocess
import threading
import time
from pathlib import Path

from robocli.runner import record
from robocli.runner.session import agent_operator


def none_operator(ctx: dict) -> dict:
    return {"operator": "none"}


_MARKER = re.compile(rf"^{re.escape(record.OP_MARKER)}\d+$")


def blocks(script: str) -> list[str]:
    """The operations of a command file: the text between consecutive
    ``# robocli op N`` lines (what commands.sh extraction writes). A file
    without markers is one operation; text that is only comments and
    blank lines (the file's header) is none."""
    out: list[str] = []
    current: list[str] = []

    def flush() -> None:
        if any(l.strip() and not l.lstrip().startswith("#") for l in current):
            out.append("\n".join(current).strip("\n"))
        current.clear()

    for line in script.splitlines():
        if _MARKER.match(line):
            flush()
        else:
            current.append(line)
    flush()
    return out


class Shell:
    """One bash inside the sandbox, kept open across operations the way
    the agent's own shell is (cwd and variables carry over). Each
    operation is written to it followed by a sentinel echo; the output
    is everything up to that sentinel."""

    def __init__(self, sandbox: str):
        self._proc = subprocess.Popen(
            ["docker", "exec", "-i", "-w", "/workspace", sandbox, "bash"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1)
        self._n = 0

    def run(self, command: str, timeout_s: float) -> tuple[str, bool]:
        """Run one operation; returns (output, finished). Not finished
        means the wall clock ran out while it was still running, and
        the shell is unusable afterwards."""
        self._n += 1
        sentinel = f"__robocli_op_done_{self._n}__"
        # Braces keep the operation in this shell (cd and variables
        # persist); the redirection gives it an empty stdin, so a command
        # that reads input cannot swallow the operations queued behind it.
        self._proc.stdin.write(f"{{\n{command}\n}} </dev/null\necho {sentinel}\n")
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
        for i, command in enumerate(ops):
            t0 = time.time()
            output, finished = shell.run(command, deadline - time.monotonic())
            timed.append({"kind": "shell", "command": command, "output": output,
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
