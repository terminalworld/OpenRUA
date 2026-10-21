"""What ``openrua exec`` shows while the agent works, as a coding agent's
own non-interactive mode does: what the agent says, and each command or
tool it runs, once done, with whether it worked and how long it took. On
a terminal, the commands still running show on one line that updates
every second. The whole record (outputs included) stays in the run
directory; this is a view.

Steps arrive from the agent's own transcript (``Agent.steps``), on the
runner's thread; a ``Progress`` is the callable that receives them.
"""

from __future__ import annotations

import shutil
import sys
import textwrap
import threading
import time

AGENT = "agent  "


def _first_line(text: str) -> str:
    lines = text.strip().splitlines() or [""]
    return lines[0] + (" …" if len(lines) > 1 else "")


def _clip(text: str, width: int) -> str:
    return text if len(text) <= width else text[:max(1, width - 1)] + "…"


def _seconds(s: float) -> str:
    return f"{s:.1f}s" if s < 60 else f"{int(s // 60)}m{int(s % 60):02d}s"


class Progress:
    """Prints steps to ``out``; ``live`` (default: ``out`` is a terminal)
    adds colour and the running line."""

    def __init__(self, out=None, live: bool | None = None, clock=time.monotonic):
        self.out = out or sys.stdout
        self.live = self.out.isatty() if live is None else live
        self.clock = clock
        self.running: dict[str, tuple[str, bool, float]] = {}
        self.lock = threading.Lock()
        self.transient = False
        self.closed = threading.Event()
        if self.live:
            threading.Thread(target=self._tick, daemon=True).start()

    def width(self) -> int:
        return shutil.get_terminal_size((100, 24)).columns

    def _paint(self, code: str, text: str) -> str:
        return f"\033[{code}m{text}\033[0m" if self.live else text

    def _write(self, text: str) -> None:
        self.out.write(text)
        self.out.flush()

    def _clear(self) -> None:
        if self.transient:
            self._write("\r\033[K")
            self.transient = False

    def _redraw(self) -> None:
        self._clear()
        if not (self.live and self.running):
            return
        label, _, t0 = list(self.running.values())[-1]
        more = f"{len(self.running)} running · " if len(self.running) > 1 else ""
        line = f"  … {more}{_first_line(label)}"
        tail = f"  {_seconds(self.clock() - t0)}"
        self._write(self._paint("2", _clip(line, self.width() - len(tail) - 1) + tail))
        self.transient = True

    def _tick(self) -> None:
        while not self.closed.wait(1.0):
            with self.lock:
                try:
                    self._redraw()
                except (OSError, ValueError):   # the terminal is gone
                    return

    def __call__(self, step: dict) -> None:
        with self.lock:
            self._clear()
            kind = step.get("kind")
            if kind == "begin":
                self._write(f"[exec] {step['agent']} is at work; Ctrl-C stops it and powers "
                            "the robot off\n")
            elif kind == "text":
                lines = [part for line in step["text"].strip().splitlines()
                         for part in (textwrap.wrap(line, max(20, self.width() - len(AGENT)))
                                      or [""])]
                body = ("\n" + " " * len(AGENT)).join(lines)
                self._write(self._paint("1", AGENT.rstrip()) + AGENT[len(AGENT.rstrip()):]
                            + body + "\n")
            elif kind == "start":
                previous = self.running.get(step["id"])
                self.running[step["id"]] = (step["label"], bool(step.get("shell")),
                                            previous[2] if previous else self.clock())
            elif kind == "end":
                self._finished(*self.running.pop(step["id"], ("", True, self.clock())),
                               self._paint("32", "✓") if step.get("ok") else self._paint("31", "✗"))
            elif kind == "done":
                # What the agent left running when it stopped (a budget, Ctrl-C).
                for label, shell, t0 in self.running.values():
                    self._finished(label, shell, t0, self._paint("33", "unfinished"))
                self.running.clear()
            self._redraw()

    def _finished(self, label: str, shell: bool, t0: float, mark: str) -> None:
        tail = f"  {mark} {_seconds(self.clock() - t0)}"
        head = ("$ " if shell else "· ") + _first_line(label)
        self._write(_clip(head, self.width() - 20) + tail + "\n")

    def close(self) -> None:
        self.closed.set()
        with self.lock:
            self._clear()
