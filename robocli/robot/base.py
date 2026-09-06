"""The backend contract: how a robot is provided to a trial.

A backend brings a robot up and hands back a ``Handle``; the runner
never learns which kind it got. ``robot.up()`` dispatches on the
resolved config's ``machine.backend.kind``.
"""

from __future__ import annotations


class Handle:
    """What ``up()`` returns: the runner's only relationship with a live robot."""

    name: str

    def wait_ready(self, timeout_s: float = 1800.0) -> None:
        """Block until the robot answers, or raise TimeoutError."""
        raise NotImplementedError

    def rpc(self, obj: dict, timeout_note: str = "", timeout_s: float = 900.0) -> dict:
        """One truth-side question (reset, success, task_info, steps ...),
        one answer. A backend with no truth side answers
        ``{"ok": True, "not_applicable": True}``."""
        raise NotImplementedError

    def shutdown(self) -> None:
        """Stop the robot; never raises."""
        raise NotImplementedError
