"""The backend contract: how a robot is provided to a trial.

A backend brings a robot up and hands back a ``Handle``; the runner
never learns which kind it got. ``robot.up()`` dispatches on the
resolved config's ``machine.backend.kind``.
"""

from __future__ import annotations


class Handle:
    """What ``up()`` returns: the runner's only relationship with a live robot."""

    name: str

    def wait_ready(self, timeout_s: float = 1800.0, *, cancelled=None) -> None:
        """Wait for readiness; raise InterruptedError if cancelled() is true."""
        raise NotImplementedError

    def rpc(self, obj: dict, timeout_note: str = "", timeout_s: float = 900.0, *, cancelled=None) -> dict:
        """One truth-side question (reset, success, task_info, steps ...),
        one answer. A backend with no truth side answers
        ``{"ok": True, "not_applicable": True}``. The caller may cancel a
        pending startup request through the optional predicate."""
        raise NotImplementedError

    def shutdown(self) -> None:
        """Stop owned robot resources; report any unconfirmed shutdown."""
        raise NotImplementedError

    def stopped(self) -> str | None:
        """Why the robot's resources ended on their own, once they have
        (a driver that exited, a simulator that died); None while they
        run, or when the backend owns nothing it could watch."""
        return None
