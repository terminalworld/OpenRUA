"""``robocli run``: a task set on a robot (the runner's arguments, on this parser)."""

from __future__ import annotations

from robocli.runner import main as runner


def add_parser(sub) -> None:
    p = sub.add_parser("run", help="run a task set: one trial per task and seed",
                       description=runner.__doc__.split("\n\n")[0])
    runner.add_arguments(p, include_home=False)
    p.set_defaults(fn=runner.run)
