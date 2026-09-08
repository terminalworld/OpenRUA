"""``openrua bench``: a task set on a robot (the runner's arguments, on this parser)."""

from __future__ import annotations

from openrua.runner import main as runner


def add_parser(sub) -> None:
    p = sub.add_parser("bench", help="run a benchmark: one trial per task and seed",
                       description=runner.__doc__.split("\n\n")[0])
    runner.add_arguments(p, include_home=False)
    p.set_defaults(fn=runner.run)
