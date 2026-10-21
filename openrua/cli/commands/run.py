"""``openrua run``: removed; it says where its uses went.

It opened the coding agent's own terminal on a robot, from before
OpenRUA had a terminal of its own. Talking to the agent is ``openrua``;
its own terminal is ``openrua up`` and then ``openrua agent``; the agent
doing a task on its own, recorded, is ``openrua exec``. This stub goes
in the next release.
"""

from __future__ import annotations

import argparse

from openrua.errors import UsageError


def run(args) -> int:
    raise UsageError("openrua run was removed",
                     hint="to talk to the agent: openrua; for the agent's own terminal: "
                     "openrua up <robot>, then openrua agent in a second terminal; for the "
                     "agent to do a task on its own, recorded: openrua exec")


def add_parser(sub) -> None:
    p = sub.add_parser("run", help="removed: use openrua, openrua exec, or up + agent",
                       description=__doc__)
    p.add_argument("rest", nargs=argparse.REMAINDER, help=argparse.SUPPRESS)
    p.set_defaults(fn=run)
