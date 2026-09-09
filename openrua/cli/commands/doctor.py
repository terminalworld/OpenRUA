"""``openrua doctor [robot] [--sim S] [--bench B]``: is this machine ready
to bring a robot up?"""

from __future__ import annotations

from openrua import doctor
from openrua.cli.output import add_json


def run(args) -> int:
    report = doctor.run(robot=args.robot, agent_names=args.agent, home=args.home,
                        sim=args.sim, bench=args.bench)
    return doctor.print_report(report, as_json=True if args.json else None)


def add_parser(sub) -> None:
    p = sub.add_parser("doctor", help="check the install: docker, images, simulator, login")
    p.add_argument("robot", nargs="?", default=None,
                   help="also check this robot's images and simulator (with --sim, or "
                   "--bench, as for openrua run)")
    p.add_argument("--sim", default=None, help="simulator that embodies the robot")
    p.add_argument("--bench", default=None, help="benchmark to check the install of")
    p.add_argument("--agent", action="append", default=None, metavar="NAME[@VERSION]",
                   help="agent(s) the images must carry, @VERSION as pinned at build "
                   "(default: the robot's config, else the configured default)")
    add_json(p)
    p.set_defaults(fn=run)
