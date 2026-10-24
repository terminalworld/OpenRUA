"""``openrua doctor [robot] [--sim S] [--bench B]``: is this machine ready
to bring a robot up?"""

from __future__ import annotations

import argparse
from pathlib import Path

from openrua import doctor
from openrua.cli.output import add_json
from openrua.config import projects


def run(args) -> int:
    chosen = projects.options(projects.resolve(args.workspace or Path.cwd()),
                              dict(robot=args.robot, sim=args.sim, bench=args.bench), relative_to=Path.cwd())
    report = doctor.run(robot=chosen['robot'], agent_names=args.agent or ([chosen['agent']] if chosen['agent'] else None),
                        home=args.home, sim=chosen['sim'], bench=chosen['bench'])
    return doctor.print_report(report, as_json=True if args.json else None)


def add_parser(sub) -> None:
    p = sub.add_parser("doctor", help="check the install: docker, images, simulator, login")
    p.add_argument('-C', '--cd', '--workspace', dest='workspace', default=argparse.SUPPRESS,
                   help='project directory (default: the current directory)')
    p.add_argument("robot", nargs="?", default=None,
                   help="also check this robot's images and simulator (with --sim, or "
                   "--bench, as for openrua exec); default: your saved robot, simulator "
                   "and benchmark, what openrua starts")
    p.add_argument("--sim", default=None, help="simulator that embodies the robot")
    p.add_argument("--bench", default=None, help="benchmark to check the install of")
    p.add_argument("--agent", action="append", default=None, metavar="NAME[@VERSION]",
                   help="agent(s) the images must carry, @VERSION as pinned at build "
                   "(default: the robot's config, else the configured default)")
    add_json(p)
    p.set_defaults(fn=run)
