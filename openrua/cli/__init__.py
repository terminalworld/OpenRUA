"""The command line: ``openrua <verb> ...``.

    openrua robots | simulators | benchmarks | agents   what is available (bundled + yours)
    openrua build [robot|sandbox|proxy]    the three images (or one of them)
    openrua config set robot panda simulator robosuite   your defaults
    openrua run "pick up the red cube"     a robot up, the agent on it, off after
    openrua up                             a live robot (real or simulated) with
                                           a sandbox terminal on its ROS 2 graph
    openrua agent                          open a coding agent on that terminal
    openrua down                           power everything off
    openrua bench --config libero_pro ...  run a benchmark
    openrua demo <trial>                   a video of a recorded trial
    openrua probe --host                   draft a profile from a live graph
    openrua config schema                  every config key and its meaning
    openrua doctor [robot]                 check docker, images, simulator, login

Each verb is a module under ``commands/`` with ``add_parser`` and
``run``; this file builds the parser, reads the environment once
(``OPENRUA_HOME`` seeds ``--home``) and turns a ``OpenRUAError`` into a
message and an exit status.
"""

from __future__ import annotations

import argparse
import os
import sys

from openrua import __version__
from openrua.cli.commands import COMMANDS
from openrua.config import paths
from openrua.errors import OpenRUAError


def build_parser(default_home: str | None = None) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="openrua", description=__doc__.split("\n\n")[0])
    ap.add_argument("--version", action="version", version=f"openrua {__version__}")
    ap.add_argument("--home", default=default_home, type=paths.home,
                    help="the user directory: your robots/, benchmarks/, agents/, "
                    "credentials/, simulators/ (default: $OPENRUA_HOME or ~/.openrua)")
    sub = ap.add_subparsers(dest="verb", metavar="<verb>")
    for command in COMMANDS:
        command.add_parser(sub)
    return ap


def main(argv: list[str] | None = None) -> int:
    # The one place the environment is read: OPENRUA_HOME seeds --home's
    # default; from here on the user directory travels as a parameter.
    ap = build_parser(default_home=os.environ.get("OPENRUA_HOME") or None)
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)
    if not args.verb:
        ap.print_help()
        return 0
    if args.home is None:
        args.home = paths.home()
    try:
        return args.fn(args) or 0
    except OpenRUAError as e:
        # What is wrong, how to fix it, and a sysexits code scripts can branch on.
        print(f"error: {e.message}", file=sys.stderr)
        if e.hint:
            print(f"hint: {e.hint}", file=sys.stderr)
        return e.exit_code


if __name__ == "__main__":
    sys.exit(main())
