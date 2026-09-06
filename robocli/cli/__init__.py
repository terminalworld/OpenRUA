"""The command line: ``robocli <verb> ...``.

    robocli robots | benchmarks | agents   what is available (bundled + yours)
    robocli build <robot|sandbox|proxy>    build one of the three images
    robocli up panda-sim                   a live robot (real or simulated) with
                                           a sandbox terminal on its ROS 2 graph
    robocli agent                          open a coding agent on that terminal
    robocli down                           power everything off
    robocli run --config libero_pro ...    run a task set
    robocli probe --host                   draft a profile from a live graph
    robocli config schema                  every config key and its meaning
    robocli doctor [robot]                 check docker, images, simulator, login

Each verb is a module under ``commands/`` with ``add_parser`` and
``run``; this file builds the parser, reads the environment once
(``ROBOCLI_HOME`` seeds ``--home``) and turns a ``RoboCLIError`` into a
message and an exit status.
"""

from __future__ import annotations

import argparse
import os
import sys

from robocli import __version__
from robocli.cli.commands import COMMANDS
from robocli.config import paths
from robocli.errors import RoboCLIError


def build_parser(default_home: str | None = None) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="robocli", description=__doc__.split("\n\n")[0])
    ap.add_argument("--version", action="version", version=f"robocli {__version__}")
    ap.add_argument("--home", default=default_home, type=paths.home,
                    help="the user directory: your robots/, benchmarks/, agents/, "
                    "credentials/, simulators/ (default: $ROBOCLI_HOME or ~/.robocli)")
    sub = ap.add_subparsers(dest="verb", metavar="<verb>")
    for command in COMMANDS:
        command.add_parser(sub)
    return ap


def main(argv: list[str] | None = None) -> int:
    # The one place the environment is read: ROBOCLI_HOME seeds --home's
    # default; from here on the user directory travels as a parameter.
    ap = build_parser(default_home=os.environ.get("ROBOCLI_HOME") or None)
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)
    if not args.verb:
        ap.print_help()
        return 0
    if args.home is None:
        args.home = paths.home()
    try:
        return args.fn(args) or 0
    except RoboCLIError as e:
        # What is wrong, how to fix it, and a sysexits code scripts can branch on.
        print(f"error: {e.message}", file=sys.stderr)
        if e.hint:
            print(f"hint: {e.hint}", file=sys.stderr)
        return e.exit_code


if __name__ == "__main__":
    sys.exit(main())
