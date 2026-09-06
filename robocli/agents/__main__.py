"""``python -m robocli.agents launch ...``: run an agent headless in a live sandbox.

The one verb this package exposes from the command line; image builds
take their facts from the manifests through ``robocli build``.
"""

from __future__ import annotations

import sys


def main() -> int:
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help"):
        print("usage: python -m robocli.agents launch [options]\n"
              "options: python -m robocli.agents launch --help")
        return 0
    if args[0] != "launch":
        print(f"unknown verb {args[0]!r}; the only verb is launch", file=sys.stderr)
        return 2
    sys.argv = ["python -m robocli.agents launch", *args[1:]]
    from robocli.agents import launcher
    return launcher.main()


if __name__ == "__main__":
    sys.exit(main())
