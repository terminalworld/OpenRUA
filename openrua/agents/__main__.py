"""``python -m openrua.agents launch ...``: run an agent headless in a live sandbox.

The one verb this package exposes from the command line; image builds
take their facts from the manifests through ``openrua build``.
"""

from __future__ import annotations

import sys

from openrua.agents import launcher


def main() -> int:
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help"):
        print("usage: python -m openrua.agents launch [options]\n"
              "options: python -m openrua.agents launch --help")
        return 0
    if args[0] != "launch":
        print(f"unknown verb {args[0]!r}; the only verb is launch", file=sys.stderr)
        return 2
    sys.argv = ["python -m openrua.agents launch", *args[1:]]
    return launcher.main()


if __name__ == "__main__":
    sys.exit(main())
