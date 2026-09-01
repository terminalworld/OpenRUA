"""Verb ``down``: name -> removed.

    python3 -m robocli.proxy.down [--name robocli-proxy]

Teardown/testing only; campaigns leave the wall standing (killing a
live wall cuts every in-flight session through it).
"""

from __future__ import annotations

import argparse
import subprocess
import sys


def down(name: str = "robocli-proxy") -> bool:
    """Remove the wall container; True if it existed."""
    return subprocess.run(["docker", "rm", "-f", name],
                          capture_output=True).returncode == 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--name", default="robocli-proxy")
    args = ap.parse_args()
    return 0 if down(args.name) else 1


if __name__ == "__main__":
    sys.exit(main())
