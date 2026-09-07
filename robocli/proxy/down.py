"""Verb ``down``: name -> removed.

    python3 -m robocli.proxy.down [--name robocli-proxy]

Teardown and testing only; campaigns leave the proxy standing (killing
a live proxy cuts every in-flight session through it).
"""

from __future__ import annotations

import argparse
import subprocess
import sys

from robocli.proxy import NAME


def down(name: str = NAME) -> bool:
    """Remove the proxy container; True if it existed."""
    return subprocess.run(["docker", "rm", "-f", name],
                          capture_output=True).returncode == 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--name", default=NAME)
    args = ap.parse_args()
    return 0 if down(args.name) else 1


if __name__ == "__main__":
    sys.exit(main())
