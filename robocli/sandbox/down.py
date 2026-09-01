"""Verb ``down``: container name -> removed.

    python3 -m robocli.sandbox.down --name <container>

The workspace dir survives (it is the caller's file output, not ours).
"""

from __future__ import annotations

import argparse
import subprocess
import sys


def down(name: str) -> bool:
    """Remove the container; True if it existed."""
    r = subprocess.run(["docker", "rm", "-f", name],
                       capture_output=True, text=True)
    return r.returncode == 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--name", required=True)
    args = ap.parse_args()
    return 0 if down(args.name) else 1


if __name__ == "__main__":
    sys.exit(main())
