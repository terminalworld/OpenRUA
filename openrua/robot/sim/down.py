"""Remove the simulated robot's container from outside. Host side.

The ordinary way to stop the robot is the control line: the caller sends
``shutdown`` (or exits; pipe EOF makes the bridge stop itself). This is
for when that line is dead: a wedged process cannot be asked to stop,
so the cut lives outside the robot's own software.
"""

from __future__ import annotations

import argparse
import subprocess


def down(name: str) -> None:
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)


def main() -> int:  # standalone: remove one robot container by name

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--name", required=True, help="robot container name")
    args = ap.parse_args()
    down(args.name)
    print(f"removed {args.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
