"""Tear down the simulated robot's body. HOST-SIDE. The power switch.

The graceful path is CONVERSATION, not this module: the caller sends
the shutdown verb on the stdio line (or simply exits; pipe EOF makes
the robot power itself off). This is the e-stop: used only when the
conversation is already dead -- asking a wedged process to stop is
impossible by definition, so the cut must live outside the robot's own
software, exactly like a physical e-stop circuit.
"""

from __future__ import annotations

import subprocess


def down(name: str) -> None:
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)


def main() -> int:  # standalone: e-stop one body by name
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--name", required=True, help="body container name")
    args = ap.parse_args()
    down(args.name)
    print(f"e-stopped {args.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
