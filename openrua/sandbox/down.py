"""Verb ``down``: container name -> removed.

    python3 -m openrua.sandbox.down --name <container>

The workspace dir survives (it is the caller's file output, not ours).
"""

from __future__ import annotations

import argparse
import subprocess
import sys


def stop_processes(name: str, grace_s: float = 5.0) -> None:
    """End everything the sandbox user runs (the agent and whatever it
    started) and keep the container: TERM first, so a tool that holds a
    goal can cancel it, then KILL what is left after ``grace_s``. The
    container's first process is not the user's to signal, so it stays."""
    for sig, wait in (("TERM", grace_s), ("KILL", 0)):
        subprocess.run(["docker", "exec", "-u", "robot", name, "bash", "-c",
                        f"kill -{sig} -1 2>/dev/null; sleep {wait:g}"],
                       capture_output=True, timeout=wait + 30)


def down(name: str) -> bool:
    """Remove the container; True if it existed."""
    r = subprocess.run(["docker", "rm", "-f", name],
                       capture_output=True, text=True)
    if r.returncode and "No such container" not in r.stderr:
        raise RuntimeError(f"could not remove {name}: {r.stderr.strip()}; check docker info")
    return r.returncode == 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--name", required=True)
    args = ap.parse_args()
    return 0 if down(args.name) else 1


if __name__ == "__main__":
    sys.exit(main())
