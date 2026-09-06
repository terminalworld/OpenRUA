"""Build the simulated robot's image from Dockerfile.<distro>.

Explicit and low-frequency (Dockerfile / distro changes only); ``up``
never builds. Docker's layer cache makes an unchanged re-build a cheap
no-op returning the same digest.

    python3 -m robocli.robot.sim.build [--distro jazzy] [--tag ...]

Value output: ``<tag> <digest>`` (one line). The image self-describes
(labels: Dockerfile sha256 and build time), so a running container can
be traced back to the exact Dockerfile that made it.
"""

from __future__ import annotations

import argparse
import hashlib
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

_HERE = Path(__file__).resolve().parent


def build(distro: str = "jazzy", tag: str | None = None) -> tuple[str, str]:
    """Build the image from Dockerfile.<distro>; returns (tag, digest)."""
    dockerfile = _HERE / f"Dockerfile.{distro}"
    if not dockerfile.exists():
        have = sorted(p.name for p in _HERE.glob("Dockerfile.*"))
        raise RuntimeError(f"no Dockerfile for distro '{distro}' (have {have})")
    tag = tag or f"robocli-sim-{distro}"
    cmd = [
        "docker", "build", "-f", str(dockerfile), "-t", tag,
        "--label",
        f"robocli.dockerfile_sha256="
        f"{hashlib.sha256(dockerfile.read_bytes()).hexdigest()}",
        "--label",
        f"robocli.built_utc={datetime.now(timezone.utc).isoformat()}",
        str(_HERE),
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"docker build {tag} failed:\n{r.stderr[-2000:]}")
    digest = subprocess.run(
        ["docker", "inspect", "--format", "{{.Id}}", tag],
        capture_output=True, text=True, check=True).stdout.strip()
    return tag, digest


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--distro", default="jazzy",
                    help="which Dockerfile: Dockerfile.<distro>")
    ap.add_argument("--tag", default=None,
                    help="image name (default robocli-sim-<distro>)")
    args = ap.parse_args()
    try:
        tag, digest = build(distro=args.distro, tag=args.tag)
    except RuntimeError as e:
        raise SystemExit(str(e))
    print(f"{tag} {digest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
