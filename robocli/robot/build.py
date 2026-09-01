"""Verb ``build``: blueprint -> the simulated robot's body image. HOST-SIDE.

Explicit and low-frequency (blueprint / distro changes only); ``up``
never builds. Docker's layer cache makes an unchanged re-build a cheap
no-op returning the same digest.

    python3 -m robocli.robot.build [--distro jazzy] [--tag ...]

Value output: ``<tag> <digest>`` (one line). The image self-describes
(labels: blueprint sha256 + build time), so a running body can always
be traced back to the exact blueprint that made it.
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
    """Build the body image from sim-<distro>.Dockerfile; (tag, digest)."""
    dockerfile = _HERE / f"sim-{distro}.Dockerfile"
    if not dockerfile.exists():
        have = sorted(p.name for p in _HERE.glob("sim-*.Dockerfile"))
        raise RuntimeError(f"no blueprint for distro '{distro}' (have {have})")
    tag = tag or f"robocli-sim-{distro}"
    cmd = [
        "docker", "build", "-f", str(dockerfile), "-t", tag,
        "--label",
        f"robocli.blueprint_sha256="
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
                    help="which blueprint: sim-<distro>.Dockerfile")
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
