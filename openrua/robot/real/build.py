"""Verb ``build``: a directory holding a Dockerfile -> a driver image.

    python3 -m openrua.robot.real.build --image <name> --context <dir>

A real robot's config names the images its launch command uses and the
directories they are built from (``backend.build``, as docker compose's
``build:``); this builds one. Value output: ``<image> <digest>``. The
build's own output streams through; a failed build is an error naming
its exit status, never a stale image reported as built.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


class DriverBuildError(RuntimeError):
    """A driver image did not build (libraries raise; only main exits)."""


def build(image: str, context: str) -> tuple[str, str]:
    """``docker build -t image context``; returns (image, digest)."""
    if not (Path(context) / "Dockerfile").is_file():
        raise DriverBuildError(f"no Dockerfile in {context} (backend.build names it for {image})")
    r = subprocess.run(["docker", "build", "-t", image, context])
    if r.returncode != 0:
        raise DriverBuildError(f"docker build {image} failed (exit {r.returncode}); "
                               "see the build output above")
    digest = subprocess.run(["docker", "image", "inspect", "--format", "{{.Id}}", image],
                            capture_output=True, text=True, check=True).stdout.strip()
    return image, digest


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--image", required=True, help="image name to tag")
    ap.add_argument("--context", required=True, help="directory holding its Dockerfile")
    args = ap.parse_args()
    try:
        image, digest = build(args.image, args.context)
    except DriverBuildError as e:
        raise SystemExit(str(e))
    print(f"{image} {digest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
