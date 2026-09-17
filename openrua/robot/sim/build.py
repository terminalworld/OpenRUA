"""Build the simulated robot's images: the ROS base per distro, and one
image per simulator install on top of it.

Explicit and low-frequency; ``up`` never builds. Docker's layer cache
makes an unchanged re-build a cheap no-op returning the same digest.

    python3 -m openrua.robot.sim.build base [--distro jazzy]

Value output: ``<tag> <digest>`` (one line). Every image self-describes
through labels: the base carries its Dockerfile's sha256, a simulator
image the fingerprint of the declaration it was rendered from
(``install.fingerprint``), the package version and, from a checkout,
the commit; a running container can be traced back to what made it.
"""

from __future__ import annotations

import argparse
import hashlib
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

from openrua import __version__
from openrua.robot.sim import install as installer

_HERE = Path(__file__).resolve().parent

LABEL_FINGERPRINT = "openrua.install_sha256"
LABEL_VERSION = "openrua.version"
LABEL_COMMIT = "openrua.commit"


def _docker_build(dockerfile: Path, context: Path, tag: str,
                  labels: dict[str, str]) -> str:
    cmd = ["docker", "build", "-f", str(dockerfile), "-t", tag]
    for k, v in labels.items():
        cmd += ["--label", f"{k}={v}"]
    cmd.append(str(context))
    # The build's own output is the user's: layers, downloads, errors.
    r = subprocess.run(cmd)
    if r.returncode != 0:
        raise RuntimeError(f"docker build {tag} failed (exit {r.returncode}); "
                           "the failing step is the last one printed above")
    return digest(tag)


def digest(tag: str) -> str:
    return subprocess.run(["docker", "inspect", "--format", "{{.Id}}", tag],
                          capture_output=True, text=True, check=True).stdout.strip()


def image_exists(tag: str) -> bool:
    r = subprocess.run(["docker", "image", "inspect", tag], capture_output=True)
    return r.returncode == 0


def build_base(distro: str = "jazzy", tag: str | None = None) -> tuple[str, str]:
    """Build the ROS base image from Dockerfile.<distro>; returns (tag, digest)."""
    dockerfile = _HERE / f"Dockerfile.{distro}"
    if not dockerfile.exists():
        have = sorted(p.name.split(".", 1)[1] for p in _HERE.glob("Dockerfile.*"))
        raise RuntimeError(f"no Dockerfile for distro '{distro}' (have {have})")
    tag = tag or installer.base_image(distro)
    labels = {
        "openrua.dockerfile_sha256": hashlib.sha256(dockerfile.read_bytes()).hexdigest(),
        "openrua.built_utc": datetime.now(timezone.utc).isoformat(),
    }
    return tag, _docker_build(dockerfile, _HERE, tag, labels)


def build_simulator(install: dict, name: str, code_root: Path,
                    tag: str) -> tuple[str, str]:
    """Render a resolved install declaration (``name`` names it in the
    Dockerfile) and build it as ``tag``. The base image for the declared
    distro is built first when missing."""
    distro = install.get("ros_distro", "jazzy")
    if not image_exists(installer.base_image(distro)):
        build_base(distro)
    dockerfile, files = installer.render(install, name, code_root)
    labels = {
        LABEL_FINGERPRINT: installer.fingerprint(dockerfile, files),
        LABEL_VERSION: __version__,
        LABEL_COMMIT: _commit(code_root),
        "openrua.built_utc": datetime.now(timezone.utc).isoformat(),
    }
    with tempfile.TemporaryDirectory(prefix="openrua-build-") as tmp:
        ctx = installer.context(dockerfile, files, Path(tmp))
        return tag, _docker_build(ctx / "Dockerfile", ctx, tag, labels)


def _commit(code_root: Path) -> str:
    """The checkout's HEAD when the package runs from one, else the
    release ("unavailable")."""
    try:
        r = subprocess.run(["git", "-C", str(code_root), "rev-parse", "HEAD"],
                           capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return "unavailable"
    return r.stdout.strip() if r.returncode == 0 else "unavailable"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("unit", choices=["base"], help="what to build (simulator images: openrua build)")
    ap.add_argument("--distro", default="jazzy", help="which Dockerfile: Dockerfile.<distro>")
    ap.add_argument("--tag", default=None, help="image name (default openrua-sim-base-<distro>)")
    args = ap.parse_args()
    try:
        tag, dig = build_base(distro=args.distro, tag=args.tag)
    except RuntimeError as e:
        raise SystemExit(str(e))
    print(f"{tag} {dig}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
