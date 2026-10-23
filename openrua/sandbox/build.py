"""Verb ``build``: Dockerfile + parameters -> the sandbox image.

Explicit and low-frequency (Dockerfile / preinstall / distro changes only);
``up`` never builds. Docker's layer cache makes an unchanged re-build a
cheap no-op returning the same digest.

    python3 -m openrua.sandbox.build \
        [--preinstall "<one-line install chain>"] \
        [--distro jazzy] [--robot-uid N] [--tag openrua-sandbox-<distro>]

Value output: ``<tag> <digest>`` (one line). The proxy image is the
proxy package's own build.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
from uuid import uuid4

from openrua.sandbox import SandboxError

_HERE = Path(__file__).resolve().parent


def _docker_build(dockerfile: Path, context: Path, tag: str,
                  build_args: dict[str, str],
                  labels: dict[str, str] | None = None) -> str:
    cmd = ["docker", "build", "-f", str(dockerfile), "-t", tag]
    for k, v in build_args.items():
        cmd += ["--build-arg", f"{k}={v}"]
    for k, v in (labels or {}).items():
        cmd += ["--label", f"{k}={v}"]
    cmd.append(str(context))
    r = subprocess.run(cmd)
    if r.returncode != 0:
        raise SandboxError(f"docker build {tag} failed (exit {r.returncode}); see build output above")
    return subprocess.run(
        ["docker", "inspect", "--format", "{{.Id}}", tag],
        capture_output=True, text=True, check=True).stdout.strip()


RECIPE_LABEL = "openrua.recipe_sha256"


def recipe_sha256() -> str:
    """The sandbox recipe as it reads today: the image carries the one it
    was built from, so doctor can tell an image built from an older one."""
    return hashlib.sha256((_HERE / "sandbox.Dockerfile").read_bytes()).hexdigest()


def build(preinstall: str = "", distro: str = "jazzy",
          robot_uid: int | None = None,
          tag: str | None = None,
          labels: dict[str, str] | None = None,
          stamp: str = "", base_image: str | None = None) -> tuple[str, str]:
    """Build the sandbox image for a ROS 2 distro; returns (tag, digest).
    The tag defaults to openrua-sandbox-<distro>, the name a profile's
    ros_distro resolves to. ``labels`` are
    stamped on the image alongside the preinstall hash (the caller adds
    one per agent, so doctor can check each selected agent). A new
    ``stamp`` rebuilds the install layer instead of reusing a cached one:
    the caller passes one when the chain installs the current release.
    ``base_image`` retains an existing image's tools and runtime settings;
    ``robot_uid`` applies only when building a fresh sandbox."""
    # Contract: PREINSTALL is a single-line command chain without double
    # quotes (the one quoting hazard of string-valued build args).
    if '"' in preinstall or "\n" in preinstall.strip():
        raise SandboxError(
            "--preinstall must be a single-line command chain without "
            "double quotes")
    tag = tag or f"openrua-sandbox-{distro}"
    if base_image is not None:
        if robot_uid is not None:
            raise SandboxError('--base-image preserves existing users; --robot-uid applies to fresh sandbox images')
        return _extend(base_image, preinstall, tag, labels or {}, stamp)
    uid = os.getuid() if robot_uid is None else robot_uid
    # The image says what was baked into it: doctor compares this label
    # with the install chain the selected agents would emit today.
    labels = {"openrua.preinstall_sha256": hashlib.sha256(preinstall.encode()).hexdigest(),
              RECIPE_LABEL: recipe_sha256(), **(labels or {})}
    return tag, _docker_build(
        _HERE / "sandbox.Dockerfile", _HERE, tag,
        {"ROS_DISTRO": distro, "ROBOT_UID": str(uid),
         "PREINSTALL": preinstall, "PREINSTALL_STAMP": stamp}, labels)


def _extend(base_image: str, preinstall: str, tag: str, labels: dict, stamp: str) -> tuple[str, str]:
    result = subprocess.run(['docker', 'image', 'inspect', base_image, '--format', '{{json .}}'],
                            capture_output=True, text=True)
    if result.returncode:
        raise SandboxError(f'Cannot inspect base image {base_image}: {result.stderr.strip()}')
    facts = json.loads(result.stdout)
    dockerfile = _HERE / 'extend.Dockerfile'
    recipe = hashlib.sha256(dockerfile.read_bytes()).hexdigest()
    base_tag = 'openrua-build-base-' + uuid4().hex
    tagged = subprocess.run(['docker', 'tag', facts['Id'], base_tag], capture_output=True, text=True)
    if tagged.returncode:
        raise SandboxError(f'Cannot retain base image {base_image}: {tagged.stderr.strip()}')
    labels = {'openrua.base_image_sha256': facts['Id'],
              'openrua.extension_recipe_sha256': recipe,
              'openrua.preinstall_sha256': hashlib.sha256(preinstall.encode()).hexdigest(), **labels}
    try:
        return tag, _docker_build(dockerfile, _HERE, tag,
                                 {'BASE_IMAGE': base_tag, 'BASE_USER': facts['Config'].get('User') or 'root',
                                  'PREINSTALL': preinstall, 'PREINSTALL_STAMP': stamp}, labels)
    finally:
        subprocess.run(['docker', 'rmi', base_tag], capture_output=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--preinstall", default="",
                    help="one-line install command chain baked into the "
                    "image (empty = the bare terminal)")
    ap.add_argument("--distro", default="jazzy", help="ROS 2 distro: jazzy | humble")
    ap.add_argument("--robot-uid", type=int, default=None,
                    help="container uid (default: current user)")
    ap.add_argument("--tag", default=None, help="image tag (default: openrua-sandbox-<distro>)")
    ap.add_argument('--base-image', default=None, help='add the install layer to an existing image, retaining its tools and runtime settings')
    args = ap.parse_args()
    try:
        tag, digest = build(preinstall=args.preinstall,
                            distro=args.distro,
                            robot_uid=args.robot_uid, tag=args.tag, base_image=args.base_image)
    except SandboxError as e:
        raise SystemExit(str(e))
    print(f"{tag} {digest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
