"""Verb ``build``: Dockerfile + parameters -> the sandbox image.

Explicit and low-frequency (Dockerfile / preinstall / distro changes only);
``up`` never builds. Docker's layer cache makes an unchanged re-build a
cheap no-op returning the same digest.

    python3 -m robocli.sandbox.build \
        [--preinstall "<one-line install chain>"] \
        [--ros-distro jazzy] [--robot-uid N] [--tag robocli-sandbox]

Value output: ``<tag> <digest>`` (one line). The proxy image is the
proxy package's own build.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import subprocess
import sys
from pathlib import Path

from robocli.sandbox import SandboxError

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
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SandboxError(f"docker build {tag} failed:\n{r.stderr[-2000:]}")
    return subprocess.run(
        ["docker", "inspect", "--format", "{{.Id}}", tag],
        capture_output=True, text=True, check=True).stdout.strip()


def build(preinstall: str = "", ros_distro: str = "jazzy",
          robot_uid: int | None = None,
          tag: str = "robocli-sandbox") -> tuple[str, str]:
    """Build the sandbox image; returns (tag, digest)."""
    # Contract: PREINSTALL is a single-line command chain without double
    # quotes (the one quoting hazard of string-valued build args).
    if '"' in preinstall or "\n" in preinstall.strip():
        raise SandboxError(
            "--preinstall must be a single-line command chain without "
            "double quotes")
    uid = os.getuid() if robot_uid is None else robot_uid
    # The image says what was baked into it: doctor compares this label
    # with the install chain the selected agents would emit today.
    labels = {"robocli.preinstall_sha256": hashlib.sha256(preinstall.encode()).hexdigest()}
    return tag, _docker_build(
        _HERE / "sandbox.Dockerfile", _HERE, tag,
        {"ROS_DISTRO": ros_distro, "ROBOT_UID": str(uid),
         "PREINSTALL": preinstall}, labels)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--preinstall", default="",
                    help="one-line install command chain baked into the "
                    "image (empty = the bare terminal)")
    ap.add_argument("--ros-distro", default="jazzy")
    ap.add_argument("--robot-uid", type=int, default=None,
                    help="container uid (default: current user)")
    ap.add_argument("--tag", default="robocli-sandbox")
    args = ap.parse_args()
    try:
        tag, digest = build(preinstall=args.preinstall,
                            ros_distro=args.ros_distro,
                            robot_uid=args.robot_uid, tag=args.tag)
    except SandboxError as e:
        raise SystemExit(str(e))
    print(f"{tag} {digest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
