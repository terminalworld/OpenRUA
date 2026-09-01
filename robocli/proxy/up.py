"""Verb ``up``: IDEMPOTENT ensure -> proxy URL.

    python3 -m robocli.proxy.up --network <internal-net> \
        [--name robocli-proxy] [--image robocli-proxy]

Singleton-infra semantics (contrast sandbox.up, which always births a
fresh container): if the wall is running, running is the answer. On a
cold start N callers may race; losers of the name conflict simply
adopt the winner's instance; a live wall is NEVER ``rm -f``-ed (that
would cut every in-flight session through it).

Value output: the proxy URL (``http://<name>:<port>``; the container
name doubles as its DNS hostname on the shared network, so the URL is
stable across wall rebuilds; the port is read back from the label the
image stamped at build, never a copied constant).
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

from robocli.proxy import ProxyError


def _running(name: str) -> str:
    return subprocess.run(
        ["docker", "ps", "-q", "--filter", f"name=^{name}$"],
        capture_output=True, text=True).stdout.strip()


def _exists(name: str) -> str:
    return subprocess.run(
        ["docker", "ps", "-aq", "--filter", f"name=^{name}$"],
        capture_output=True, text=True).stdout.strip()


def _inspect(kind: str, target: str, fmt: str) -> str:
    # kind is "container" or "image": a container and an image may share
    # the name (ours do), and bare `docker inspect` resolves the
    # container first; ambiguity here produced a false drift warning.
    return subprocess.run(
        ["docker", kind, "inspect", "--format", fmt, target],
        capture_output=True, text=True).stdout.strip()


def _port(name: str) -> str:
    """The wall's listen port, self-described by its build-time label."""
    out = subprocess.run(
        ["docker", "inspect", "--format",
         '{{index .Config.Labels "robocli.proxy.port"}}', name],
        capture_output=True, text=True).stdout.strip()
    if not out or out == "<no value>":
        raise ProxyError(
            f"container {name!r} carries no robocli.proxy.port label; "
            "rebuild the wall image (python3 -m robocli.proxy.build)")
    return out


def ensure(network: str, name: str = "robocli-proxy",
           image: str = "robocli-proxy") -> str:
    """Ensure the wall serves ``network``; returns its URL.

    Reliability contract (reviewed 2026-08-15):
    - a STOPPED wall (host reboot, crash) is revived with ``docker
      start``, never recreated; new walls carry ``--restart
      unless-stopped`` so reboots self-heal;
    - aliveness is VERIFIED before the URL is returned; a missing image
      or dead daemon is an instructive error, never a silent URL to
      nothing (tinyproxy runs as PID 1, so running == daemon alive);
    - network membership is re-ensured on EVERY call (idempotent
      connect), not only at creation;
    - an image drift (wall rebuilt under the tag while the old container
      stands) is reported to stderr but never auto-fixed: killing a live
      wall cuts every in-flight session, so that call stays human.
    """
    if not _running(name):
        if _exists(name):
            subprocess.run(["docker", "start", name], capture_output=True)
        else:
            subprocess.run(
                ["docker", "run", "-d", "--restart", "unless-stopped",
                 "--name", name, image], capture_output=True)
        for _ in range(10):  # wait out a sibling's in-progress creation
            if _running(name):
                break
            time.sleep(1)
        if not _running(name):
            raise ProxyError(
                f"wall {name!r} failed to come up; is the image built? "
                f"(python3 -m robocli.proxy.build) docker logs {name} "
                "for the death cry")
    subprocess.run(["docker", "network", "connect", network, name],
                   capture_output=True)  # idempotent re-ensure
    running_image = _inspect("container", name, "{{.Image}}")
    tagged_image = _inspect("image", image, "{{.Id}}")
    if tagged_image and running_image != tagged_image:
        print(f"[proxy] WARNING: standing wall {name!r} runs an older "
              f"image than tag {image!r} (whitelist may have changed); "
              f"recycle it between campaigns: docker rm -f {name}",
              file=sys.stderr)
    return f"http://{name}:{_port(name)}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--network", required=True,
                    help="internal docker network the wall serves")
    ap.add_argument("--name", default="robocli-proxy")
    ap.add_argument("--image", default="robocli-proxy")
    args = ap.parse_args()
    try:
        print(ensure(network=args.network, name=args.name, image=args.image))
    except ProxyError as e:
        raise SystemExit(str(e))
    return 0


if __name__ == "__main__":
    sys.exit(main())
