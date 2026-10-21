"""Verb ``up``: whitelist -> idempotent ensure -> proxy URL.

    python3 -m openrua.proxy.up --network <internal-net> \
        [--whitelist "<one regex per line>"] [--image openrua-proxy]

One standing instance per whitelist (contrast sandbox.up, which always
creates a fresh container): its name carries the whitelist's hash, so
agents with different hosts get different proxies and none is ever
replaced under a running session. If the proxy is running, running is
the answer. On a cold start several callers may race; losers of the name
conflict adopt the winner's instance; a live proxy is never ``rm -f``-ed
(that would cut every in-flight session through it).

Value output: the proxy URL (``http://<name>:<port>``; the container
name doubles as its DNS hostname on the shared network; the port is read
back from the label the image stamped at build, never a copied constant).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time

from openrua.proxy import ProxyError, IMAGE, name_for


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
    """The proxy's listen port, self-described by its build-time label."""
    out = subprocess.run(
        ["docker", "inspect", "--format",
         '{{index .Config.Labels "openrua.proxy.port"}}', name],
        capture_output=True, text=True).stdout.strip()
    if not out or out == "<no value>":
        raise ProxyError(
            f"container {name!r} carries no openrua.proxy.port label; "
            "rebuild the proxy image (openrua build proxy)")
    return out


def ensure(network: str, whitelist: str = "", image: str = IMAGE) -> str:
    """Ensure a proxy enforcing ``whitelist`` serves ``network``; returns
    its URL.

    - its name is ``name_for(whitelist)``: the same hosts always meet the
      same proxy, other hosts get their own;
    - a stopped proxy (host reboot, crash) is revived with ``docker
      start``, never recreated; new ones carry ``--restart
      unless-stopped`` so reboots self-heal;
    - liveness is verified before the URL is returned; a missing image
      or dead daemon is an instructive error, never a silent URL to
      nothing (tinyproxy runs as PID 1, so running means the daemon is
      alive);
    - network membership is re-ensured on every call (idempotent
      connect), not only at creation;
    - an image drift (proxy rebuilt under the tag while the old
      container stands) is reported to stderr but never auto-fixed:
      killing a live proxy cuts every in-flight session, so that call
      stays with a person.
    """
    name = name_for(whitelist)
    if not _running(name):
        if _exists(name):
            started = subprocess.run(["docker", "start", name], capture_output=True, text=True)
        else:
            started = subprocess.run(
                ["docker", "run", "-d", "--restart", "unless-stopped",
                 "--name", name, "-e", f"WHITELIST={whitelist}", image],
                capture_output=True, text=True)
        for _ in range(10):  # wait out a sibling's in-progress creation
            if _running(name):
                break
            time.sleep(1)
        if not _running(name):
            raise ProxyError(
                f"proxy {name!r} failed to come up; is the image built? "
                f"(openrua build proxy) docker logs {name} for the reason\n"
                f"{started.stderr.strip()}")
    attached = subprocess.run(["docker", "network", "connect", network, name],
                              capture_output=True, text=True)
    # A duplicate connect is harmless only if the requested membership exists.
    # Other errors must not produce a URL to an unreachable proxy.
    try:
        networks = json.loads(_inspect("container", name, '{{json .NetworkSettings.Networks}}'))
    except (ValueError, TypeError):
        networks = None
    if not isinstance(networks, dict) or network not in networks:
        raise ProxyError(
            f"proxy {name!r} is not connected to network {network!r}\n"
            f"{attached.stderr.strip()}\n"
            f"check docker network inspect {network} and docker inspect {name}; "
            "retry after restoring network access. Existing sessions were not stopped.")
    running_image = _inspect("container", name, "{{.Image}}")
    tagged_image = _inspect("image", image, "{{.Id}}")
    if tagged_image and running_image != tagged_image:
        print(f"[proxy] WARNING: standing proxy {name!r} runs an older "
              f"image than tag {image!r}; recycle it between campaigns: "
              f"docker rm -f {name}", file=sys.stderr)
    return f"http://{name}:{_port(name)}"


def url_from_network(name: str, network: str = "bridge") -> str:
    """The proxy's URL by IP address on ``network``, for a client that
    cannot resolve docker container names (a sandbox on the host
    network)."""
    ip = _inspect("container", name,
                  f'{{{{(index .NetworkSettings.Networks "{network}").IPAddress}}}}')
    if not ip:
        raise ProxyError(f"proxy {name!r} has no address on docker network {network!r}")
    return f"http://{ip}:{_port(name)}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--network", required=True,
                    help="internal docker network the proxy serves")
    ap.add_argument("--whitelist", default="",
                    help="hosts it lets through, one regex per line (empty = none)")
    ap.add_argument("--image", default=IMAGE)
    args = ap.parse_args()
    try:
        print(ensure(network=args.network, whitelist=args.whitelist, image=args.image))
    except ProxyError as e:
        raise SystemExit(str(e))
    return 0


if __name__ == "__main__":
    sys.exit(main())
