"""Verb ``up``: a reachable machine + its config -> ONE live sandbox.

    python3 -m robocli.sandbox.up --config <assembly.yaml> \
        --workspace <dir> [--task-suite S] [--image robocli-sandbox] \
        [--network host|<docker-net>] [--static-peer <hostname>] \
        [--ros-domain N] [--internet none|proxy:<url>|open] \
        [--name <container>] [--mount SRC:DST ...] [--env K=V ...]

Internet posture (default ``none``; the house ships sealed):
``none`` pairs with a network that has no internet route;
``proxy:<url>`` bakes HTTPS_PROXY/HTTP_PROXY at birth so every occupant
(human or agent) lives behind the whitelist wall; ``open`` declares the
online sub-experiment switch (pair with a routed network). Sealing is
enforced by the NETWORK (docker --internal); this flag sets the one
door and keeps the intent explicit.

Value output: the container name (one line). File output: the seeded
workspace under --workspace. State output: a running container attached
to the given network/domain with /workspace mounted; a human enters it
with ``docker exec -it <name> bash``; sending an agent in is the
occupant package's verb (robocli.agents), not ours.

--mount/--env are GENERIC birth-time slots (docker mounts exist only at
container creation); the package does not interpret their values. ``up``
never builds images: a missing image is an instructive error.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import uuid
from pathlib import Path

import yaml

from robocli.sandbox import SandboxError

from . import workspace as _workspace

DEFAULT_IMAGE = "robocli-sandbox"


def up(config: Path, workspace: Path,
       image: str | None = None, network: str = "host",
       static_peer: str | None = None, ros_domain: int = 0,
       internet: str = "none", name: str | None = None,
       mounts: tuple[str, ...] = (), env: tuple[str, ...] = (),
       seed_workspace: bool = True, peers_xml: str | None = None) -> str:
    """Create the live sandbox; returns its container name.

    Give every container a FRESH workspace dir: seeding merges into an
    existing dir (stale files from a previous occupant would leak into
    the new session).
    """
    config = Path(config)
    if not config.is_file():
        raise SandboxError(f"config not found: {config}")
    try:
        cfg = yaml.safe_load(config.read_text())
    except yaml.YAMLError as e:
        raise SandboxError(f"config is not valid yaml: {config}: {e}")
    if not isinstance(cfg, dict):
        raise SandboxError(f"config is empty or not a mapping: {config}")
    # The config arrives ALREADY RESOLVED to the trial's suite view (the
    # conductor computes it once and distributes; ruling 2026-08-16) --
    # this package applies no overrides of its own.
    # Internet posture: the house ships SEALED; opening a door is the
    # caller's explicit decision. `--network` is the robot's net (DDS
    # reachability); `--internet` is the outside world. Orthogonal.
    if internet != "none" and not (internet == "open"
                                   or internet.startswith("proxy:")):
        raise SandboxError(
            f"--internet must be none, open, or proxy:<url> (got {internet!r})")
    proxy_env: list[str] = []
    if internet.startswith("proxy:"):
        url = internet.split(":", 1)[1]
        if not url:
            raise SandboxError("--internet proxy: needs a url "
                              "(e.g. proxy:http://robocli-proxy:8888)")
        # Baked at birth: humans entering the sandbox live under the
        # same posture as agents (no exec-time injection asymmetry).
        proxy_env = ["-e", f"HTTPS_PROXY={url}", "-e", f"HTTP_PROXY={url}"]
    image = image or cfg.get("machine", {}).get("body", {}).get(
        "sandbox_image", DEFAULT_IMAGE)
    if subprocess.run(["docker", "image", "inspect", image],
                      capture_output=True).returncode != 0:
        raise SandboxError(
            f"sandbox image {image!r} not found; build it first: "
            "python3 -m robocli.sandbox.build ... (up never builds)")
    name = name or f"sandbox-{uuid.uuid4().hex[:8]}"

    ws = Path(workspace).resolve()
    if seed_workspace:
        _workspace.seed(cfg, ws)
    # else: mount the dir AS-IS (replaying an archived workspace, custom
    # content); the caller owns what is inside.
    ws.mkdir(parents=True, exist_ok=True)  # no-template configs
    ws.chmod(0o777)  # container's non-root uid must be able to write

    if static_peer and not peers_xml:
        raise SandboxError(
            "static_peer needs the rendered peers profile too; the "
            "conductor renders it (assembly.FASTDDS_PEERS_XML) and "
            "passes peers_xml")
    peer_env = ["-e", f"ROS_DOMAIN_ID={ros_domain}"]
    if static_peer:
        # Multicast-less networks (docker internal nets) need a unicast
        # bootstrap peer; profile written after start (see below).
        peer_env += [
            "-e", f"ROS_STATIC_PEERS={static_peer}",
            "-e", "FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds-peers.xml",
        ]
    extra = []
    for m in mounts:
        extra += ["-v", m]
    for e in env:
        extra += ["-e", e]

    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    r = subprocess.run(
        ["docker", "run", "-d", "-i", "--tty", "--name", name,
         "--network", network, *peer_env, *proxy_env,
         "-v", f"{ws}:/workspace", *extra, image, "bash"],
        capture_output=True, text=True,
    )
    if r.returncode != 0:
        # Surface docker's own words (bad network name, malformed
        # --mount, ...); a swallowed stderr here cost blind debugging
        # in the bridge's early days.
        subprocess.run(["docker", "rm", "-f", name], capture_output=True)
        raise SandboxError(f"docker run failed for {name!r}:\n"
                           f"{r.stderr.strip()[-1000:]}")
    if static_peer:
        w = subprocess.run(
            ["docker", "exec", "-i", name, "sh", "-c",
             "cat > /tmp/fastdds-peers.xml"],
            input=peers_xml,
            text=True, capture_output=True,
        )
        if w.returncode != 0:
            raise SandboxError(_death_report(name, "peers-profile write failed"))
    # Birth verification: `docker run -d` succeeds even when the
    # container exits immediately (image without bash, broken
    # entrypoint); never hand back the name of a corpse.
    alive = subprocess.run(
        ["docker", "ps", "-q", "--filter", f"name=^{name}$"],
        capture_output=True, text=True).stdout.strip()
    if not alive:
        raise SandboxError(_death_report(name, "exited at birth"))
    return name


def _death_report(name: str, what: str) -> str:
    """Capture the corpse's last words, then reclaim it (auto-generated
    names would otherwise leak a container per failed attempt)."""
    logs = subprocess.run(["docker", "logs", "--tail", "20", name],
                          capture_output=True, text=True)
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    cry = (logs.stdout + logs.stderr).strip()[-1000:]
    return f"sandbox {name!r} {what}" + (f"; last output:\n{cry}" if cry else "")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", required=True, type=Path,
                    help="RESOLVED assembly yaml (the trial's suite view; "
                    "for hand runs without suite overrides the raw config "
                    "is identical)")
    ap.add_argument("--workspace", required=True, type=Path,
                    help="host dir to seed and mount at /workspace")
    ap.add_argument("--image", default=None,
                    help="seat image (default: config machine.body.sandbox_image, "
                    f"then {DEFAULT_IMAGE!r})")
    ap.add_argument("--network", default="host",
                    help="'host' (real machine, multicast) or a docker "
                    "network name")
    ap.add_argument("--static-peer", default=None,
                    help="unicast bootstrap peer hostname for "
                    "multicast-less networks")
    ap.add_argument("--ros-domain", type=int, default=0)
    ap.add_argument("--internet", default="none",
                    help="outside-world access: none (default, sealed) | "
                    "proxy:<url> (whitelist wall, baked at birth) | open")
    ap.add_argument("--name", default=None,
                    help="container name (default: sandbox-<hex8>); an "
                    "existing name is replaced")
    ap.add_argument("--mount", action="append", default=[],
                    metavar="SRC:DST", help="generic birth-time mount "
                    "(repeatable; the package does not interpret it)")
    ap.add_argument("--env", action="append", default=[], metavar="K=V",
                    help="generic birth-time env var (repeatable)")
    ap.add_argument("--no-seed", action="store_true",
                    help="mount the workspace dir as-is (no template "
                    "seeding; replay/custom workspaces)")
    args = ap.parse_args()
    try:
        name = up(config=args.config, workspace=args.workspace,
                  image=args.image,
                  network=args.network, static_peer=args.static_peer,
                  ros_domain=args.ros_domain, internet=args.internet,
                  name=args.name,
                  mounts=tuple(args.mount), env=tuple(args.env),
                  seed_workspace=not args.no_seed)
    except SandboxError as e:
        raise SystemExit(str(e))
    print(name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
