"""Verb ``up``: a reachable machine + its config -> one live sandbox.

    python3 -m robocli.sandbox.up --config <config.yaml> \
        --workspace <dir> [--task-suite S] [--image robocli-sandbox] \
        [--network host|<docker-net>] [--static-peer <hostname>] \
        [--ros-domain N] [--internet none|proxy:<url>|open] \
        [--name <container>] [--mount SRC:DST ...] [--env K=V ...]

Internet access (default ``none``): ``none`` pairs with a network that
has no internet route; ``proxy:<url>`` sets HTTPS_PROXY/HTTP_PROXY at
creation so everyone in the container (person or agent) goes through
the whitelist proxy; ``open`` pairs with a routed network. Isolation is
enforced by the network (docker --internal); this flag sets the one
route out and keeps the intent explicit.

Value output: the container name (one line). File output: the seeded
workspace under --workspace. State output: a running container attached
to the given network/domain with /workspace mounted; a person enters it
with ``docker exec -it <name> bash``; launching an agent in it is the
agents package's job.

--mount/--env are generic creation-time slots (docker mounts exist only
at container creation); the package does not interpret their values.
``up`` never builds images: a missing image is an instructive error.
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


def load(config: Path) -> dict:
    """The resolved config file as a dict, with instructive errors."""
    config = Path(config)
    if not config.is_file():
        raise SandboxError(f"config not found: {config}")
    try:
        cfg = yaml.safe_load(config.read_text())
    except yaml.YAMLError as e:
        raise SandboxError(f"config is not valid yaml: {config}: {e}")
    if not isinstance(cfg, dict):
        raise SandboxError(f"config is empty or not a mapping: {config}")
    return cfg


def up(cfg: dict, workspace: Path, *,
       image: str = DEFAULT_IMAGE, network: str = "host",
       static_peer: str | None = None, ros_domain: int = 0,
       internet: str = "none", name: str | None = None,
       mounts: tuple[str, ...] = (), env: tuple[str, ...] = (),
       run_args: tuple[str, ...] = (),
       seed_workspace: bool = True, peers_xml: str | None = None) -> str:
    """Create the live sandbox; returns its container name.

    ``cfg`` is the resolved config (it seeds the workspace); everything
    about the container itself arrives as a parameter. Give every
    container a fresh workspace dir: seeding merges into an existing
    dir, and stale files from a previous session would leak into the
    new one.
    """
    # `--network` is the robot's net (DDS reachability); `--internet`
    # is the outside world. Orthogonal.
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
        # Set at creation: a person entering the sandbox has the same
        # network access as an agent.
        proxy_env = ["-e", f"HTTPS_PROXY={url}", "-e", f"HTTP_PROXY={url}"]
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
            "runner renders it (bringup.peers_profile) and "
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

    # run_args: this machine's own flags for the sandbox container
    # (rootless podman's --userns=keep-id), passed through as written.
    extra += list(run_args)

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
    # entrypoint); never hand back the name of a dead container.
    alive = subprocess.run(
        ["docker", "ps", "-q", "--filter", f"name=^{name}$"],
        capture_output=True, text=True).stdout.strip()
    if not alive:
        raise SandboxError(_death_report(name, "exited at creation"))
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
                    help="resolved config yaml (the trial's suite view; "
                    "for hand runs without suite overrides the raw config "
                    "is identical)")
    ap.add_argument("--workspace", required=True, type=Path,
                    help="host dir to seed and mount at /workspace")
    ap.add_argument("--image", default=None,
                    help="sandbox image (default: config machine.backend.sandbox_image, "
                    f"then {DEFAULT_IMAGE!r})")
    ap.add_argument("--network", default="host",
                    help="'host' (real machine, multicast) or a docker "
                    "network name")
    ap.add_argument("--static-peer", default=None,
                    help="unicast bootstrap peer hostname for "
                    "multicast-less networks")
    ap.add_argument("--ros-domain", type=int, default=0)
    ap.add_argument("--internet", default="none",
                    help="outside-world access: none (default) | "
                    "proxy:<url> (whitelist proxy, set at creation) | open")
    ap.add_argument("--name", default=None,
                    help="container name (default: sandbox-<hex8>); an "
                    "existing name is replaced")
    ap.add_argument("--mount", action="append", default=[],
                    metavar="SRC:DST", help="generic creation-time mount "
                    "(repeatable; the package does not interpret it)")
    ap.add_argument("--env", action="append", default=[], metavar="K=V",
                    help="generic creation-time env var (repeatable)")
    ap.add_argument("--no-seed", action="store_true",
                    help="mount the workspace dir as-is (no template "
                    "seeding; replay/custom workspaces)")
    args = ap.parse_args()
    try:
        cfg = load(args.config)
        name = up(cfg, args.workspace,
                  image=args.image or cfg.get("machine", {}).get("backend", {}).get(
                      "sandbox_image", DEFAULT_IMAGE),
                  run_args=tuple(cfg.get("sandbox", {}).get("run_args", [])),
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
