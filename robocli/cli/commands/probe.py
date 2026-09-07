"""``robocli probe``: read a real robot's graph and print a profile draft."""

from __future__ import annotations

import subprocess
import tempfile
import uuid
from pathlib import Path

import yaml

from robocli.cli import state
from robocli.errors import UsageError
from robocli.robot.real.probe import draft_profile, read_graph
from robocli.runner.bringup import sandbox_reachability
from robocli.sandbox.down import down as sandbox_down
from robocli.sandbox.up import DEFAULT_IMAGE as DEFAULT_SANDBOX_IMAGE
from robocli.sandbox.up import up as sandbox_up


def _discovery(args) -> dict:
    given = [k for k in ("host", "static_peers", "discovery_server") if getattr(args, k)]
    if len(given) != 1:
        raise UsageError("say how to reach the graph: exactly one of --host, "
                         "--static-peers, --discovery-server")
    if args.host:
        return {"network": "host"}
    if args.static_peers:
        return {"static_peers": [p.strip() for p in args.static_peers.split(",") if p.strip()]}
    return {"discovery_server": args.discovery_server}


def _exec_in(sandbox: str):
    def run(cmd: str) -> str:
        r = subprocess.run(
            ["docker", "exec", sandbox, "bash", "-c", cmd],
            capture_output=True, text=True, timeout=120)
        return r.stdout
    return run


def run(args) -> int:
    if args.name:
        st = state.load(args.name, args.home)
        print(draft_profile(read_graph(_exec_in(st["sandbox"])),
                            {"network": "host"}), end="")
        return 0
    discovery = _discovery(args)
    backend = {"kind": "real", "discovery": discovery}
    reach = sandbox_reachability(backend, "host", "probe", "")
    reach["internet"] = "none"
    name = f"robocli-probe-{uuid.uuid4().hex[:6]}"
    with tempfile.TemporaryDirectory(prefix="robocli-probe-") as tmp:
        sandbox_up({"machine": {"workspace_template": None}}, Path(tmp) / "ws",
                   image=args.image or DEFAULT_SANDBOX_IMAGE, ros_domain=args.ros_domain,
                   name=name, seed_workspace=False, **reach)
        try:
            print(draft_profile(read_graph(_exec_in(name)), discovery), end="")
        finally:
            sandbox_down(name)
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("probe", help="draft a robot profile from a live ROS 2 graph",
                       description="Start a throwaway sandbox that can see the robot's "
                       "graph (or reuse a live robot's), read its topics, actions, "
                       "services and /robot_description, and print a profile draft "
                       "with TODO on what only you know. Redirect it to a file and edit.")
    p.add_argument("--host", action="store_true", help="the graph is on the host network")
    p.add_argument("--static-peers", default=None, metavar="ADDR[,ADDR]",
                   help="reach the graph through these unicast peers")
    p.add_argument("--discovery-server", default=None, metavar="HOST:PORT",
                   help="reach the graph through a Fast DDS discovery server")
    p.add_argument("--ros-domain", type=int, default=0, help="ROS_DOMAIN_ID of the graph")
    p.add_argument("--image", default=None,
                   help="sandbox image to probe from (default: robocli-sandbox)")
    p.add_argument("--name", default=None,
                   help="probe from a robot already up under this handle instead")
    p.set_defaults(fn=run)
