"""``openrua down``: power a robot and its terminal off from outside."""

from __future__ import annotations

from openrua import robot
from openrua.config import paths
from openrua.sessions.client import Client
from openrua.errors import UnavailableError
from openrua.cli import state
from openrua.cli.state import DEFAULT_NAME
from openrua.sandbox.down import down as sandbox_down


def run(args) -> int:
    endpoint = paths.sandbox_dir(args.name, args.home) / "endpoint.json"
    if endpoint.exists():
        try:
            Client.from_file(endpoint).end()
        except (OSError, ValueError, RuntimeError) as exc:
            raise UnavailableError(str(exc), hint="inspect the service before stopping its resources independently") from exc
        print("[down] shared session ended; files retained")
        return 0
    sim_name, sandbox_name = state.container_names(args.name)
    p = state.path(args.name, args.home)
    kind = state.load(args.name, args.home, require_running=False).get("backend", "sim") if p.is_file() else "sim"
    sandbox_down(sandbox_name)
    robot.down(sim_name, kind)
    state.stopped(args.name, args.home)
    print(f"[down] files retained at {p.parent}")
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("down", help="power a robot and its terminal off, retaining session files")
    p.add_argument("--name", default=DEFAULT_NAME, help=f"the robot's handle (default: {DEFAULT_NAME})")
    p.set_defaults(fn=run)
