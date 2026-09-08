"""``openrua down``: power a robot and its terminal off from outside."""

from __future__ import annotations

from openrua import robot
from openrua.cli import state
from openrua.cli.state import DEFAULT_NAME
from openrua.sandbox.down import down as sandbox_down


def run(args) -> int:
    sim_name, sandbox_name = state.container_names(args.name)
    p = state.path(args.name, args.home)
    kind = state.load(args.name, args.home).get("backend", "sim") if p.is_file() else "sim"
    sandbox_down(sandbox_name)
    robot.down(sim_name, kind)
    state.forget(args.name, args.home)
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("down", help="power a robot and its terminal off")
    p.add_argument("--name", default=DEFAULT_NAME, help=f"the robot's handle (default: {DEFAULT_NAME})")
    p.set_defaults(fn=run)
