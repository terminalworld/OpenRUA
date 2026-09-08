"""``openrua run <robot> [prompt]``: up, agent, down in one command.

The shape of ``docker run``: bring the robot up with a sandbox terminal
on it, open the coding agent there, and power everything off when the
agent exits. ``up`` / ``agent`` / ``down`` are the same three steps for
a robot that should stay up between sessions.
"""

from __future__ import annotations

import subprocess

from openrua import agents
from openrua.cli import state
from openrua.cli.commands import up
from openrua.errors import UnavailableError


def run(args) -> int:
    session = up.open_session(args)
    try:
        st = state.load(args.name, args.home)
        adapter = agents.get(args.agent or st["agent"], args.home)
        argv = adapter.interactive_argv(
            st["sandbox"], args.model or st["model"], st["proxy"],
            options=st.get("options"), prompt=args.prompt)
        if argv is None:
            raise UnavailableError(
                f"{adapter.name} has no interactive mode",
                hint=f"bring the robot up with `openrua up` and open a shell on it: "
                     f"docker exec -it -u robot -w /workspace {st['sandbox']} bash")
        print(session.header("run", args.prompt), flush=True)
        print("[run] the robot powers off when the agent exits", flush=True)
        return subprocess.call(argv)
    finally:
        session.power_off()


def add_parser(sub) -> None:
    p = sub.add_parser("run", help="bring a robot up, open the agent on it, power off after",
                       description="Bring a robot up, open the coding agent on its "
                       "terminal with PROMPT as the opening message, and power the robot "
                       "off when the agent exits. Same steps as up, agent, down.")
    p.add_argument("robot", nargs="?", default=None,
                   help="robot profile: a name (openrua robots) or a path; "
                   "default: --bench's robot, else the user config's default")
    p.add_argument("prompt", nargs="?", default=None, help="opening message for the agent")
    p.add_argument("--model", default=None, help="model (default: the config's)")
    up.add_options(p)
    p.set_defaults(fn=run)
