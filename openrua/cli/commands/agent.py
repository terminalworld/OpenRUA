"""``openrua agent``: open the coding agent interactively on a live robot's terminal."""

from __future__ import annotations

import os

from openrua import agents
from openrua.cli import state
from openrua.cli.state import DEFAULT_NAME
from openrua.errors import UnavailableError


def run(args) -> int:
    st = state.load(args.name, args.home)
    agent = agents.get(args.agent or st["agent"], args.home)
    argv = agent.interactive_argv(
        st["sandbox"], args.model or st["model"], st["proxy"],
        options=st.get("options"), prompt=args.prompt)
    if argv is None:
        raise UnavailableError(
            f"{agent.name} has no interactive mode",
            hint=f"open a shell on the terminal instead: docker exec -it -u robot "
                 f"-w /workspace {st['sandbox']} bash")
    os.execvp(argv[0], argv)
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("agent", help="open a coding agent on the robot's terminal",
                       description="Open the configured coding agent interactively on a "
                       "live robot's terminal (docker exec into its sandbox).")
    p.add_argument("prompt", nargs="?", default=None, help="opening message")
    p.add_argument("--name", default=DEFAULT_NAME, help=f"the robot's handle (default: {DEFAULT_NAME})")
    p.add_argument("--agent", default=None, help="agent (default: the one `up` opened)")
    p.add_argument("--model", default=None, help="model (default: the one `up` recorded)")
    p.set_defaults(fn=run)
