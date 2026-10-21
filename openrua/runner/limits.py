"""What the agent may reach beyond the robot, as command-line flags.

The three limits are ``config.schema.Limits``; a free session (openrua,
run, up, serve) starts from none of them and a trial (openrua bench)
from its protocol's. A flag left unset keeps that default.
"""

from __future__ import annotations

import argparse

LIMITS = ("internet", "web_search", "account_connectors")


def add_limit_options(p, default: str) -> None:
    """The three limits as flags; ``default`` says what an unset one means
    for the command."""
    p.add_argument("--internet", choices=("open", "model-only"), default=None,
                   help=f"open, or only the agent's model service and login, through "
                   f"a whitelist proxy ({default})")
    p.add_argument("--web-search", action=argparse.BooleanOptionalAction, default=None,
                   help=f"the agent CLI's own web search and fetch tools ({default})")
    p.add_argument("--account-connectors", action=argparse.BooleanOptionalAction,
                   default=None, help="what your account brings beyond the CLI: its "
                   f"connectors or apps, synced skills and plugins ({default})")


def limit_overrides(args) -> dict:
    """The limits named on the command line (the rest keep their defaults)."""
    return {key: getattr(args, key) for key in LIMITS if getattr(args, key, None) is not None}
