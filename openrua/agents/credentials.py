"""Staging an agent's login for one sandbox.

The profile directory (settings and other non-rotating state) is always
a fresh per-sandbox copy, made where the caller says (the sandbox's own
directory under ``~/.openrua/sandboxes/``; it goes when the sandbox
goes). The credentials file is never copied: OAuth refresh tokens
rotate, and two copies of the file invalidate each other, so the one
file is bind-mounted into every sandbox that needs it. A sandbox that
authenticates with a token of its own needs no credentials file at all.
"""

from __future__ import annotations

import shutil
from pathlib import Path

from openrua.agents.base import Agent


def prepare_profile(creds_home: Path, agent: Agent, dest: Path,
                    require_credentials: bool = True) -> tuple[Path, Path | None]:
    """Copy the agent's profile (never its credentials file) into
    ``dest``, emptied first. Returns (dest, shared_credentials_file_or_None).

    With ``require_credentials`` the credentials file must exist and is
    returned as a path for bind-mounting. Without it (a token
    authenticates the sandbox) the second element is None. An agent
    without a profile-directory login gets an empty directory and None.
    The sandbox runs as this user, so the directory keeps this user's
    permissions.
    """
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)
    creds = agent.credentials
    if creds is None:
        return dest, None
    creds_file: Path | None = creds_home / creds.filename
    if not require_credentials:
        creds_file = None
    elif not creds_file.exists():
        raise RuntimeError(f"credentials missing: {agent.login_hint(creds_home)}")
    for pattern in ("*.json", ".*.json"):
        for f in creds_home.glob(pattern):
            if f.name != creds.filename:
                shutil.copy2(f, dest / f.name)
    return dest, creds_file
