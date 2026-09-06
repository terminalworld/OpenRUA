"""Staging an agent's login for one sandbox.

The profile directory (settings and other non-rotating state) is always
a fresh per-sandbox copy. The credentials file is never copied: OAuth
refresh tokens rotate, and two copies of the file invalidate each other,
so the one file is bind-mounted into every sandbox that needs it. A
sandbox that authenticates with a token of its own needs no credentials
file at all.
"""

from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path

from robocli.agents.base import Agent


def prepare_profile(creds_home: Path, agent: Agent, require_credentials: bool = True
                    ) -> tuple[Path, Path | None]:
    """Returns (profile_copy_dir, shared_credentials_file_or_None); the
    caller owns deleting the copy dir.

    With ``require_credentials`` the credentials file must exist and is
    returned as a path for bind-mounting. Without it (a token
    authenticates the sandbox) the second element is None. An agent
    without a profile-directory login gets an empty copy dir and None.
    """
    cfg_dir = Path(tempfile.mkdtemp(prefix="robocli-agentcfg-"))
    creds = agent.credentials
    if creds is None:
        return cfg_dir, None
    creds_file: Path | None = creds_home / creds.filename
    if not require_credentials:
        creds_file = None
    elif not creds_file.exists():
        raise RuntimeError(f"credentials missing: {agent.login_hint(creds_home)}")
    for pattern in ("*.json", ".*.json"):
        for f in creds_home.glob(pattern):
            if f.name != creds.filename:
                shutil.copy2(f, cfg_dir / f.name)
    subprocess.run(["chmod", "-R", "777", str(cfg_dir)])
    return cfg_dir, creds_file
