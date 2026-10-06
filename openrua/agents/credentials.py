"""Staging an agent's login for one sandbox.

The profile directory (settings and other non-rotating state) is always
a fresh per-sandbox copy, made where the caller says (the sandbox's own
directory under ``~/.openrua/sandboxes/``; interactive sessions retain
it after shutdown until explicitly deleted). The credentials file is never copied: OAuth refresh tokens
rotate, and two copies of the file invalidate each other, so the one
file is bind-mounted into every sandbox that needs it. A sandbox that
authenticates with a token of its own needs no credentials file at all.
"""

from __future__ import annotations

from pathlib import Path
from collections.abc import Mapping

from openrua.agents.base import Agent, _copy_profile


def resolve_profile(agent: Agent, configured: str | Path | None, alias: Path, *,
                    user_home: Path, environment: Mapping[str, str], link: bool = False) -> Path:
    """Resolve the login without changing explicit or existing account selections.

    Native locations belong to the agent manifest. Read-only callers discover
    the same source; launchers can create an absent alias without copying secrets.
    """
    def expand(value):
        value = str(value)
        return user_home / value[2:] if value.startswith('~/') else Path(value)

    if configured:
        return expand(configured)
    creds = getattr(agent, 'credentials', None)
    if creds is None:
        return alias
    override = environment.get(creds.config_env)
    if not override and (alias.exists() or alias.is_symlink()):
        return alias
    if not override and not creds.native_dir:
        return alias
    source = expand(override) if override else user_home / creds.native_dir
    if alias.exists() or alias.is_symlink():
        # Honor the native override without replacing a different stored account.
        return alias if alias.resolve() == source.resolve() else source
    login = agent.inspect_login(source) if link else None
    if login is not None and login.available:
        alias.parent.mkdir(parents=True, exist_ok=True)
        try:
            alias.symlink_to(source.resolve(), target_is_directory=True)
        except FileExistsError:
            # A concurrent launch selected an account first. Never overwrite it.
            if override and alias.resolve() != source.resolve():
                return source
        return alias
    return source


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
    return _copy_profile(creds_home, dest, agent.credentials,
                         require_credentials, agent.login_hint(creds_home))
