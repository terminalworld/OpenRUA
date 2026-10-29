"""``openrua login``: sign your coding agent in on this computer.

The agent's own sign-in (its manifest's ``credentials.login``) runs
inside OpenRUA's sandbox image, where its CLI is installed already, so
nothing else is installed here. It shows a link to open in any browser,
and a code to paste back or to enter on the page. The login is written
to the directory every session reads it from (the CLI's own in your home
directory, unless your config names another), and sessions use it from
the next start on. ``openrua`` offers this by itself when the agent you
start with is not signed in.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Callable

from openrua import agents
from openrua.cli import docker_setup
from openrua.cli.agent_settings import resolve as _settings
from openrua.config import paths
from openrua.errors import AuthError, UsageError


def directory(home: Path, adapter: agents.Agent, settings: dict) -> Path:
    """Where sessions read this agent's login: the same choice doctor and
    a session's start make."""
    return agents.resolve_profile(adapter, settings.get("credentials_dir"),
                                  paths.credentials_dir(home) / adapter.name,
                                  user_home=Path.home(), environment=os.environ)


def image_with(name: str) -> str | None:
    """An image of OpenRUA's sandbox that carries this agent's CLI."""
    r = subprocess.run(["docker", "images", "--filter", f"label=openrua.agent.{name}.install_sha256",
                        "--format", "{{.Repository}}:{{.Tag}}"], capture_output=True, text=True)
    tags = [t for t in r.stdout.split() if "<none>" not in t]
    return sorted(tags, key=lambda t: (not t.startswith("openrua-sandbox-jazzy"), t))[0] if tags else None


def sign_in(home: Path, adapter: agents.Agent, where: Path, *, image: str | None = None) -> None:
    """Run the agent's own sign-in in a sandbox image, writing to ``where``;
    use an explicitly prepared image, or find/build a shared sandbox."""
    creds = adapter.credentials
    title = adapter.title or adapter.name
    if image is None:
        image = image_with(adapter.name)
    if image is None:
        from openrua.cli.commands import build
        print(f"Preparing the sandbox {title} runs in (first time only, a few minutes)...",
              flush=True)
        image, _ = build._sandbox([adapter.name], None, "jazzy", None, None, home)
    where.mkdir(parents=True, exist_ok=True, mode=0o700)
    print(f"Signing {title} in; its login goes to {where}.", flush=True)
    subprocess.run(["docker", "run", "--rm", "-it", "--entrypoint", "",
                    "-u", f"{os.getuid()}:{os.getgid()}", "-e", "HOME=/tmp",
                    "-e", f"{creds.config_env}={creds.mount_point}",
                    "-v", f"{where.resolve()}:{creds.mount_point}", image, *creds.login])
    login = adapter.inspect_login(where)
    if login is None or not login.available:
        raise AuthError(f"{title} did not finish signing in",
                          hint=f"run openrua login {adapter.name} to try again")
    print(f"{title} is signed in.", flush=True)


def _yes(answer: str, default: bool) -> bool:
    answer = answer.strip().lower()
    return default if not answer else answer in ("y", "yes")


def ensure(home: Path, name: str | None, auth_mode: str = "native",
           ask: Callable[[str], str] = input, interact=None, *, image: str | None = None) -> None:
    """Before a session starts at a terminal: when its agent signs in
    natively and is not signed in, offer to sign it in now. Declining is a
    AuthError with the command for later."""
    if auth_mode == "api":
        return
    adapter, settings = _settings(home, name)
    creds = adapter.credentials
    if creds is None or not creds.login:
        return
    where = directory(home, adapter, settings)
    login = adapter.inspect_login(where)
    if login is None or login.available:
        return
    if interact is not None:
        return interact(lambda: ensure(home, name, auth_mode, ask, image=image))
    title = adapter.title or adapter.name
    print(f"\n{title} is not signed in on this computer yet. OpenRUA works through your "
          f"{title} account.", flush=True)
    if not _yes(ask("Sign in now? [Y/n] "), default=True):
        raise AuthError(f"{title} is not signed in on this computer",
                          hint=f"run openrua login {adapter.name}")
    sign_in(home, adapter, where, **({'image': image} if image is not None else {}))


def run(args) -> int:
    if not (sys.stdin.isatty() and sys.stdout.isatty()):
        raise UsageError("signing in needs an interactive terminal",
                         hint="run openrua login in a terminal")
    docker_setup.ensure()
    adapter, settings = _settings(args.home, args.agent)
    creds = adapter.credentials
    if creds is None or not creds.login:
        raise UsageError(f"{adapter.name} names no sign-in command",
                         hint=adapter.login_hint(paths.credentials_dir(args.home) / adapter.name))
    where = Path(args.dir).expanduser() if args.dir else directory(args.home, adapter, settings)
    login = adapter.inspect_login(where)
    title = adapter.title or adapter.name
    if login is not None and login.available:
        print(f"{title} is already signed in on this computer ({where}).")
        if not _yes(input("Sign in again? [y/N] "), default=False):
            return 0
    sign_in(args.home, adapter, where)
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("login", help="sign your coding agent in on this computer",
                       description=__doc__)
    p.add_argument("agent", nargs="?", default=None,
                   help="the agent to sign in (default: the configured one; openrua agents lists them)")
    p.add_argument("--dir", default=None,
                   help="the login directory (default: the one sessions read)")
    p.set_defaults(fn=run)
