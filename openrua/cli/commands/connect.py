"""``openrua connect``: connect a real robot, with your coding agent doing the work.

A session opens whose terminal reaches this computer's docker, network
and devices, working in the robot's folder. Your agent finds out what
the robot is, builds and starts its ROS 2 drivers and writes the robot's
config, asking you for what only a person can do and before each
command that changes something; OpenRUA checks each step. When every
check has passed, the session moves to the robot's own terminal and the
conversation goes on there.
"""

from __future__ import annotations

import sys
from pathlib import Path

from openrua.cli import docker_setup, preparation, workspace
from openrua.cli.commands import login, start
from openrua.config import paths
from openrua.errors import UsageError
from openrua.runner import history, service


def launch(home: Path, robot: str, folder: Path | None = None, agent: str | None = None,
           model: str | None = None, distro: str = "jazzy", name: str | None = None, project: Path | None = None):
    """Start the connecting session's service; returns (client, session name)."""
    folder = (folder or paths.home(home) / "robots" / robot).expanduser()
    name = name or history.new_id()
    if paths.sandbox_dir(name, home).exists():
        raise UsageError(f"session name {name!r} already has retained or running work",
                         hint="choose a new --name")
    preparation.prepare_connect(home, agent, distro)
    argv = [sys.executable, "-m", "openrua", "--home", str(home), "serve",
            "--connect", str(folder.resolve()), "--name", name, "--distro", distro,
            "--permissions", "ask"]
    if project is not None:
        argv += ['--workspace', str(project)]
    for key, value in (("agent", agent), ("model", model)):
        if value:
            argv += [f"--{key}", value]
    print(f"[connect] {robot}: folder {folder}; starting the connecting terminal", flush=True)
    client = service.start(argv, paths.sandbox_dir(name, home) / "endpoint.json",
                           paths.launch_dir(name, home) / "service.log",
                           paths.launch_dir(name, home) / "start.lock")
    return client, name


def run(args) -> int:
    selected = workspace.choose(args.workspace or Path.cwd(), args.home, accept=args.trust_workspace)
    if selected is None:
        return 0
    if sys.stdin.isatty() and sys.stdout.isatty():
        docker_setup.ensure()
        login.ensure(args.home, args.agent)
    client, name = launch(args.home, args.robot, Path(args.dir) if args.dir else None,
                          args.agent, args.model, args.distro, args.name, project=selected)
    return start.open_terminal(args.home, client, name)


def add_parser(sub) -> None:
    p = sub.add_parser("connect", help="connect a real robot, your coding agent doing the work",
                       description=__doc__)
    p.add_argument("robot", nargs="?", default="my-robot",
                   help="the robot's name (default: my-robot)")
    p.add_argument("--dir", default=None,
                   help="the robot's folder (default: <home>/robots/<robot>); an existing "
                   "one goes on from what is in it")
    p.add_argument("--agent", default=None, help="coding agent (default: the configured one)")
    p.add_argument("--model", default=None, help="model (default: the agent's)")
    p.add_argument("--distro", default="jazzy", choices=("jazzy", "humble"),
                   help="the connecting terminal's ROS 2; the robot's own comes from its drivers")
    p.add_argument("--name", default=None, help="session ID (default: a new one)")
    p.add_argument('--workspace', default=None, help='robot project after connection (default: the current directory)')
    p.add_argument('--trust-workspace', action='store_true', help='explicitly trust the project without a question')
    p.set_defaults(fn=run)
