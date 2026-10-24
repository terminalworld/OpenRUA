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

import argparse
import sys
from pathlib import Path

from openrua import agents
from openrua.cli import agent_settings, docker_setup, preparation, workspace
from openrua.cli.commands import login, start
from openrua.config import paths, projects, settings
from openrua.errors import UsageError
from openrua.runner import history, service


def save_values(home: Path, values: dict) -> None:
    """Save only the connecting agent's choices; the robot is not known yet."""
    adapter, selected = agent_settings.resolve(home, values['agent'] or None)
    if 'conversation' not in adapter.capabilities:
        raise UsageError(f'{adapter.name} does not support shared chat')
    auth = selected.get('auth') or {'mode': 'native'}
    if 'auth_mode' in values:
        auth = {'mode': values['auth_mode']}
        if values.get('api_key_file'):
            auth['key_file'] = str(Path(values['api_key_file']).expanduser().resolve())
    try:
        agents.api_key_file(adapter, auth)
    except ValueError as exc:
        raise UsageError(str(exc), hint='choose native login, or a readable API key file') from exc
    settings.update(home, {}, {'auth': auth}, agent=selected['name'])


def launch(home: Path, folder: Path | None = None, agent: str | None = None,
           model: str | None = None, distro: str = "jazzy", name: str | None = None, project: Path | None = None):
    """Start the connecting session's service; returns (client, session name)."""
    if project is None:
        raise UsageError('connecting a robot requires a project directory')
    folder = (project / (folder or Path('.')).expanduser()).resolve()
    if not folder.is_relative_to(project.resolve()):
        raise UsageError('the robot adaptation directory must be inside the project',
                         hint='choose a project subdirectory with --dir')
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
    print(f"[connect] project {project}; adaptation {folder}; starting the connecting terminal", flush=True)
    client = service.start(argv, paths.sandbox_dir(name, home) / "endpoint.json",
                           paths.launch_dir(name, home) / "service.log",
                           paths.launch_dir(name, home) / "start.lock")
    return client, name


def run(args) -> int:
    selected = workspace.choose(args.workspace or Path.cwd(), args.home, accept=args.trust_workspace)
    if selected is None:
        return 0
    chosen = projects.options(selected, dict(agent=args.agent, model=args.model), relative_to=Path.cwd())
    if sys.stdin.isatty() and sys.stdout.isatty():
        docker_setup.ensure()
        _, selected_agent = agent_settings.resolve(args.home, chosen['agent'])
        login.ensure(args.home, chosen['agent'], (selected_agent.get('auth') or {}).get('mode', 'native'))
    client, name = launch(args.home, Path(args.dir) if args.dir else None,
                          chosen['agent'], chosen['model'], args.distro, args.name, project=selected)
    return start.open_terminal(args.home, client, name)


def add_parser(sub) -> None:
    p = sub.add_parser("connect", help="connect a real robot, your coding agent doing the work",
                       description=__doc__)
    p.add_argument("--dir", default=None,
                   help='adaptation subdirectory inside the project (default: the project root)')
    p.add_argument("--agent", default=None, help="coding agent (default: the configured one)")
    p.add_argument("--model", default=None, help="model (default: the agent's)")
    p.add_argument("--distro", default="jazzy", choices=("jazzy", "humble"),
                   help="the connecting terminal's ROS 2; the robot's own comes from its drivers")
    p.add_argument("--name", default=None, help="session ID (default: a new one)")
    p.add_argument('-C', '--cd', '--workspace', dest='workspace', default=argparse.SUPPRESS,
                   help='project directory (default: the current directory)')
    p.add_argument('--trust-workspace', action='store_true', default=argparse.SUPPRESS,
                   help='explicitly trust the project without a question')
    p.set_defaults(fn=run)
