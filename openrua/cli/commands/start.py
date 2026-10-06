"""Open the product interface using the existing shared session service."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from openrua import agents, config, doctor
from openrua.cli.commands import chat, session, up
from openrua.config import paths, settings, startup
from openrua.cli.preparation import prepare
from openrua.errors import NotFound, UnavailableError, UsageError
from openrua.runner import service, history

# Command arguments belong here; the service launcher only sees explicit argv.
SELECTION = ('robot', 'sim', 'bench', 'agent', 'model')
RESOURCE_OPTIONS = ('task_suite', 'task_id', 'init_state', 'task', 'workspace', 'ros_domain', 'port')


def add_options(parser) -> None:
    group = parser.add_argument_group('start or reconnect (without a subcommand)')
    group.add_argument('--robot', default=None, help='robot name or profile path for a new session')
    up.add_options(group, default_name=None)
    group.add_argument('--model', default=None, help="model for a new session (default: the agent's configuration)")
    interface = group.add_mutually_exclusive_group()
    interface.add_argument('--gui', action='store_true', help='open the browser instead of terminal chat')
    interface.add_argument('--tui', choices=['pi'], default='pi', metavar='UI',
                           help='keyboard terminal frontend (default: pi, included)')
    interface.add_argument('--cli', action='store_true', help='use plain text chat instead of the TUI')
    group.add_argument('--port', type=int, default=None, help='local service port for a new session (default: a free port)')
    group.add_argument('--setup', action='store_true', help='edit shared defaults in the keyboard setup menu')
    group.add_argument('--resume', nargs='?', const='', default=None, metavar='ID',
                       help='choose a previous conversation, or reconnect by its ID/name')
    parser.set_defaults(init_state=None, name=None)


def values_for(args) -> tuple[dict[str, str], dict[str, str]]:
    user = config.load_user_config(paths.config_path(args.home))
    package = config.load_user_config(paths.package_config_path())
    chosen = args.agent or user.agent or package.agent
    models = {name: facts.model or '' for name, facts in user.agents.items()}
    values = {
        'robot': args.robot or user.robot or package.robot or '',
        'sim': args.sim or user.simulator or package.simulator or '',
        'bench': args.bench or user.benchmark or package.benchmark or '',
        'agent': chosen or '',
        'model': args.model or models.get(chosen, ''),
        'name': args.name or history.new_id(),
    }
    try:
        resolved = config.compose(args.robot, args.sim, args.bench, args.home, agent=args.agent)
    except (UsageError, NotFound, config.ConfigError):
        # An incomplete first-run selection stays editable in the menu.
        pass
    else:
        selected = resolved.cfg['agent']
        values.update(robot=resolved.robot if not resolved.robot.startswith('(') else '',
                      sim=resolved.simulator or '', bench=resolved.benchmark or '',
                      agent=selected['name'], model=args.model or selected.get('model') or '')
    return values, models


def save_values(home: Path, values: dict[str, str]) -> None:
    paths.sandbox_dir(values['name'], home)
    validate_values(home, values)
    settings.update(home, {key: values[field] or None for field, key in
                         [('robot', 'robot'), ('sim', 'simulator'), ('bench', 'benchmark'), ('agent', 'agent')]},
                    {'model': values['model'] or None})


def save_guided_values(home: Path, values: dict, environments: list[dict]) -> None:
    if not any(all(row['selection'][key] == values[key] for key in startup.FIELDS)
               for row in environments):
        raise UsageError('select a tested robot, simulator and benchmark combination',
                         hint='choose the robot first; custom profiles can use explicit CLI options')
    save_values(home, values)


def validate_values(home: Path, values: dict[str, str]) -> dict:
    """Validate the exact selection before writing defaults or touching resources."""
    cfg = config.compose(values['robot'], values['sim'], values['bench'], home,
                         agent=values['agent'] or None, model=values['model'] or None).cfg
    selected = cfg['agent']
    adapter = agents.get(selected['name'], home, version=selected.get('version'))
    agents.api_key_file(adapter, selected.get('auth'))
    if 'conversation' not in adapter.capabilities:
        raise UsageError(f'{adapter.name} does not support shared chat',
                         hint='use this plugin with openrua run or select a conversation-capable agent')
    return cfg


def check_values(home: Path, values: dict[str, str]) -> tuple[bool, str]:
    validate_values(home, values)
    report = doctor.run(robot=values['robot'], sim=values['sim'],
                        bench=values['bench'], agent_names=[values['agent']] if values['agent'] else None,
                        home=home)
    return report.ok, report.render()


def launch(args, values: dict[str, str]):
    name = values['name']
    directory = paths.sandbox_dir(name, args.home)
    if directory.exists():
        raise UsageError(f'session name {name!r} already has retained or running work',
                         hint='reconnect with --name only, or choose a new name; files are never overwritten')
    argv = [sys.executable, '-m', 'openrua', '--home', str(args.home), 'serve']
    if values['robot']:
        argv.append(values['robot'])
    for key in ('sim', 'bench', 'agent', 'model', 'name'):
        if values[key] or key in {'sim', 'bench'}:
            argv.extend(['--' + key, values[key]])
    for key in RESOURCE_OPTIONS:
        value = getattr(args, key)
        if value is not None:
            argv.extend(['--' + key.replace('_', '-'), str(value)])
    logs = paths.launch_dir(name, args.home)
    return service.start(argv, directory / 'endpoint.json', logs / 'service.log', logs / 'start.lock')


def open_terminal(home, client, name: str, frontend: str = 'pi') -> int:
    from openrua.terminal.launcher import chat as terminal_chat
    return terminal_chat(client, name, lambda: history.entries(home),
                         lambda name: history.open_session(home, name))


def choose_history(home, frontend: str = 'pi'):
    from openrua.terminal.launcher import choose_history as terminal_history
    return terminal_history(lambda: history.entries(home), lambda name: history.open_session(home, name))


def open_interface(args, client, name: str) -> int:
    if args.gui or args.cli:
        if getattr(client, 'read_only', False):
            raise UsageError('this conversation is available as read-only history',
                             hint=f'openrua --resume {name} opens its retained transcript in the TUI')
        if args.gui:
            return session.run(argparse.Namespace(home=args.home, name=name, action='web', no_open=False))
        return chat.run(argparse.Namespace(home=args.home, name=name, tui=False,
                                           message=None, follow=False, after=0))
    return open_terminal(args.home, client, name, args.tui)


def run(args) -> int:
    try:
        return _run(args)
    except (RuntimeError, OSError, ValueError) as exc:
        raise UnavailableError(str(exc), hint='the session may still be running; inspect it with --resume') from exc


def _run(args) -> int:
    if not (args.gui or args.cli) or args.setup or args.resume == '':
        from openrua.terminal.launcher import check_terminal
        try:
            check_terminal()
        except RuntimeError as exc:
            raise UnavailableError(str(exc)) from exc
    if args.resume is not None:
        if args.name is not None or args.setup or any(getattr(args, key) is not None for key in SELECTION + RESOURCE_OPTIONS):
            raise UsageError('--resume cannot be combined with new-session settings', hint='resume by ID, or start a new session separately')
        if args.resume:
            try:
                selected = history.open_session(args.home, args.resume)
            except (OSError, RuntimeError, ValueError) as exc:
                raise UnavailableError(str(exc), hint='use openrua --resume to find retained conversations') from exc
        else:
            if not sys.stdin.isatty() or not sys.stdout.isatty():
                raise UsageError('history selection needs an interactive terminal', hint='use openrua --resume ID to select a conversation explicitly')
            selected = choose_history(args.home, args.tui)
        return open_interface(args, *selected) if selected else 0
    if args.name is None:
        args.name = history.new_id()
    endpoint = paths.sandbox_dir(args.name, args.home) / 'endpoint.json'
    overrides = any(getattr(args, key) is not None for key in SELECTION + RESOURCE_OPTIONS)
    if endpoint.exists() and not args.setup:
        if overrides:
            raise UsageError('a shared session already uses this name; startup options cannot change it',
                             hint='connect with --name only, or choose a new --name for those settings')
        try:
            client = service.connect(endpoint)
        except (OSError, ValueError, KeyError, RuntimeError) as exc:
            raise UnavailableError(str(exc), hint='inspect the retained service log and session status before restarting') from exc
        return open_interface(args, client, args.name)
    values, models = values_for(args)
    if args.setup or not (values['robot'] or values['bench']) or (not args.gui and not args.cli):
        if not sys.stdin.isatty() or not sys.stdout.isatty():
            raise UsageError('terminal setup needs an interactive terminal',
                             hint='use openrua config set or edit config.yaml, then use --gui or --cli')
        environments = startup.verified_environments()
        if values['robot'] in startup.options(environments, {}, 'robot'):
            values = startup.select(environments, values, 'robot', values['robot'])
        choices = {'agent': [entry.name for entry in paths.available('agents')
                             if 'conversation' in agents.get(entry.name, args.home).capabilities]}
        choices.update({field: startup.options(environments, values, field) for field in startup.FIELDS})
        from openrua.terminal.launcher import setup as terminal_setup
        result = terminal_setup(values, choices, str(paths.config_path(args.home)),
                                lambda values: save_guided_values(args.home, values, environments),
                                lambda values: check_values(args.home, values),
                                lambda values: launch(args, values), models,
                                lambda: history.entries(args.home),
                                lambda name: history.open_session(args.home, name),
                                environments=environments, prepare=lambda v, progress: prepare(args.home, v, progress))
        if result is None:
            return 0
        client, name = result
    else:
        validate_values(args.home, values)
        prepare(args.home, values)
        ok, report = check_values(args.home, values)
        if not ok:
            raise UnavailableError(report, hint='complete the preparation steps, then start again')
        client, name = launch(args, values), args.name
    return open_interface(args, client, name)
