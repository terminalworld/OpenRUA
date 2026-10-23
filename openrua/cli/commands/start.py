"""Open the product interface using the existing shared session service."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from openrua import agents, config, doctor
from openrua.cli.commands import chat, connect, login, session, up
from openrua.config import paths, projects, settings, startup
from openrua.cli import agent_settings, docker_setup, updates, workspace
from openrua.cli.preparation import prepare
from openrua.errors import NotFound, UnavailableError, UsageError
from openrua.runner import service, history, live_state
from openrua.runner.limits import limit_overrides

# Command arguments belong here; the service launcher only sees explicit argv.
SELECTION = projects.FIELDS
RESOURCE_OPTIONS = ('task_suite', 'task_id', 'init_state', 'task', 'workspace', 'ros_domain', 'permissions', 'port')


def add_options(parser) -> None:
    group = parser.add_argument_group('start or reconnect (without a subcommand)')
    group.add_argument('--robot', default=None, help='robot name or profile path for a new session')
    up.add_options(group, default_name=None)
    up.add_permissions_option(group)
    group.add_argument('--model', default=None, help="model for a new session (default: the agent's configuration)")
    interface = group.add_mutually_exclusive_group()
    interface.add_argument('--gui', action='store_true', help='open the browser instead of terminal chat')
    interface.add_argument('--tui', choices=['pi'], default='pi', metavar='UI',
                           help='keyboard terminal frontend (default: pi, included)')
    interface.add_argument('--cli', action='store_true', help='use plain text chat instead of the TUI')
    group.add_argument('--port', type=int, default=None, help='local service port for a new session (default: a free port)')
    group.add_argument('--setup', action='store_true', help='edit shared defaults in the keyboard setup menu')
    group.add_argument('--resume', nargs='?', const='', default=None, metavar='ID',
                       help='resume a conversation in this workspace, optionally by ID/name')
    group.add_argument('--all', action='store_true',
                       help='with --resume, show conversations from all project directories')
    parser.set_defaults(init_state=None, name=None)


def values_for(args) -> tuple[dict[str, str], dict[str, str]]:
    local = projects.selection(Path(args.workspace)) if getattr(args, 'workspace', None) else {}
    robot, sim, bench, agent = (getattr(args, key) if getattr(args, key) is not None else local.get(key)
                              for key in ('robot', 'sim', 'bench', 'agent'))
    model = args.model if args.model is not None else agent_settings.project_model(local, args.agent)
    user = config.load_user_config(paths.config_path(args.home))
    package = config.load_user_config(paths.package_config_path())
    chosen = agent or user.agent or package.agent
    models = {name: facts.model or '' for name, facts in user.agents.items()}
    selected = config.layer_agent({}, package, user, agent=chosen)
    values = {
        'robot': robot or user.robot or package.robot or '',
        'sim': sim or user.simulator or package.simulator or '',
        'bench': (user.benchmark or package.benchmark or '') if bench is None else bench,
        'agent': chosen or '',
        'model': model or models.get(chosen, ''),
        'name': args.name or history.new_id(),
    }
    try:
        resolved = config.compose(robot, sim, bench, args.home, agent=agent)
    except (UsageError, NotFound, config.ConfigError):
        # An incomplete first-run selection stays editable in the menu.
        pass
    else:
        selected = resolved.cfg['agent']
        values.update(robot=resolved.robot if not resolved.robot.startswith('(') else '',
                      sim=resolved.simulator or '',
                      agent=selected['name'], model=model or selected.get('model') or '')
        if resolved.cfg['machine']['backend']['kind'] == 'real' and bench is None:
            # a real robot takes no saved simulation defaults
            values['bench'] = ''
    auth = selected.get('auth') or {}
    values.update(auth_mode=auth.get('mode', 'native'), api_key_file=auth.get('key_file') or '')
    return values, models


def authentication_choices(home: Path, names: list[str]) -> dict:
    """Describe plugin capabilities and saved choices, without reading keys."""
    result = {}
    for name in names:
        adapter, selected = agent_settings.resolve(home, name)
        if 'conversation' not in adapter.capabilities:
            raise UsageError(f'{adapter.name} does not support shared chat',
                             hint='use this plugin in its own terminal (openrua up, then openrua agent) or select a conversation-capable agent')
        auth = selected.get('auth') or {}
        result[name] = {'api': 'prepare_api_profile' in adapter.capabilities,
                        'auth_mode': auth.get('mode', 'native'),
                        'api_key_file': auth.get('key_file') or '', 'model': selected.get('model') or ''}
    return result


def save_values(home: Path, values: dict[str, str]) -> None:
    paths.sandbox_dir(values['name'], home)
    cfg = validate_values(home, values)
    facts = {'model': values['model'] or None}
    if 'auth_mode' in values:
        facts['auth'] = cfg['agent']['auth']
    settings.update(home, {key: values[field] or None for field, key in
                         [('robot', 'robot'), ('sim', 'simulator'), ('bench', 'benchmark'), ('agent', 'agent')]},
                    facts)


def save_guided_values(home: Path, values: dict, environments: list[dict], project: Path | None = None) -> None:
    if not any(all(row['selection'][key] == values[key] for key in startup.FIELDS)
               for row in environments):
        raise UsageError('select a tested robot, simulator and benchmark combination',
                         hint='choose the robot first; custom profiles can use explicit CLI options')
    save_values(home, values)
    if project is not None:
        projects.save_selection(project, values)


def validate_values(home: Path, values: dict[str, str]) -> dict:
    """Validate the exact selection before writing defaults or touching resources."""
    cfg = config.compose(values['robot'], values['sim'], values['bench'], home,
                         agent=values['agent'] or None, model=values['model'] or None).cfg
    selected = cfg['agent']
    adapter = agents.get(selected['name'], home, version=selected.get('version'))
    if 'auth_mode' in values:
        auth = {'mode': values['auth_mode']}
        if values.get('api_key_file'):
            auth['key_file'] = str(Path(values['api_key_file']).expanduser().resolve())
        # A benchmark's explicit facts outrank saved defaults. Do not promise
        # a choice that would be replaced on launch.
        if values['bench']:
            _, benchmark = config.load_benchmark(values['bench'])
            fixed = benchmark.get('agent') or {}
            if fixed.get('name') in (None, selected['name']) and fixed.get('auth') and fixed['auth'] != auth:
                raise UsageError('the benchmark prescribes different authentication',
                                 hint='edit its agent.auth configuration before selecting another source')
        selected['auth'] = auth
    try:
        agents.api_key_file(adapter, selected.get('auth'))
    except ValueError as exc:
        raise UsageError(str(exc), hint='choose native login, or API key file with a readable raw-key file') from exc
    if 'conversation' not in adapter.capabilities:
        raise UsageError(f'{adapter.name} does not support shared chat',
                         hint='use this plugin in its own terminal (openrua up, then openrua agent) or select a conversation-capable agent')
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
    if args.workspace:
        projects.save_selection(Path(args.workspace), values)
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
    for key, value in limit_overrides(args).items():
        flag = key.replace('_', '-')
        argv.append(f'--{flag}' if value is True else f'--no-{flag}' if value is False
                    else f'--{flag}={value}')
    logs = paths.launch_dir(name, args.home)
    return service.start(argv, directory / 'endpoint.json', logs / 'service.log', logs / 'start.lock')


def open_history(home: Path, name: str):
    """Connect to a live owner, or restart a confirmed ended project conversation."""
    client, name = history.open_session(home, name)
    if not getattr(client, 'read_only', False):
        return client, name
    project = history.project_directory(home, name)
    snapshot = client.snapshot()['state']
    if not project or not snapshot['closed']:
        return client, name
    facts = live_state.load(name, home, require_running=False)
    if facts.get('status') != 'stopped' or facts.get('backend') == 'connect':
        return client, name
    if not Path(project).is_dir():
        client.reason = 'Project directory is missing; retained conversation is read-only.'
        return client, name
    settings = snapshot.get('settings') or {}
    facts['model'] = settings.get('model') or facts.get('model')
    selected = workspace.choose(project, home)
    if selected is None:
        raise UsageError('workspace was not trusted; conversation was not restarted')
    if selected != Path(project).resolve():
        raise UsageError('resuming this conversation needs its original project directory',
                         hint='start a new conversation to work in the other directory')
    argv = [sys.executable, '-m', 'openrua', '--home', str(home), 'serve']
    if facts.get('robot'):
        argv.append(facts['robot'])
    for key, flag in [('simulator', '--sim'), ('benchmark', '--bench'), ('agent_spec', '--agent'),
                      ('model', '--model'), ('suite', '--task-suite'), ('task_id', '--task-id'),
                      ('init_state', '--init-state'), ('task', '--task')]:
        value = facts.get(key)
        if value is not None or key in {'simulator', 'benchmark'}:
            argv.extend([flag, str(value) if value is not None else ''])
    argv.extend(['--name', name, '--workspace', str(selected), '--resume-conversation'])
    argv.extend(['--permissions', settings.get('permissions') or facts.get('permissions', 'auto')])
    for key, value in (facts.get('limits') or {}).items():
        flag = key.replace('_', '-')
        argv.append(f'--{flag}' if value is True else f'--no-{flag}' if value is False else f'--{flag}={value}')
    directory, logs = paths.sandbox_dir(name, home), paths.launch_dir(name, home)
    print('Reconnecting this conversation to the robot. Past actions and queued instructions are not replayed.', flush=True)
    return service.start(argv, directory / 'endpoint.json', logs / 'service.log', logs / 'start.lock'), name


def open_terminal(home, client, name: str, frontend: str = 'pi') -> int:
    from openrua.terminal.launcher import chat as terminal_chat
    current = {'project': history.project_directory(home, name) or str(Path.cwd().resolve())}
    def select(name):
        result = open_history(home, name)
        current['project'] = history.project_directory(home, name) or current['project']
        return result
    return terminal_chat(client, name, lambda: history.entries(home, Path(current['project'])), select,
                         facts=lambda name: history.robot_facts(home, name),
                         list_all=lambda: history.entries(home), scope=current['project'])


def choose_history(home, frontend: str = 'pi', *, project: Path | None = None, all_projects=False):
    from openrua.terminal.launcher import choose_history as terminal_history
    return terminal_history(lambda: history.entries(home, project), lambda name: open_history(home, name),
                            list_all=lambda: history.entries(home), scope=str(project) if project else None,
                            all_projects=all_projects)


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


def offer_update(args, values: dict) -> bool:
    """Offer a newer OpenRUA and a newer agent CLI before the session
    starts (see updates); False when the person chose to quit or to run
    the new OpenRUA. Only at an interactive terminal."""
    if not (sys.stdin.isatty() and sys.stdout.isatty()):
        return True
    try:
        composed = config.compose(values['robot'], values['sim'], values['bench'], args.home,
                                  agent=values['agent'] or None, model=values['model'] or None)
    except Exception:  # noqa: BLE001; an incomplete selection is the setup screen's job
        composed = None
    return all(updates.ask(offer, args.home) for offer in updates.offers(composed, args.home))


def _run(args) -> int:
    if args.all and args.resume is None:
        raise UsageError('--all is used with --resume', hint='openrua --resume --all')
    if not (args.gui or args.cli) or args.setup or args.resume == '':
        from openrua.terminal.launcher import check_terminal
        try:
            check_terminal()
        except RuntimeError as exc:
            raise UnavailableError(str(exc)) from exc
    if args.resume is not None:
        if args.name is not None or args.setup or any(getattr(args, key) is not None for key in SELECTION + RESOURCE_OPTIONS if key != 'workspace'):
            raise UsageError('--resume cannot be combined with new-session settings', hint='resume by ID, or start a new session separately')
        if args.resume:
            try:
                recorded = history.project_directory(args.home, args.resume)
                if recorded and not args.all and recorded != str(projects.resolve(args.workspace or Path.cwd())):
                    raise UsageError('conversation belongs to a different project',
                                     hint='change to its directory, or use --resume ID --all explicitly')
                selected = open_history(args.home, args.resume)
            except (OSError, RuntimeError, ValueError) as exc:
                raise UnavailableError(str(exc), hint='use openrua --resume to find retained conversations') from exc
        else:
            if not sys.stdin.isatty() or not sys.stdout.isatty():
                raise UsageError('history selection needs an interactive terminal', hint='use openrua --resume ID to select a conversation explicitly')
            selected = choose_history(args.home, args.tui,
                                      project=projects.resolve(args.workspace or Path.cwd()), all_projects=args.all)
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
    selected = workspace.choose(args.workspace or Path.cwd(), args.home, accept=args.trust_workspace)
    if selected is None:
        return 0
    args.workspace = str(selected)
    interactive = sys.stdin.isatty() and sys.stdout.isatty()
    values, models = values_for(args)
    if args.permissions is not None:
        adapter, _ = agent_settings.resolve(args.home, values['agent'] or None)
        if args.permissions not in adapter.permission_modes:
            raise UsageError(f'{adapter.name} does not support permission mode {args.permissions!r}',
                             hint='choose one of: ' + ', '.join(adapter.permission_modes))
    if not offer_update(args, values):
        return 0
    custom_environment = any(paths.is_path(values[key]) for key in startup.FIELDS if values[key])
    if args.setup and custom_environment:
        raise UsageError('guided setup offers tested bundled environments, not custom profile editing',
                         hint='start without --setup to use your profile; edit it or use openrua config set to change defaults')
    if not custom_environment and (args.setup or not (values['robot'] or values['bench']) or (not args.gui and not args.cli)):
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
                                lambda values: save_guided_values(args.home, values, environments, selected),
                                lambda values: check_values(args.home, values),
                                lambda values: launch(args, values), models,
                                lambda: history.entries(args.home, selected),
                                lambda name: open_history(args.home, name),
                                list_all=lambda: history.entries(args.home), scope=str(selected),
                                environments=environments, prepare=lambda v, progress: prepare(args.home, v, progress, limit_overrides(args)),
                                authentication=authentication_choices(args.home, list(dict.fromkeys(choices['agent'] + [values['agent']]))),
                                resolve_agent=lambda name: authentication_choices(args.home, [name])[name],
                                ready=lambda values: docker_setup.ensure(),
                                save_connect=lambda values: connect.save_values(args.home, values),
                                sign_in=lambda values: login.ensure(args.home, values['agent'] or None,
                                                                    values.get('auth_mode', 'native')),
                                connect=lambda values: connect.launch(args.home, 'my-robot',
                                                                      agent=values['agent'] or None,
                                                                      model=values['model'] or None, project=selected))
        if result is None:
            return 0
        client, name = result
    else:
        validate_values(args.home, values)
        if interactive:
            docker_setup.ensure()
        prepare(args.home, values, limits=limit_overrides(args))
        if interactive:
            login.ensure(args.home, values['agent'] or None, values.get('auth_mode', 'native'))
        ok, report = check_values(args.home, values)
        if not ok:
            raise UnavailableError(report, hint='complete the preparation steps, then start again')
        client, name = launch(args, values), args.name
    return open_interface(args, client, name)
