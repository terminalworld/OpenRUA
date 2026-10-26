"""Open the product interface using the existing shared session service."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from openrua import __version__, agents, config, doctor
from openrua.cli.commands import connect, login, session, up
from openrua.config import paths, projects, settings, startup
from openrua.cli import agent_settings, docker_setup, updates, workspace
from openrua.cli.preparation import prepare
from openrua.errors import NotFound, OpenRUAError, UnavailableError, UsageError
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
    group.add_argument('--port', type=int, default=None, help='local service port for a new session (default: a free port)')
    group.add_argument('--setup', action='store_true', help='edit this project in the keyboard setup menu')
    group.add_argument('--resume', nargs='?', const='', default=None, metavar='ID',
                       help='resume a conversation in this workspace, optionally by ID/name')
    group.add_argument('--all', action='store_true',
                       help='with --resume, show conversations from all project directories')
    parser.set_defaults(init_state=None, name=None)


def values_for(args) -> tuple[dict[str, str], dict[str, str]]:
    local = projects.options(Path(args.workspace or Path.cwd()),
                             {key: getattr(args, key, None) for key in projects.FIELDS},
                             relative_to=Path.cwd())
    robot, sim, bench, agent = (local.get(key) for key in ('robot', 'sim', 'bench', 'agent'))
    model = local.get('model')
    user = config.load_user_config(paths.config_path(args.home))
    package = config.load_user_config(paths.package_config_path())
    chosen = agent or user.agent or package.agent
    models = {name: facts.model or '' for name, facts in user.agents.items()}
    selected = config.layer_agent({}, package, user, agent=chosen)
    values = {
        'robot': robot or '',
        'sim': sim or '',
        'bench': bench or '',
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


def authentication_choices(home: Path, names: list[str], *, project: Path | None = None) -> dict:
    """Describe plugin capabilities and saved choices, without reading keys."""
    result = {}
    for name in names:
        spec = projects.options(project, {'agent': name}, relative_to=project)['agent'] if project else name
        adapter, selected = agent_settings.resolve(home, spec)
        if 'conversation' not in adapter.capabilities:
            raise UsageError(f'{adapter.name} does not support shared chat',
                             hint='use this plugin in its own terminal (openrua up, then openrua agent) or select a conversation-capable agent')
        auth = selected.get('auth') or {}
        result[name] = {'api': 'prepare_api_profile' in adapter.capabilities,
                        'auth_mode': auth.get('mode', 'native'),
                        'api_key_file': auth.get('key_file') or '', 'model': selected.get('model') or adapter.default_model,
                        'title': adapter.title or adapter.name, 'version': adapter.version,
                        'model_selection': adapter.model_selection,
                        'model_discovery': 'model_catalog' in adapter.capabilities,
                        'native_hint': adapter.login_hint(paths.credentials_dir(home) / adapter.name)}
    return result


def discover_models(home, values, intent, application, interact):
    from openrua.cli import model_catalog
    adapter, _ = agent_settings.resolve(home, values['agent'] or None)
    if 'model_catalog' not in adapter.capabilities:
        raise UnavailableError('This agent does not provide a model list.',
                               hint='use its configured model or enter a supported model ID when available')
    docker_setup.ensure(**({'interact': interact('Docker setup')} if application else {}))
    progress = application.progress if application else print
    if intent == 'connect':
        from openrua.cli.preparation import prepare_connect
        prepare_connect(home, values['agent'] or None, progress=progress)
        from openrua.config.schema import sandbox_image
        image = sandbox_image('jazzy')
    else:
        prepare(home, values, progress=progress, environment_only=True)
        image = validate_values(home, values)['machine']['backend']['sandbox_image']
    if values.get('auth_mode', 'native') == 'native':
        login.ensure(home, values['agent'] or None, 'native',
                     **({'interact': interact('Agent sign-in')} if application else {}))
    return model_catalog.choices(home, values, image)


def setup_values(project: Path, incoming: dict) -> dict:
    values = {**incoming, **projects.options(project,
        {key: incoming[key] for key in projects.FIELDS}, relative_to=project)}
    if values.get('api_key_file'):
        values['api_key_file'] = str((project / Path(values['api_key_file']).expanduser()).resolve())
    return values


def save_guided_values(home: Path, values: dict, project: Path) -> None:
    validate_values(home, values)
    projects.save_selection(project, values)
    if 'auth_mode' in values and values['agent']:
        cfg = validate_values(home, values)
        settings.update(home, {}, {'auth': cfg['agent']['auth']}, agent=values['agent'])


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
            same_agent = fixed.get('name') is None or paths.agent_key(fixed['name']) == paths.agent_key(selected['name'])
            if same_agent and fixed.get('auth') and fixed['auth'] != auth:
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
                        home=home, environment_only=True)
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
    for key, value in limit_overrides(args).items():
        flag = key.replace('_', '-')
        argv.append(f'--{flag}' if value is True else f'--no-{flag}' if value is False
                    else f'--{flag}={value}')
    logs = paths.launch_dir(name, args.home)
    return service.start(argv, directory / 'endpoint.json', logs / 'service.log', logs / 'start.lock')


def open_history(home: Path, name: str, *, screen=None):
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
        client.reason = 'The original project directory is missing. You can read the saved messages, but you cannot continue this conversation.'
        return client, name
    settings = snapshot.get('settings') or {}
    facts['model'] = settings.get('model') or facts.get('model')
    selected = workspace.choose(project, home, **({'screen': screen} if screen else {}))
    if selected is None:
        raise UsageError('This directory was not trusted, so the conversation was not restarted.')
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
    adapter = agents.get(facts.get('agent_spec') or facts['agent'], home, version=facts.get('agent_version'))
    permissions = agents.saved_permissions(adapter, facts, snapshot)
    if permissions is not None:
        argv.extend(['--permissions', permissions])
    for key, value in (facts.get('limits') or {}).items():
        flag = key.replace('_', '-')
        argv.append(f'--{flag}' if value is True else f'--no-{flag}' if value is False else f'--{flag}={value}')
    directory, logs = paths.sandbox_dir(name, home), paths.launch_dir(name, home)
    print('Restarting the robot for this conversation. Previous actions and waiting instructions will not run again.', flush=True)
    return service.start(argv, directory / 'endpoint.json', logs / 'service.log', logs / 'start.lock'), name


def reconnect(home, name, application=None):
    if application:
        return application.perform('Opening conversation',
                                   lambda: open_history(home, name, screen=application.screen))
    return open_history(home, name)


def setup_project(args, selected: Path, application=None, *, values=None, models=None, previous_session=None, before_launch=None):
    """Configure and start a fresh session using the project's existing setup flow."""
    if values is None:
        values, models = values_for(args)
    def interact(title):
        return (lambda operation: application.perform(title, operation, native=True)) if application else None
    def open_previous(name):
        if before_launch is not None and name != previous_session and not before_launch():
            return None
        return reconnect(args.home, name, application)
    environments = startup.verified_environments()
    if any(paths.is_path(values[key]) for key in startup.FIELDS if values[key]):
        environments = [dict(selection={key: values[key] for key in startup.FIELDS}), *environments]
    if values['robot'] in startup.options(environments, {}, 'robot'):
        values = startup.select(environments, values, 'robot', values['robot'])
    choices = {'agent': [entry.name for entry in paths.available('agents')
                         if 'conversation' in agents.get(entry.name, args.home).capabilities]}
    choices.update({field: startup.options(environments, values, field) for field in startup.FIELDS})
    from openrua.terminal.launcher import setup as terminal_setup
    result = terminal_setup(values, choices, str(projects.settings_path(selected)),
                            lambda values: save_guided_values(args.home, values, selected),
                            lambda values: check_values(args.home, values),
                            lambda values: launch(args, {**values, **projects.selection(selected)}), models,
                            lambda: history.entries(args.home, selected),
                            open_previous,
                            list_all=lambda: history.entries(args.home), scope=str(selected),
                            **({'screen': application.screen, 'application': application} if application else {}),
                            environments=environments, prepare=lambda v, progress: prepare(args.home, v, progress, limit_overrides(args), environment_only=True),
                            authentication=authentication_choices(args.home, list(dict.fromkeys(choices['agent'] + [values['agent']])), project=selected),
                            resolve_agent=lambda name: authentication_choices(args.home, [name], project=selected)[name],
                            ready=lambda values: docker_setup.ensure(**({'interact': interact('Docker setup')} if application else {})),
                            save_connect=lambda values: connect.save_values(args.home, values),
                            sign_in=lambda values: login.ensure(args.home, values['agent'] or None,
                                                                values.get('auth_mode', 'native'), **({'interact': interact('Agent sign-in')} if application else {})),
                            resolve_values=lambda incoming: setup_values(selected, incoming),
                            resolve_models=lambda incoming, intent: discover_models(args.home, incoming, intent,
                                application, interact),
                            previous_session=previous_session,
                            before_launch=before_launch,
                            connect=lambda values: connect.launch(args.home,
                                                                  agent=values['agent'] or None,
                                                                  model=values['model'] or None, project=selected))
    return result


def open_terminal(home, client, name: str, frontend: str = 'pi', application=None) -> int:
    if application is None:
        from openrua.terminal.application import Application
        from openrua.terminal.launcher import runtime, entrypoint
        with Application(runtime, entrypoint) as host:
            return open_terminal(home, client, name, frontend, host)
    from openrua.terminal.launcher import chat as terminal_chat
    current = {'project': history.project_directory(home, name) or str(Path.cwd().resolve())}
    def select(name):
        result = reconnect(home, name, application)
        current['project'] = history.project_directory(home, name) or current['project']
        return result
    def configure(name, before_launch):
        project = history.project_directory(home, name) or current['project']
        if not Path(project).is_dir():
            raise UsageError('this project directory is missing', hint='open an existing project with openrua --setup')
        selected = workspace.choose(project, home, **({'screen': application.screen} if application else {}))
        if selected is None:
            return None
        from openrua.cli import build_parser
        args = build_parser().parse_args(['--home', str(home), '--workspace', str(selected), '--setup'])
        args.name = history.new_id()
        result = setup_project(args, selected, application, previous_session=name, before_launch=before_launch)
        if result:
            current['project'] = history.project_directory(home, result[1]) or str(selected)
        return result
    return terminal_chat(client, name, lambda: history.entries(home, Path(current['project'])), select,
                         setup_session=configure, version=__version__, application=application,
                         retained_session=lambda name: history.open_session(home, name),
                         facts=lambda name: history.robot_facts(home, name),
                         list_all=lambda: history.entries(home), scope=current['project'],
                         **({'screen': application.screen} if application else {}))


def choose_history(home, frontend: str = 'pi', *, project: Path | None = None, all_projects=False, application=None):
    from openrua.terminal.launcher import choose_history as terminal_history
    return terminal_history(lambda: history.entries(home, project), lambda name: reconnect(home, name, application),
                            list_all=lambda: history.entries(home), scope=str(project) if project else None,
                            all_projects=all_projects, **({'screen': application.screen} if application else {}))


def open_interface(args, client, name: str, application=None) -> int:
    if application and args.gui:
        return application.perform('Opening interface', lambda: open_interface(args, client, name), native=True)
    if args.gui:
        if getattr(client, 'read_only', False):
            raise UsageError('This saved conversation is available for viewing only in the terminal. You cannot send messages from this view.',
                             hint=f'openrua --resume {name} opens its saved messages')
        return session.run(argparse.Namespace(home=args.home, name=name, action='web', no_open=False))
    return open_terminal(args.home, client, name, args.tui, **({'application': application} if application else {}))


def run(args) -> int:
    try:
        if args.gui and not args.setup and args.resume != '':
            return _run(args)
        from openrua.terminal.application import Application
        from openrua.terminal.launcher import runtime, entrypoint
        with Application(runtime, entrypoint) as application:
            return _run(args, application)
    except OpenRUAError:
        raise
    except (RuntimeError, OSError, ValueError) as exc:
        raise UnavailableError(str(exc)) from exc


def offer_update(args, values: dict, application=None) -> bool:
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
    for offer in updates.offers(composed, args.home):
        if not (updates.present(offer, args.home, application) if application else updates.ask(offer, args.home)):
            return False
    return True


def _run(args, application=None) -> int:
    from openrua.cli.skill_scheduler import autostart
    autostart(args.home)
    if args.all and args.resume is None:
        raise UsageError('--all is used with --resume', hint='openrua --resume --all')
    if not args.gui or args.setup or args.resume == '':
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
                selected = reconnect(args.home, args.resume, application)
            except (OSError, RuntimeError, ValueError) as exc:
                raise UnavailableError(str(exc), hint='use openrua --resume to find a saved conversation') from exc
        else:
            if not sys.stdin.isatty() or not sys.stdout.isatty():
                raise UsageError('history selection needs an interactive terminal', hint='use openrua --resume ID to select a conversation explicitly')
            selected = choose_history(args.home, args.tui,
                                      project=projects.resolve(args.workspace or Path.cwd()), all_projects=args.all, application=application)
        return open_interface(args, *selected, application=application) if selected else 0
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
        return open_interface(args, client, args.name, application=application)
    selected = workspace.choose(args.workspace or Path.cwd(), args.home, accept=args.trust_workspace,
                                **({'screen': application.screen} if application else {}))
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
    def interact(title):
        return (lambda operation: application.perform(title, operation, native=True)) if application else None
    updated = application.perform('Checking updates', lambda: offer_update(args, values, application)) if application else offer_update(args, values)
    if not updated:
        return 0
    if args.setup or not (values['robot'] or values['bench']):
        if not sys.stdin.isatty() or not sys.stdout.isatty():
            raise UsageError('terminal setup needs an interactive terminal',
                             hint='use an interactive terminal for setup, or configure the project with openrua config set and use --gui')
        result = setup_project(args, selected, application, values=values, models=models)
        if result is None:
            return 0
        client, name = result
    else:
        validate_values(args.home, values)
        def perform(title, operation, native=False):
            return application.perform(title, operation, native=native) if application else operation()
        if interactive:
            perform('Docker setup', lambda: docker_setup.ensure(**({'interact': interact('Docker setup')} if application else {})))
        perform('Preparing environment', lambda: prepare(args.home, values,
                progress=application.progress if application else print, limits=limit_overrides(args), environment_only=True))
        if interactive:
            perform('Agent sign-in', lambda: login.ensure(args.home, values['agent'] or None, values.get('auth_mode', 'native'),
                    **({'interact': interact('Agent sign-in')} if application else {})))
        ok, report = perform('Checking preparation', lambda: check_values(args.home, values))
        if not ok:
            raise UnavailableError(report, hint='complete the preparation steps, then start again')
        client, name = perform('Starting session', lambda: launch(args, values)), args.name
    return open_interface(args, client, name, application=application)
