"""The default frontend shares configuration and never replaces live work."""

from pathlib import Path

import pytest

from openrua.cli import build_parser, main
from openrua.cli.commands import start, connect, config as config_command
from openrua.config import load_user_config, projects
from openrua.errors import UsageError


@pytest.fixture(autouse=True)
def terminal_presentation(monkeypatch):
    from openrua.terminal import application
    class Presentation:
        def __init__(self, *args): pass
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def perform(self, title, operation, **kwargs): return operation()
        def progress(self, line): pass
        def screen(self, spec): pytest.fail('unexpected terminal input in command test')
    monkeypatch.setattr(application, 'Application', Presentation)


@pytest.fixture(autouse=True)
def trusted_project(tmp_path, monkeypatch):
    from openrua.config import projects
    root = tmp_path / 'project'
    root.mkdir()
    monkeypatch.chdir(root)
    projects.trust(root, tmp_path)


@pytest.fixture(autouse=True)
def no_update_offers(monkeypatch):
    """What npm and PyPI hold today must not turn a start into a question
    (tests/cli/test_updates.py covers the offers themselves)."""
    from openrua.cli import updates
    monkeypatch.setattr(updates, "offers", lambda composed, home: [])


@pytest.fixture(autouse=True)
def ready_computer(monkeypatch):
    """Whether this computer has Docker and an agent login must not turn a
    start into a question either (tests/cli/test_first_run.py covers them)."""
    from openrua.cli import docker_setup
    from openrua.cli.commands import login
    monkeypatch.setattr(docker_setup, "ensure", lambda *a, **k: None)
    monkeypatch.setattr(login, "ensure", lambda *a, **k: None)


def args(tmp_path, *flags):
    return build_parser().parse_args(['--home', str(tmp_path), *flags])


@pytest.mark.parametrize('contents', [
    'robot: panda\nsimulator: robosuite\nbenchmark: libero_pro\nagent: codex\n',
    'agent: codex\nagents: [\n',
])
def test_bad_personal_config_keeps_config_exit_code_and_repair_hint(tmp_path, contents, capsys, monkeypatch):
    path = tmp_path / 'config.yaml'
    path.write_text(contents)
    monkeypatch.setattr(start, 'prepare', lambda *a, **kw: pytest.fail('must not prepare resources'))
    monkeypatch.setattr(start, 'launch', lambda *a, **kw: pytest.fail('must not start a session'))
    assert main(['--home', str(tmp_path), '--cli']) == 78
    error = capsys.readouterr().err
    assert str(path) in error
    assert 'config schema --user' in error
    assert 'session may still be running' not in error and '--resume' not in error
    assert path.read_text() == contents


@pytest.mark.parametrize('contents', ['{', '{"unknown": "value"}'])
def test_bad_project_config_is_a_config_error_before_resources_start(tmp_path, contents, capsys, monkeypatch):
    path = projects.settings_path(tmp_path / 'project')
    path.parent.mkdir()
    path.write_text(contents)
    monkeypatch.setattr(start, 'prepare', lambda *a, **kw: pytest.fail('must not prepare resources'))
    assert main(['--home', str(tmp_path), '--cli']) == 78
    error = capsys.readouterr().err
    assert str(path) in error
    assert 'session may still be running' not in error
    assert path.read_text() == contents


def test_missing_robot_keeps_missing_file_exit_code(tmp_path, capsys):
    profile = tmp_path / 'missing-robot.yaml'
    assert main(['--home', str(tmp_path), '--cli', '--robot', str(profile)]) == 66
    assert str(profile) in capsys.readouterr().err


def test_personal_schema_can_be_read_with_a_broken_preferences_file(tmp_path, capsys):
    import json
    path = tmp_path / 'config.yaml'
    path.write_text('agent: [')
    assert main(['--home', str(tmp_path), 'config', 'schema', '--user']) == 0
    schema = json.loads(capsys.readouterr().out)
    assert set(schema['properties']) == {'agent', 'agents', 'sandbox'}
    assert path.read_text() == 'agent: ['


def test_setup_resolves_an_agent_manifest_in_the_selected_project(tmp_path, monkeypatch):
    from openrua.config import paths
    from openrua.terminal import launcher
    project = tmp_path / 'other-project'
    (project / 'adapters').mkdir(parents=True)
    (project / 'adapters/agent.yaml').write_bytes(paths.find('agents', 'codex').read_bytes())
    monkeypatch.setattr(launcher, 'check_terminal', lambda: None)
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    facts = []
    def setup(*args, **callbacks):
        facts.append(callbacks['resolve_agent']('./adapters/agent.yaml'))
    monkeypatch.setattr(launcher, 'setup', setup)
    assert start.run(args(tmp_path, '--setup', '--workspace', str(project), '--trust-workspace')) == 0
    assert facts[0]['api'] and facts[0]['auth_mode'] == 'native'


def test_setup_offers_every_bundled_conversation_agent(tmp_path, monkeypatch):
    from openrua.terminal import launcher
    monkeypatch.setattr(launcher, 'check_terminal', lambda: None)
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    captured = []
    def setup(values, choices, *args, **callbacks):
        captured.append(choices['agent'])
        for name in ('claude-code', 'codex', 'kimi', 'zcode'):
            facts = callbacks['authentication'][name]
            assert facts['title'] and facts['model'] and facts['api']
            assert facts['auth_mode'] == 'native'
    monkeypatch.setattr(launcher, 'setup', setup)
    assert start.run(args(tmp_path, '--setup')) == 0
    assert captured == [['claude-code', 'codex', 'kimi', 'zcode']]
    assert not (tmp_path / 'config.yaml').exists()
    assert not (tmp_path / 'sandboxes').exists()


def test_setup_resolves_an_api_key_file_in_the_selected_project(tmp_path, monkeypatch):
    from openrua.terminal import launcher
    project = tmp_path / 'other-project'
    (project / 'keys').mkdir(parents=True)
    key = project / 'keys/provider.key'
    key.write_text('synthetic-provider-key')
    key.chmod(0o600)
    monkeypatch.setattr(launcher, 'check_terminal', lambda: None)
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    def setup(values, *args, **callbacks):
        chosen = callbacks['resolve_values']({**values, 'robot': 'panda', 'sim': 'robosuite',
            'agent': 'codex', 'auth_mode': 'api', 'api_key_file': 'keys/provider.key'})
        auth = start.validate_values(tmp_path, chosen)['agent']['auth']
        assert auth['key_file'] == str(key)
    monkeypatch.setattr(launcher, 'setup', setup)
    assert start.run(args(tmp_path, '--setup', '--workspace', str(project), '--trust-workspace')) == 0


def test_cli_and_setup_edit_the_same_project(tmp_path):
    config_command.run(args(tmp_path, 'config', 'set', '--robot', 'panda', '--sim', 'robosuite', '--agent', 'codex', '--model', 'm1'))
    values, models = start.values_for(args(tmp_path))
    assert Path(values['robot']).stem == 'panda' and values['model'] == 'm1'
    values.update(bench='capbench', model='m2')
    start.save_guided_values(tmp_path, values, tmp_path / 'project')
    config_command.run(args(tmp_path, 'config', 'set', '--bench', 'null'))
    saved = projects.selection(tmp_path / 'project')
    assert saved['bench'] == '' and saved['model'] == 'm2'
    assert 'robot' not in (tmp_path / 'config.yaml').read_text()


@pytest.mark.parametrize('agent', ['claude-code', 'codex'])
def test_setup_authentication_roundtrip_and_agent_isolation(tmp_path, agent):
    key = tmp_path / 'provider.key'
    key.write_text('synthetic-api-key')
    values, _ = start.values_for(args(tmp_path, '--agent', agent, '--bench', 'capbench'))
    assert values['auth_mode'] == 'native' and values['api_key_file'] == ''
    values.update(auth_mode='api', api_key_file=str(key))
    connect.save_values(tmp_path, values)
    reopened, _ = start.values_for(args(tmp_path, '--agent', agent))
    assert reopened['auth_mode'] == 'api' and reopened['api_key_file'] == str(key)
    choices = start.authentication_choices(tmp_path, ['claude-code', 'codex'])
    assert choices[agent]['api'] and choices[agent]['api_key_file'] == str(key)
    other = 'codex' if agent == 'claude-code' else 'claude-code'
    assert choices[other]['auth_mode'] == 'native' and choices[other]['api_key_file'] == ''
    assert 'synthetic-api-key' not in (tmp_path / 'config.yaml').read_text()
    reopened.update(auth_mode='native', api_key_file='')
    connect.save_values(tmp_path, reopened)
    assert load_user_config(tmp_path / 'config.yaml').agents[agent].auth.key_file is None
    assert key.read_text() == 'synthetic-api-key'


def test_invalid_api_selection_cannot_write_defaults_or_launch(tmp_path, monkeypatch):
    from openrua.terminal import launcher
    values, _ = start.values_for(args(tmp_path, '--bench', 'capbench'))
    connect.save_values(tmp_path, values)
    path = tmp_path / 'config.yaml'
    original = path.read_text()
    values.update(auth_mode='api', api_key_file=str(tmp_path / 'missing.key'))
    responses = iter([{'action': 'start', 'values': values}, {'action': 'quit'}])
    screens = []
    def screen(spec):
        screens.append(spec)
        return next(responses)
    launcher.setup(values, {}, str(path), lambda v: connect.save_values(tmp_path, v),
        lambda v: pytest.fail('must not check resources'), lambda v: pytest.fail('must not launch'),
        {}, lambda: [], lambda n: None, screen,
        prepare=lambda *a, **k: pytest.fail('must not prepare resources'))
    assert 'key' in screens[1]['notice']
    assert path.read_text() == original
    assert not (tmp_path / 'sandboxes').exists()
    values.update(auth_mode='native')
    with pytest.raises(UsageError, match='requires mode: api'):
        connect.save_values(tmp_path, values)


def test_no_arguments_on_a_pipe_prints_help_without_launching(monkeypatch, capsys):
    monkeypatch.setattr(start, 'run', lambda args: pytest.fail('must not launch'))
    assert main([]) == 0
    assert '--setup' in capsys.readouterr().out


def test_interactive_default_uses_product_entry(monkeypatch, tmp_path):
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    called = []
    monkeypatch.setattr(start, 'run', lambda args: called.append(args) or 0)
    assert main(['--home', str(tmp_path)]) == 0
    assert called[0].home == tmp_path
    assert called[0].tui == 'pi'


def test_existing_session_reconnects_without_reading_changed_defaults(tmp_path, monkeypatch):
    directory = tmp_path / 'sandboxes/openrua'
    directory.mkdir(parents=True)
    (directory / 'endpoint.json').write_text('{}')
    (tmp_path / 'config.yaml').write_text('invalid: [')
    client = object()
    monkeypatch.setattr(start.service, 'connect', lambda path: client)
    opened = []
    monkeypatch.setattr(start, 'open_interface', lambda args, c, name, **kw: opened.append((c, name)) or 0)
    assert start.run(args(tmp_path, '--cli', '--name', 'openrua')) == 0
    assert opened == [(client, 'openrua')]
    with pytest.raises(UsageError, match='startup options'):
        start.run(args(tmp_path, '--cli', '--name', 'openrua', '--agent', 'codex'))


def test_retained_directory_is_not_overwritten(tmp_path, monkeypatch):
    (tmp_path / 'sandboxes/used').mkdir(parents=True)
    monkeypatch.setattr(start.service, 'start', lambda *a: pytest.fail('must not start'))
    values, _ = start.values_for(args(tmp_path, '--name', 'used'))
    with pytest.raises(UsageError, match='retained'):
        start.launch(args(tmp_path, '--name', 'used'), values)


def test_launch_passes_explicit_options_without_a_shell(tmp_path, monkeypatch):
    selected = args(tmp_path, '--robot', 'panda', '--sim', 'robosuite', '--bench', 'capbench',
                    '--agent', 'codex', '--task-suite', 'capbench_lift', '--task-id', '0')
    values, _ = start.values_for(selected)
    captured = []
    monkeypatch.setattr(start.service, 'start', lambda *a: captured.append(a) or 'connected')
    assert start.launch(selected, values) == 'connected'
    argv, endpoint, log, lock = captured[0]
    assert argv[1:6] == ['-m', 'openrua', '--home', str(tmp_path), 'serve']
    child = build_parser().parse_args(argv[3:])
    assert (child.agent, child.task_suite, child.task_id) == ('codex', 'capbench_lift', 0)
    assert endpoint.parent == tmp_path / 'sandboxes' / values['name']
    assert log.parent == lock.parent == tmp_path / 'launches' / values['name']


def test_setup_preview_preserves_benchmark_configuration_precedence(tmp_path):
    config_command.run(args(tmp_path, 'config', 'set', '--bench', 'robocasa365', '--agent', 'codex'))
    values, _ = start.values_for(args(tmp_path))
    assert Path(values['robot']).stem == 'panda-omron'
    assert Path(values['bench']).stem == 'robocasa365'
    explicit, _ = start.values_for(args(tmp_path, '--agent', 'codex', '--model', 'chosen'))
    assert explicit['agent'] == 'codex' and explicit['model'] == 'chosen'


def test_unnamed_starts_create_distinct_ids_without_reconnecting_old_default(tmp_path, monkeypatch):
    (tmp_path / 'sandboxes/openrua').mkdir(parents=True)
    (tmp_path / 'sandboxes/openrua/endpoint.json').write_text('{}')
    monkeypatch.setattr(start.service, 'connect', lambda path: pytest.fail('must not attach implicitly'))
    monkeypatch.setattr(start, 'check_values', lambda *a: (True, 'Ready'))
    monkeypatch.setattr(start, 'prepare', lambda *a, **k: None)
    launched = []
    monkeypatch.setattr(start, 'launch', lambda a, v: launched.append(v['name']) or object())
    monkeypatch.setattr(start, 'open_interface', lambda *a, **kw: 0)
    for _ in range(2):
        start.run(args(tmp_path, '--cli', '--robot', 'panda', '--sim', 'robosuite'))
    assert len(set(launched)) == 2 and all(len(name) == 32 for name in launched)


def test_resume_does_not_load_config_or_launch_resources(tmp_path, monkeypatch):
    (tmp_path / 'config.yaml').write_text('broken: [')
    selected = object()
    monkeypatch.setattr(start.history, 'open_session', lambda home, name: (selected, name))
    opened = []
    monkeypatch.setattr(start, 'open_interface', lambda a, c, n, **kw: opened.append((c, n)) or 0)
    assert start.run(args(tmp_path, '--cli', '--resume', 'old')) == 0
    assert opened == [(selected, 'old')]
    with pytest.raises(UsageError):
        start.run(args(tmp_path, '--cli', '--resume', 'old', '--robot', 'panda'))


def test_pi_uses_same_launcher_after_keyboard_setup(tmp_path, monkeypatch):
    from openrua.terminal import launcher
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    monkeypatch.setattr(launcher, 'runtime', lambda: None)
    monkeypatch.setattr(launcher, 'entrypoint', lambda: None)
    phases = []
    monkeypatch.setattr(start, 'save_guided_values', lambda home, v, *a: phases.append('save'))
    monkeypatch.setattr(start, 'check_values', lambda home, v: (phases.append('check') or True, 'Ready'))
    client = object()
    monkeypatch.setattr(start, 'launch', lambda a, v: phases.append('launch') or client)
    def setup(values, choices, location, save, check, launch, *callbacks, **kwargs):
        save(values)
        assert check(values)[0]
        return launch(values), values['name']
    monkeypatch.setattr(launcher, 'setup', setup)
    monkeypatch.setattr(start, 'open_interface', lambda a, c, name, **kw: phases.append('chat') or 0)
    assert start.run(args(tmp_path, '--setup', '--bench', 'robocasa365')) == 0
    assert phases == ['save', 'check', 'launch', 'chat']


def test_pi_resume_never_loads_new_settings_or_launches(tmp_path, monkeypatch):
    from openrua.terminal import launcher
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    monkeypatch.setattr(launcher, 'runtime', lambda: None)
    monkeypatch.setattr(launcher, 'entrypoint', lambda: None)
    monkeypatch.setattr(start, 'values_for', lambda a: pytest.fail('must not configure'))
    monkeypatch.setattr(start, 'launch', lambda *a: pytest.fail('must not launch'))
    monkeypatch.setattr(start.history, 'open_session', lambda home, name: ('client', name))
    opened = []
    monkeypatch.setattr(launcher, 'chat', lambda client, name, *ops, **kw: opened.append((client, name)) or 0)
    assert start.run(args(tmp_path, '--resume', 'retained')) == 0
    assert opened == [('client', 'retained')]


def test_bad_selection_never_overwrites_existing_settings(tmp_path):
    values, _ = start.values_for(args(tmp_path, '--robot', 'panda', '--sim', 'robosuite'))
    start.save_guided_values(tmp_path, values, tmp_path / 'project')
    path = projects.settings_path(tmp_path / 'project')
    original = path.read_bytes()
    values.update(sim='maniskill', bench='libero_pro')
    with pytest.raises(Exception):
        start.save_guided_values(tmp_path, values, tmp_path / 'project')
    assert path.read_bytes() == original
    assert not (tmp_path / 'sandboxes').exists()


def test_launch_preserves_explicit_cleared_environment(tmp_path, monkeypatch):
    selected = args(tmp_path, '--cli', '--robot', 'panda', '--sim', 'robosuite')
    values = dict(robot='panda', sim='robosuite', bench='', agent='claude-code', model='', name='new')
    calls = []
    monkeypatch.setattr(start.service, 'start', lambda *a: calls.append(a))
    start.launch(selected, values)
    child = build_parser().parse_args(calls[0][0][3:])
    assert child.bench == ''


def test_invalid_saved_combination_still_opens_editable_setup(tmp_path):
    path = projects.settings_path(tmp_path / 'project'); path.parent.mkdir()
    path.write_text('{"robot":"panda","sim":"maniskill","bench":"libero_pro"}')
    values, _ = start.values_for(args(tmp_path))
    assert values['robot'] == 'panda' and values['sim'] == 'maniskill'


@pytest.mark.parametrize('flag', ['--gui', '--cli'])
def test_non_tui_resume_needs_no_terminal_runtime(tmp_path, monkeypatch, flag):
    from openrua.terminal import launcher
    monkeypatch.setattr(launcher, 'check_terminal', lambda: pytest.fail('must not open terminal'))
    monkeypatch.setattr(start.history, 'open_session', lambda home, name: ('client', name))
    monkeypatch.setattr(start, 'open_interface', lambda *args, **kw: 0)
    assert start.run(args(tmp_path, flag, '--resume', 'running')) == 0


def test_legacy_chat_tui_opens_the_same_keyboard_frontend(tmp_path, monkeypatch):
    from openrua.cli.commands import chat
    from openrua.terminal import launcher
    monkeypatch.setattr(launcher, 'check_terminal', lambda: None)
    monkeypatch.setattr(chat, 'connect', lambda args: type('Client', (), {})())
    opened = []
    monkeypatch.setattr(launcher, 'chat', lambda client, name, *ops, **kw: opened.append(name) or 0)
    assert main(['--home', str(tmp_path), 'chat', '--tui', '--name', 'old']) == 0
    assert opened == ['old']


@pytest.mark.parametrize('saved', [False, True])
def test_custom_benchmark_selector_survives_preview_and_launch(tmp_path, monkeypatch, saved):
    from openrua.config import paths
    profile = tmp_path / 'project/custom-benchmark.yaml'
    profile.write_text(paths.find('benchmarks', 'capbench').read_text())
    if saved:
        config_command.run(args(tmp_path, 'config', 'set', '--bench', str(profile)))
    selected = args(tmp_path, '--cli', *([] if saved else ['--bench', str(profile)]))
    values, _ = start.values_for(selected)
    assert values['bench'] == str(profile)
    captured = []
    monkeypatch.setattr(start.service, 'start', lambda *a: captured.append(a))
    start.launch(selected, values)
    child = build_parser().parse_args(captured[0][0][3:])
    assert child.bench == str(profile)


@pytest.mark.parametrize('kind', ['robot', 'bench'])
@pytest.mark.parametrize('saved', [False, True])
def test_custom_environment_opens_tui_without_guided_replacement(tmp_path, monkeypatch, kind, saved):
    from openrua.config import paths
    from openrua.terminal import launcher
    profile = tmp_path / 'project' / (kind + '.yaml')
    profile.write_text('type: panda\nmachine:\n  backend: {kind: real, discovery: {network: host}}\n  cameras: {list: []}\n'
                       '  gripper: {holds_partway: true, effort_honoured: true}\n'
                       if kind == 'robot' else paths.find('benchmarks', 'capbench').read_text())
    if saved:
        config_command.run(args(tmp_path, 'config', 'set', '--' + kind, str(profile)))
    monkeypatch.setattr(launcher, 'check_terminal', lambda: None)
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    monkeypatch.setattr(launcher, 'setup', lambda *a, **kw: pytest.fail('must not replace a custom environment'))
    original = projects.settings_path(tmp_path / 'project').read_bytes() if saved else None
    phases = []
    monkeypatch.setattr(start, 'prepare', lambda h, v, **k: phases.append(('prepare', v[kind])))
    monkeypatch.setattr(start, 'check_values', lambda h, v: (phases.append(('check', v[kind])) or True, 'Ready'))
    monkeypatch.setattr(start, 'launch', lambda a, v: phases.append(('launch', v[kind])) or 'client')
    monkeypatch.setattr(start, 'open_interface', lambda a, c, n, **kw: phases.append(('chat', c)) or 0)
    assert start.run(args(tmp_path, *([] if saved else ['--' + kind, str(profile)]))) == 0
    assert phases == [('prepare', str(profile)), ('check', str(profile)), ('launch', str(profile)), ('chat', 'client')]
    assert (projects.settings_path(tmp_path / 'project').read_bytes() if projects.settings_path(tmp_path / 'project').exists() else None) == original


def test_invalid_custom_environment_fails_before_building_or_starting(tmp_path, monkeypatch):
    from openrua.terminal import launcher
    profile = tmp_path / 'broken.yaml'
    profile.write_text('type: nonexistent-robot\nmachine: {}\n')
    monkeypatch.setattr(launcher, 'check_terminal', lambda: None)
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    monkeypatch.setattr(launcher, 'setup', lambda *a, **kw: pytest.fail('must not replace explicit input'))
    monkeypatch.setattr(start, 'prepare', lambda *a, **k: pytest.fail('must not build'))
    monkeypatch.setattr(start, 'launch', lambda *a: pytest.fail('must not launch'))
    from openrua.errors import OpenRUAError
    with pytest.raises(OpenRUAError):
        start.run(args(tmp_path, '--robot', str(profile)))


def test_explicit_empty_benchmark_does_not_reintroduce_saved_default(tmp_path):
    config_command.run(args(tmp_path, 'config', 'set', '--bench', 'capbench'))
    values, _ = start.values_for(args(tmp_path, '--robot', 'panda', '--sim', 'robosuite', '--bench', ''))
    assert values['bench'] == ''


def test_custom_project_is_available_in_setup(tmp_path, monkeypatch):
    from openrua.terminal import launcher
    profile = tmp_path / 'project/robot.yaml'
    profile.write_text('type: panda\nmachine:\n  backend: {kind: real, discovery: {network: host}}\n  cameras: {list: []}\n'
                       '  gripper: {holds_partway: true, effort_honoured: true}\n')
    monkeypatch.setattr(launcher, 'check_terminal', lambda: None)
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    seen = []
    monkeypatch.setattr(launcher, 'setup', lambda *a, **kw: seen.append(kw['environments']))
    assert start.run(args(tmp_path, '--setup', '--robot', str(profile))) == 0
    assert any(row['selection']['robot'] == str(profile) for row in seen[0])


def test_connect_setup_passes_new_api_choice_to_its_service(tmp_path, monkeypatch):
    from openrua.terminal import launcher
    from openrua.cli.commands import connect
    from openrua.runner.connect import ConnectRequest, _agent
    key = tmp_path / 'provider.key'
    key.write_text('synthetic-provider-key')
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    monkeypatch.setattr(launcher, 'check_terminal', lambda: None)
    monkeypatch.setattr(connect.preparation, 'prepare_connect', lambda *a: None)
    monkeypatch.setattr(start, 'open_interface', lambda *a, **kw: 0)
    observed = []

    def service(argv, *a):
        _, selected, _, _ = _agent(ConnectRequest(tmp_path, 'test', tmp_path / 'project', tmp_path / 'project', agent='codex'))
        observed.append(selected['auth'])
        return 'client'

    monkeypatch.setattr(connect.service, 'start', service)

    def setup(values, *a, **callbacks):
        values.update(agent='codex', auth_mode='api', api_key_file=str(key))
        if callbacks.get('save_connect'):
            callbacks['save_connect'](values)
        return callbacks['connect'](values)

    monkeypatch.setattr(launcher, 'setup', setup)
    assert start.run(args(tmp_path)) == 0
    assert observed == [{'mode': 'api', 'key_file': str(key)}]


@pytest.mark.parametrize('plugin', ['kimi', 'zcode'])
def test_entered_external_plugin_can_explicitly_select_api_in_setup(tmp_path, plugin):
    from pathlib import Path
    from openrua.terminal import launcher
    source = Path(__file__).resolve().parents[2] / 'openrua/configs/agents' / (plugin + '.yaml')
    manifest_file = tmp_path / 'project' / (plugin + '.yaml')
    manifest_file.write_bytes(source.read_bytes())
    manifest = str(manifest_file)
    key = tmp_path / 'provider.key'
    key.write_text('synthetic-key-no-model-calls')
    values, models = start.values_for(args(tmp_path, '--bench', 'capbench'))
    screens = []
    def screen(spec):
        screens.append(spec)
        if len(screens) == 1:
            return {'action': 'agent', 'agent': manifest, 'values': dict(spec['values'])}
        if len(screens) == 2:
            assert spec['authentication'][manifest]['api']
            assert spec['values']['agent'] == manifest
            assert spec['values']['auth_mode'] == 'native'
            assert spec['values']['api_key_file'] == ''
            assert not (tmp_path / 'config.yaml').exists()
            return {'action': 'check', 'values': {**spec['values'], 'auth_mode': 'api', 'api_key_file': str(key)}}
        return {'action': 'quit'}
    launcher.setup(values, {}, 'config', lambda v: connect.save_values(tmp_path, v),
        lambda v: (False, 'Images not prepared in this test'), lambda v: pytest.fail('must not launch'),
        models, None, None, screen,
        resolve_agent=lambda name: start.authentication_choices(tmp_path, [name])[name])
    saved = load_user_config(tmp_path / 'config.yaml')
    assert saved.agent is None
    assert saved.agents[manifest].auth.mode == 'api'
    assert saved.agents[manifest].auth.key_file == str(key)
    assert key.read_text() not in (tmp_path / 'config.yaml').read_text()


def test_chat_setup_uses_fresh_launch_options_and_returns_the_new_session(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from openrua.terminal import launcher
    project = Path.cwd()
    old = SimpleNamespace(url='http://127.0.0.1:1', token='old')
    new = SimpleNamespace(url='http://127.0.0.1:2', token='new')
    monkeypatch.setattr(start.history, 'project_directory', lambda home, name: str(project))
    monkeypatch.setattr(start.history, 'new_id', lambda: 'fresh-session')
    seen = []
    def configure(args, selected, application, **kwargs):
        seen.append((args, selected, application, kwargs))
        return new, args.name
    monkeypatch.setattr(start, 'setup_project', configure)
    def chat(client, name, *args, **kwargs):
        assert (client, name) == (old, 'old-session')
        assert kwargs['setup_session']('old-session') == (new, 'fresh-session')
        return 0
    monkeypatch.setattr(launcher, 'chat', chat)
    host = SimpleNamespace(screen=lambda _: pytest.fail('trusted project needs no prompt'))
    assert start.open_terminal(tmp_path, old, 'old-session', application=host) == 0
    request, selected, application, context = seen[0]
    assert selected == project and application is host
    assert request.workspace == str(project) and request.setup
    assert request.name == 'fresh-session'
    assert request.permissions is None and request.task_suite is None and request.task_id is None
    assert request.resume is None
    assert context == {'previous_session': 'old-session'}
