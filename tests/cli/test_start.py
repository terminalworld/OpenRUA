"""The default frontend shares configuration and never replaces live work."""

import pytest

from openrua.cli import build_parser, main
from openrua.cli.commands import start, config as config_command
from openrua.config import load_user_config
from openrua.errors import UsageError


def args(tmp_path, *flags):
    return build_parser().parse_args(['--home', str(tmp_path), *flags])


def test_cli_and_setup_edit_the_same_defaults(tmp_path):
    config_command.run(args(tmp_path, 'config', 'set', '--robot', 'panda', '--agent', 'codex', '--model', 'm1'))
    values, models = start.values_for(args(tmp_path))
    assert (values['robot'], values['agent'], values['model']) == ('panda', 'codex', 'm1')
    values.update(sim='robosuite', bench='capbench', model='m2')
    start.save_values(tmp_path, values)
    config_command.run(args(tmp_path, 'config', 'set', '--bench', 'null'))
    user = load_user_config(tmp_path / 'config.yaml')
    assert user.benchmark is None and user.agents['codex'].model == 'm2'
    assert 'name' not in (tmp_path / 'config.yaml').read_text()


@pytest.mark.parametrize('agent', ['claude-code', 'codex'])
def test_setup_authentication_roundtrip_and_agent_isolation(tmp_path, agent):
    key = tmp_path / 'provider.key'
    key.write_text('synthetic-api-key')
    values, _ = start.values_for(args(tmp_path, '--agent', agent, '--bench', 'capbench'))
    assert values['auth_mode'] == 'native' and values['api_key_file'] == ''
    values.update(auth_mode='api', api_key_file=str(key))
    start.save_values(tmp_path, values)
    reopened, _ = start.values_for(args(tmp_path, '--agent', agent))
    assert reopened['auth_mode'] == 'api' and reopened['api_key_file'] == str(key)
    choices = start.authentication_choices(tmp_path, ['claude-code', 'codex'])
    assert choices[agent]['api'] and choices[agent]['api_key_file'] == str(key)
    other = 'codex' if agent == 'claude-code' else 'claude-code'
    assert choices[other]['auth_mode'] == 'native' and choices[other]['api_key_file'] == ''
    assert 'synthetic-api-key' not in (tmp_path / 'config.yaml').read_text()
    reopened.update(auth_mode='native', api_key_file='')
    start.save_values(tmp_path, reopened)
    assert load_user_config(tmp_path / 'config.yaml').agents[agent].auth.key_file is None
    assert key.read_text() == 'synthetic-api-key'


def test_invalid_api_selection_cannot_write_defaults_or_launch(tmp_path, monkeypatch):
    from openrua.terminal import launcher
    values, _ = start.values_for(args(tmp_path, '--bench', 'capbench'))
    start.save_values(tmp_path, values)
    path = tmp_path / 'config.yaml'
    original = path.read_text()
    values.update(auth_mode='api', api_key_file=str(tmp_path / 'missing.key'))
    responses = iter([{'action': 'start', 'values': values}, {'action': 'quit'}])
    screens = []
    def screen(spec):
        screens.append(spec)
        return next(responses)
    launcher.setup(values, {}, str(path), lambda v: start.save_values(tmp_path, v),
        lambda v: pytest.fail('must not check resources'), lambda v: pytest.fail('must not launch'),
        {}, lambda: [], lambda n: None, screen,
        prepare=lambda *a: pytest.fail('must not prepare resources'))
    assert 'key' in screens[1]['notice']
    assert path.read_text() == original
    assert not (tmp_path / 'sandboxes').exists()
    values.update(auth_mode='native')
    with pytest.raises(UsageError, match='requires mode: api'):
        start.save_values(tmp_path, values)


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
    monkeypatch.setattr(start, 'open_interface', lambda args, c, name: opened.append((c, name)) or 0)
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
    (tmp_path / 'config.yaml').write_text('robot: panda\nbenchmark: robocasa365\nagent: codex\n')
    values, _ = start.values_for(args(tmp_path))
    assert values['robot'] == 'panda-omron'
    assert values['bench'] == 'robocasa365'
    explicit, _ = start.values_for(args(tmp_path, '--agent', 'codex', '--model', 'chosen'))
    assert explicit['agent'] == 'codex' and explicit['model'] == 'chosen'


def test_unnamed_starts_create_distinct_ids_without_reconnecting_old_default(tmp_path, monkeypatch):
    (tmp_path / 'sandboxes/openrua').mkdir(parents=True)
    (tmp_path / 'sandboxes/openrua/endpoint.json').write_text('{}')
    monkeypatch.setattr(start.service, 'connect', lambda path: pytest.fail('must not attach implicitly'))
    monkeypatch.setattr(start, 'check_values', lambda *a: (True, 'Ready'))
    monkeypatch.setattr(start, 'prepare', lambda *a: None)
    launched = []
    monkeypatch.setattr(start, 'launch', lambda a, v: launched.append(v['name']) or object())
    monkeypatch.setattr(start, 'open_interface', lambda *a: 0)
    for _ in range(2):
        start.run(args(tmp_path, '--cli', '--robot', 'panda', '--sim', 'robosuite'))
    assert len(set(launched)) == 2 and all(len(name) == 32 for name in launched)


def test_resume_does_not_load_config_or_launch_resources(tmp_path, monkeypatch):
    (tmp_path / 'config.yaml').write_text('broken: [')
    selected = object()
    monkeypatch.setattr(start.history, 'open_session', lambda home, name: (selected, name))
    opened = []
    monkeypatch.setattr(start, 'open_interface', lambda a, c, n: opened.append((c, n)) or 0)
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
    monkeypatch.setattr(start, 'save_values', lambda home, v: phases.append('save'))
    monkeypatch.setattr(start, 'check_values', lambda home, v: (phases.append('check') or True, 'Ready'))
    client = object()
    monkeypatch.setattr(start, 'launch', lambda a, v: phases.append('launch') or client)
    def setup(values, choices, location, save, check, launch, *callbacks, **kwargs):
        save(values)
        assert check(values)[0]
        return launch(values), values['name']
    monkeypatch.setattr(launcher, 'setup', setup)
    monkeypatch.setattr(start, 'open_interface', lambda a, c, name: phases.append('chat') or 0)
    assert start.run(args(tmp_path, '--bench', 'robocasa365')) == 0
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
    monkeypatch.setattr(launcher, 'chat', lambda client, name, *ops: opened.append((client, name)) or 0)
    assert start.run(args(tmp_path, '--resume', 'retained')) == 0
    assert opened == [('client', 'retained')]


def test_bad_selection_never_overwrites_existing_settings(tmp_path):
    path = tmp_path / 'config.yaml'
    original = 'robot: panda\nsimulator: robosuite\nbenchmark: null\n'
    path.write_text(original)
    values = dict(robot='panda-omron', sim='', bench='', agent='claude-code', model='', name='new')
    with pytest.raises(UsageError, match='name the simulator'):
        start.save_values(tmp_path, values)
    assert path.read_text() == original
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
    (tmp_path / 'config.yaml').write_text('robot: panda\nsimulator: maniskill\nbenchmark: libero_pro\n')
    values, _ = start.values_for(args(tmp_path, '--sim', 'maniskill'))
    assert values['robot'] == 'panda' and values['sim'] == 'maniskill'


@pytest.mark.parametrize('flag', ['--gui', '--cli'])
def test_non_tui_resume_needs_no_terminal_runtime(tmp_path, monkeypatch, flag):
    from openrua.terminal import launcher
    monkeypatch.setattr(launcher, 'check_terminal', lambda: pytest.fail('must not open terminal'))
    monkeypatch.setattr(start.history, 'open_session', lambda home, name: ('client', name))
    monkeypatch.setattr(start, 'open_interface', lambda *args: 0)
    assert start.run(args(tmp_path, flag, '--resume', 'running')) == 0


def test_legacy_chat_tui_opens_the_same_keyboard_frontend(tmp_path, monkeypatch):
    from openrua.cli.commands import chat
    from openrua.terminal import launcher
    monkeypatch.setattr(launcher, 'check_terminal', lambda: None)
    monkeypatch.setattr(chat, 'connect', lambda args: type('Client', (), {})())
    opened = []
    monkeypatch.setattr(launcher, 'chat', lambda client, name, *ops: opened.append(name) or 0)
    assert main(['--home', str(tmp_path), 'chat', '--tui', '--name', 'old']) == 0
    assert opened == ['old']


@pytest.mark.parametrize('saved', [False, True])
def test_custom_benchmark_selector_survives_preview_and_launch(tmp_path, monkeypatch, saved):
    from openrua.config import paths
    profile = tmp_path / 'custom-benchmark.yaml'
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
    profile = tmp_path / (kind + '.yaml')
    profile.write_text('type: panda\nmachine:\n  backend: {kind: real, discovery: {network: host}}\n'
                       if kind == 'robot' else paths.find('benchmarks', 'capbench').read_text())
    if saved:
        config_command.run(args(tmp_path, 'config', 'set', '--' + kind, str(profile)))
    monkeypatch.setattr(launcher, 'check_terminal', lambda: None)
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    monkeypatch.setattr(launcher, 'setup', lambda *a, **kw: pytest.fail('must not replace a custom environment'))
    original = (tmp_path / 'config.yaml').read_bytes() if saved else None
    phases = []
    monkeypatch.setattr(start, 'prepare', lambda h, v: phases.append(('prepare', v[kind])))
    monkeypatch.setattr(start, 'check_values', lambda h, v: (phases.append(('check', v[kind])) or True, 'Ready'))
    monkeypatch.setattr(start, 'launch', lambda a, v: phases.append(('launch', v[kind])) or 'client')
    monkeypatch.setattr(start, 'open_interface', lambda a, c, n: phases.append(('chat', c)) or 0)
    assert start.run(args(tmp_path, *([] if saved else ['--' + kind, str(profile)]))) == 0
    assert phases == [('prepare', str(profile)), ('check', str(profile)), ('launch', str(profile)), ('chat', 'client')]
    assert ((tmp_path / 'config.yaml').read_bytes() if (tmp_path / 'config.yaml').exists() else None) == original


def test_invalid_custom_environment_fails_before_building_or_starting(tmp_path, monkeypatch):
    from openrua.terminal import launcher
    profile = tmp_path / 'broken.yaml'
    profile.write_text('type: nonexistent-robot\nmachine: {}\n')
    monkeypatch.setattr(launcher, 'check_terminal', lambda: None)
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    monkeypatch.setattr(launcher, 'setup', lambda *a, **kw: pytest.fail('must not replace explicit input'))
    monkeypatch.setattr(start, 'prepare', lambda *a: pytest.fail('must not build'))
    monkeypatch.setattr(start, 'launch', lambda *a: pytest.fail('must not launch'))
    from openrua.errors import OpenRUAError
    with pytest.raises(OpenRUAError):
        start.run(args(tmp_path, '--robot', str(profile)))


def test_explicit_empty_benchmark_does_not_reintroduce_saved_default(tmp_path):
    config_command.run(args(tmp_path, 'config', 'set', '--bench', 'capbench'))
    values, _ = start.values_for(args(tmp_path, '--robot', 'panda', '--sim', 'robosuite', '--bench', ''))
    assert values['bench'] == ''


def test_custom_setup_explains_how_to_preserve_the_profile(tmp_path, monkeypatch):
    from openrua.terminal import launcher
    profile = tmp_path / 'robot.yaml'
    profile.write_text('type: panda\nmachine:\n  backend: {kind: real, discovery: {network: host}}\n')
    monkeypatch.setattr(launcher, 'check_terminal', lambda: None)
    monkeypatch.setattr(start, 'prepare', lambda *a: pytest.fail('must not build'))
    with pytest.raises(UsageError, match='custom profile editing') as error:
        start.run(args(tmp_path, '--setup', '--robot', str(profile)))
    assert 'without --setup' in error.value.hint
    assert not (tmp_path / 'config.yaml').exists()
