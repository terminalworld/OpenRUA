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


def test_existing_session_reconnects_without_reading_changed_defaults(tmp_path, monkeypatch):
    directory = tmp_path / 'sandboxes/openrua'
    directory.mkdir(parents=True)
    (directory / 'endpoint.json').write_text('{}')
    (tmp_path / 'config.yaml').write_text('invalid: [')
    client = object()
    monkeypatch.setattr(start.service, 'connect', lambda path: client)
    opened = []
    monkeypatch.setattr(start, 'open_interface', lambda args, c, name: opened.append((c, name)) or 0)
    assert start.run(args(tmp_path, '--name', 'openrua')) == 0
    assert opened == [(client, 'openrua')]
    with pytest.raises(UsageError, match='startup options'):
        start.run(args(tmp_path, '--name', 'openrua', '--agent', 'codex'))


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
    launched = []
    monkeypatch.setattr(start, 'launch', lambda a, v: launched.append(v['name']) or object())
    monkeypatch.setattr(start, 'open_interface', lambda *a: 0)
    for _ in range(2):
        start.run(args(tmp_path, '--cli', '--robot', 'panda'))
    assert len(set(launched)) == 2 and all(len(name) == 32 for name in launched)


def test_resume_does_not_load_config_or_launch_resources(tmp_path, monkeypatch):
    (tmp_path / 'config.yaml').write_text('broken: [')
    selected = object()
    monkeypatch.setattr(start.history, 'open_session', lambda home, name: (selected, name))
    opened = []
    monkeypatch.setattr(start, 'open_interface', lambda a, c, n: opened.append((c, n)) or 0)
    assert start.run(args(tmp_path, '--resume', 'old')) == 0
    assert opened == [(selected, 'old')]
    with pytest.raises(UsageError):
        start.run(args(tmp_path, '--resume', 'old', '--robot', 'panda'))
