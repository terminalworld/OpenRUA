"""Project declarations and personal preferences have distinct owners."""

import json
from pathlib import Path

import pytest

from openrua.cli import build_parser
from openrua.config import compose, projects, load_user_config
from openrua.errors import UsageError, ConfigError


def run(home, *words):
    args = build_parser().parse_args(['--home', str(home), *words])
    return args.fn(args)


def test_config_creates_owned_templates_and_relative_project_settings(tmp_path, monkeypatch):
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir(); monkeypatch.chdir(root)
    run(home, 'config', 'set', '--trust-workspace', '--robot', 'panda', '--sim', 'robosuite', '--agent', 'codex')
    raw = json.loads(projects.settings_path(root).read_text())
    assert raw['robot'] == './configs/robots/panda.yaml'
    assert raw['sim'] == './configs/simulators/robosuite.yaml'
    assert not (home / 'config.yaml').exists()
    chosen = projects.selection(root)
    assert compose(chosen['robot'], chosen['sim'], chosen['bench'], home).suite == 'Lift'
    run(home, 'config', 'set', '--bench', 'libero_pro', '--model', 'project-model')
    chosen = projects.selection(root)
    assert compose(chosen['robot'], chosen['sim'], chosen['bench'], home).benchmark == 'libero_pro'
    assert chosen['model'] == 'project-model'


def test_user_agent_preferences_are_explicit_and_isolated(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    home = tmp_path / 'home'
    run(home, 'config', 'set', '--user', '--agent', 'codex', '--model', 'personal-model', '--version', '0.163.0')
    user = load_user_config(home / 'config.yaml')
    assert user.agents['codex'].model == 'personal-model'
    assert not projects.settings_path(tmp_path).exists()
    with pytest.raises(UsageError, match='belong to a project'):
        run(home, 'config', 'set', '--user', '--robot', 'panda')
    with pytest.raises(UsageError, match='belong to the user'):
        run(home, 'config', 'set', '--auth', 'native')


@pytest.mark.parametrize('model', ['gpt-5.4', 'provider/model-1.2', 'claude-opus-4-6'])
def test_model_ids_remain_unchanged_after_saving_copying_and_overriding(tmp_path, monkeypatch, model):
    import shutil
    from openrua.cli.commands import start

    root, moved, caller = (tmp_path / name for name in ('project', 'moved', 'caller'))
    root.mkdir(); caller.mkdir()
    monkeypatch.chdir(caller)
    home = tmp_path / 'home'
    run(home, 'config', 'set', '-C', str(root), '--trust-workspace',
        '--robot', 'panda', '--sim', 'robosuite', '--agent', 'codex', '--model', model)
    assert json.loads(projects.settings_path(root).read_text())['model'] == model
    shutil.copytree(root, moved)
    shutil.rmtree(root)
    parser = build_parser()
    args = parser.parse_args(['--home', str(home), '-C', str(moved)])
    values, _ = start.values_for(args)
    assert values['model'] == model
    args.model = 'other/provider-2.3'
    values, _ = start.values_for(args)
    assert values['model'] == args.model
    assert projects.selection(moved)['model'] == model


def test_external_source_must_be_brought_into_the_project_before_saving(tmp_path):
    project = tmp_path / 'project'; project.mkdir()
    file = tmp_path / 'robot.yaml'
    file.write_text('machine: {backend: {kind: real, discovery: {network: host}}, robot: {model: Test}, cameras: {list: []}}\n')
    with pytest.raises(ValueError, match='inside'):
        projects.save_selection(project, dict(robot=str(file), sim='', bench='', agent='codex', model=''))
    assert not projects.settings_path(project).exists()


def test_old_global_environment_keys_are_rejected(tmp_path):
    (tmp_path / 'config.yaml').write_text('robot: panda\n')
    with pytest.raises(ConfigError, match='robot'):
        load_user_config(tmp_path / 'config.yaml')


def test_robot_listing_only_contains_bundled_starting_points(tmp_path, capsys):
    connected = tmp_path / 'robots' / 'old'
    connected.mkdir(parents=True)
    (connected / 'robot.yaml').write_text('not a registry entry')
    run(tmp_path, 'robots', '--json')
    rows = json.loads(capsys.readouterr().out)
    assert 'panda' in [row['name'] for row in rows]
    assert 'old' not in [row['name'] for row in rows]


def test_setup_and_connect_keep_other_projects_personal_defaults(tmp_path):
    from openrua.cli.commands import connect, start
    home, root = tmp_path / 'home', tmp_path / 'project'
    home.mkdir(); root.mkdir()
    (home / 'config.yaml').write_text('agent: claude-code\nagents:\n  codex: {model: personal-model}\n')
    values = dict(robot='panda', sim='robosuite', bench='', agent='codex', model='project-model',
                  auth_mode='native', api_key_file='', name='new')
    start.save_guided_values(home, values, root)
    connect.save_values(home, values)
    saved = load_user_config(home / 'config.yaml')
    assert saved.agent == 'claude-code' and saved.agents['codex'].model == 'personal-model'
    assert projects.selection(root)['model'] == 'project-model'


def test_doctor_and_build_receive_the_same_project_selection(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from openrua.cli.commands import doctor as doctor_command, build
    root, caller, home = tmp_path / 'project', tmp_path / 'caller', tmp_path / 'home'
    caller.mkdir()
    projects.save_selection(root, dict(robot='panda', sim='robosuite', bench='', agent='codex', model=''))
    monkeypatch.chdir(caller)
    seen = []
    monkeypatch.setattr(doctor_command.doctor, 'run', lambda **kwargs: seen.append(kwargs) or SimpleNamespace())
    monkeypatch.setattr(doctor_command.doctor, 'print_report', lambda *a, **k: 0)
    run(home, 'doctor', '-C', str(root))
    assert seen[-1]['robot'] == str(root / 'configs/robots/panda.yaml')
    assert seen[-1]['sim'] == str(root / 'configs/simulators/robosuite.yaml')
    assert seen[-1]['agent_names'] == ['codex']
    monkeypatch.setattr(build, '_sandbox', lambda *a: ('sandbox', 'digest'))
    monkeypatch.setattr(build, 'build_proxy', lambda *a: ('proxy', 'digest'))
    monkeypatch.setattr(build, 'build_simulator', lambda install, name, code_root, tag: seen.append(tag) or (tag, 'digest'))
    run(home, 'build', '-C', str(root))
    assert seen[-1].startswith('openrua-sim-robosuite-')
