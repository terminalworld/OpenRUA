"""Consent precedes project mutation, and history stays in the chosen directory."""

from pathlib import Path

import pytest

from openrua.cli import build_parser, workspace
from openrua.cli.commands import start
from openrua.config import projects
from openrua.errors import UsageError
from openrua.terminal import launcher


def test_trust_is_explicit_exact_and_canonical(tmp_path):
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir()
    alias = tmp_path / 'alias'
    alias.symlink_to(root, target_is_directory=True)
    assert not projects.trusted(root, home)
    projects.trust(root, home)
    assert projects.trusted(alias, home)
    assert not projects.trusted(root / 'nested', home)
    assert not projects.trusted(tmp_path, home)


def test_declining_a_workspace_writes_nothing(tmp_path, monkeypatch):
    candidate, home = tmp_path / 'not-created', tmp_path / 'home'
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('sys.stdout.isatty', lambda: True)
    monkeypatch.setattr(launcher, 'workspace', lambda path: None)
    assert workspace.choose(candidate, home) is None
    assert not candidate.exists() and not home.exists()


def test_choosing_a_new_directory_requires_confirming_that_directory(tmp_path):
    root, other = tmp_path / 'first', tmp_path / 'other'
    results = iter([{'action': 'choose', 'path': str(other)}, {'action': 'trust', 'path': str(other)}])
    screens = []
    def screen(spec):
        screens.append(spec)
        assert not other.exists()
        return next(results)
    assert launcher.workspace(str(root), screen) == str(other)
    assert [s['path'] for s in screens] == [str(root), str(other)]


def test_noninteractive_start_requires_explicit_trust(tmp_path, monkeypatch):
    root, home = tmp_path / 'project', tmp_path / 'home'
    monkeypatch.setattr('sys.stdin.isatty', lambda: False)
    with pytest.raises(UsageError, match='not been trusted'):
        workspace.choose(root, home)
    assert not root.exists()
    assert workspace.choose(root, home, accept=True) == root
    assert root.is_dir() and projects.trusted(root, home)


def test_default_start_uses_cwd_and_creates_fresh_conversations(tmp_path, monkeypatch):
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir()
    monkeypatch.chdir(root)
    projects.trust(root, home)
    monkeypatch.setattr(start, 'offer_update', lambda *a: True)
    monkeypatch.setattr(start, 'prepare', lambda *a, **k: None)
    monkeypatch.setattr(start, 'check_values', lambda *a: (True, 'ready'))
    launched = []
    monkeypatch.setattr(start, 'launch', lambda args, values: launched.append((args.workspace, args.name)) or object())
    monkeypatch.setattr(start, 'open_interface', lambda *a, **kw: 0)
    for _ in range(2):
        args = build_parser().parse_args(['--home', str(home), '--cli', '--robot', 'panda', '--sim', 'robosuite'])
        start.run(args)
    assert [path for path, _ in launched] == [str(root), str(root)]
    assert launched[0][1] != launched[1][1]


def test_history_can_switch_scope_without_mutating_or_starting_sessions():
    results = iter([{'action': 'scope'}, {'action': 'scope'}, {'action': 'quit'}])
    screens = []
    def screen(spec):
        screens.append(spec)
        return next(results)
    launcher.choose_history(lambda: [{'id': 'here'}], lambda name: pytest.fail('not selected'), screen,
                            list_all=lambda: [{'id': 'here'}, {'id': 'elsewhere'}], scope='/project')
    assert [[r['id'] for r in spec['rows']] for spec in screens] == [['here'], ['here', 'elsewhere'], ['here']]


def test_project_selection_outlives_changes_to_global_defaults(tmp_path):
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir(); home.mkdir()
    projects.save_selection(root, dict(robot='panda', sim='robosuite', bench='', agent='codex', model='project-model',
                                       auth_mode='api', api_key_file='private-key', name='ignored'))
    (home / 'config.yaml').write_text('agent: claude-code\n')
    args = build_parser().parse_args(['--home', str(home), '--workspace', str(root)])
    values, _ = start.values_for(args)
    assert (values['robot'], values['sim'], values['bench'], values['agent'], values['model']) == (
        str(root / '.openrua/configs/robots/panda.yaml'), str(root / '.openrua/configs/simulators/robosuite.yaml'), '', 'codex', 'project-model')
    contents = (root / '.openrua/settings.json').read_text()
    assert 'private-key' not in contents and 'auth_mode' not in contents


def test_changing_agent_does_not_reuse_the_previous_agents_project_model(tmp_path):
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir(); home.mkdir()
    projects.save_selection(root, dict(robot='panda', sim='robosuite', bench='', agent='codex', model='codex-only-model'))
    (home / 'config.yaml').write_text('agents:\n  claude-code:\n    model: claude-project-default\n')
    args = build_parser().parse_args(['--home', str(home), '--workspace', str(root), '--agent', 'claude-code'])
    values, _ = start.values_for(args)
    assert values['model'] == 'claude-project-default'


@pytest.mark.parametrize('requested,model', [('codex@0.163.0', 'project-model'), ('claude-code', None)])
def test_native_up_uses_model_defaults_for_the_chosen_agent(tmp_path, monkeypatch, requested, model):
    from openrua.cli.commands import up
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir()
    projects.trust(root, home)
    projects.save_selection(root, dict(robot='panda', sim='robosuite', bench='', agent='codex@0.134.0', model='project-model'))
    async def check(args):
        assert args.model == model
        return 0
    monkeypatch.setattr(up, '_run', check)
    args = build_parser().parse_args(['--home', str(home), 'up', '--workspace', str(root), '--agent', requested])
    assert up.run(args) == 0


def test_a_bundled_agent_version_pin_is_not_saved_as_a_file_path(tmp_path):
    projects.save_selection(tmp_path, dict(robot='panda', sim='robosuite', bench='',
                                           agent='codex@0.134.0', model=''))
    assert projects.selection(tmp_path)['agent'] == 'codex@0.134.0'


def test_resuming_a_missing_project_keeps_history_without_recreating_it(tmp_path, monkeypatch):
    from openrua.runner import live_state
    from openrua.sessions.core import initial_state
    from openrua.sessions.store import SQLiteStore
    root = tmp_path / 'removed-project'
    state = initial_state()
    state['closed'] = True
    SQLiteStore(tmp_path / 'sandboxes/old/conversation.sqlite', state).close()
    live_state.save('old', tmp_path, status='stopped', backend='sim', project=str(root), workspace=str(root))
    monkeypatch.setattr(start.service, 'start', lambda *a, **k: pytest.fail('do not restart missing projects'))
    monkeypatch.setattr(start.workspace, 'choose', lambda *a, **k: pytest.fail('do not recreate missing projects'))
    client, name = start.open_history(tmp_path, 'old')
    assert client.read_only and 'missing' in client.reason
    assert not root.exists()
