"""Pi presentation cannot bypass existing startup, persistence or resume semantics."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from openrua.terminal import launcher


def test_private_screen_handoff_cleans_credentials_even_on_failure(tmp_path, monkeypatch):
    seen = []
    def node(argv, **kwargs):
        source, result = Path(argv[2]), Path(argv[4])
        seen.append(source.parent)
        assert source.stat().st_mode & 0o777 == 0o600
        assert source.parent.stat().st_mode & 0o777 == 0o700
        assert json.loads(source.read_text())['connection']['token'] == 'private'
        assert 'private' not in argv
        result.write_text('{"action":"history"}')
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(launcher, 'runtime', lambda: node)
    monkeypatch.setattr(launcher, 'entrypoint', lambda: tmp_path / 'main.mjs')
    assert launcher.run_screen({'connection': {'token': 'private'}}) == {'action': 'history'}
    assert not seen[0].exists()
    monkeypatch.setattr(launcher, 'runtime', lambda: lambda *a, **kw: SimpleNamespace(returncode=3))
    with pytest.raises(RuntimeError, match='may still be running'):
        launcher.run_screen({})


def test_setup_failure_is_editable_and_never_launches_before_ready():
    values = dict(robot='panda', sim='robosuite', bench='', agent='test', model='', name='unique')
    results = iter([{'action': 'start', 'values': values}, {'action': 'check', 'values': values}, {'action': 'quit'}])
    screens, saved = [], []
    def screen(spec):
        screens.append(spec)
        return next(results)
    assert launcher.setup(values, {}, 'config', saved.append, lambda v: (False, 'Missing image'),
                          lambda v: pytest.fail('must not launch'), {}, lambda: [], lambda n: None, screen) is None
    assert saved == [values, values]
    assert 'Missing image' in screens[1]['notice']


def test_startup_uses_checked_values_and_original_launcher():
    values = dict(robot='panda', sim='robosuite', bench='', agent='test', model='', name='unique')
    calls = []
    result = launcher.setup(values, {}, 'config', lambda v: calls.append('save'),
        lambda v: (calls.append('check') or True, 'Ready'),
        lambda v: calls.append('launch') or 'existing-client', {}, lambda: [], lambda n: None,
        lambda spec: {'action': 'start', 'values': values})
    assert result == ('existing-client', 'unique')
    assert calls == ['save', 'check', 'launch']


def test_history_switch_only_opens_existing_records():
    initial = SimpleNamespace(url='http://127.0.0.1:1234', token='token')
    archive = SimpleNamespace(read_only=True, reason='Ended', snapshot=lambda: {'state': {}, 'cursor': 1},
                              events=lambda cursor: [{'seq': 1, 'kind': 'closed'}] if cursor == 0 else [])
    responses = iter([{'action': 'history'}, {'action': 'select', 'id': 'old'}, {'action': 'quit'}])
    screens, opened = [], []
    def screen(spec):
        screens.append(spec)
        return next(responses)
    assert launcher.chat(initial, 'live', lambda: [{'id': 'old'}],
                         lambda name: opened.append(name) or (archive, name), screen) == 0
    assert opened == ['old']
    assert screens[-1]['connection']['read_only']
    assert screens[-1]['connection']['events'] == [{'seq': 1, 'kind': 'closed'}]


def test_invalid_setup_reply_does_not_write_settings():
    with pytest.raises(RuntimeError, match='not saved'):
        launcher.setup({'robot': ''}, {}, 'config', lambda v: pytest.fail('must not save'),
                       None, None, {}, None, None,
                       lambda spec: {'action': 'start', 'values': {'unexpected': 'value'}})


def test_prepare_only_on_start_and_recheck_before_launch():
    values = dict(robot='panda', sim='robosuite', bench='', agent='test', model='', name='unique')
    calls = []
    results = iter([{'action': 'check', 'values': values}, {'action': 'start', 'values': values}])
    launcher.setup(values, {}, 'config', lambda v: calls.append('save'),
        lambda v: (calls.append('check') or True, 'Ready'),
        lambda v: calls.append('launch') or 'client', {}, lambda: [], lambda n: None,
        lambda spec: next(results), prepare=lambda v, progress: calls.append('prepare'))
    assert calls == ['save', 'check', 'save', 'prepare', 'check', 'launch']


def test_failed_image_build_stays_editable_without_launching():
    values = dict(robot='panda', sim='robosuite', bench='', agent='test', model='', name='unique')
    results = iter([{'action': 'start', 'values': values}, {'action': 'quit'}])
    notices = []
    def screen(spec):
        notices.append(spec['notice'])
        return next(results)
    def prepare(*args):
        raise RuntimeError('build failed; retained log')
    launcher.setup(values, {}, 'config', lambda v: None,
        lambda v: pytest.fail('must not check failed build'),
        lambda v: pytest.fail('must not launch'), {}, lambda: [], lambda n: None,
        screen, prepare=prepare)
    assert 'build failed' in notices[1]
