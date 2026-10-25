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
    ended = []
    initial = SimpleNamespace(url='http://127.0.0.1:1234', token='token',
        snapshot=lambda: {'state': {'closed': False}}, end=lambda: ended.append('live'))
    archive = SimpleNamespace(read_only=True, reason='Ended', snapshot=lambda: {'state': {}, 'cursor': 1},
                              events=lambda cursor: [{'seq': 1, 'kind': 'closed'}] if cursor == 0 else [])
    responses = iter([{'action': 'history'}, {'action': 'select', 'id': 'old'},
                      {'action': 'answer', 'value': 'no'}, {'action': 'quit'}])
    screens, opened = [], []
    def screen(spec):
        screens.append(spec)
        return next(responses)
    assert launcher.chat(initial, 'live', lambda: [{'id': 'old'}],
                         lambda name: opened.append(name) or (archive, name), screen) == 0
    assert opened == ['old']
    assert ended == ['live']
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


def test_ended_workspace_uses_recorded_external_root_without_execution(tmp_path):
    import base64
    from openrua.runner import history, live_state
    from openrua.sessions.client import Client
    from tests.runner.test_history import retained

    home, workspace = tmp_path / 'home', tmp_path / 'custom-workspace'
    workspace.mkdir()
    (workspace / 'notes.txt').write_text('retained measurement')
    png = b'\x89PNG\r\n\x1a\n' + b'example'
    (workspace / 'camera.png').write_bytes(png)
    (workspace / 'outside').symlink_to(tmp_path / 'secret')
    (tmp_path / 'secret').write_text('not a workspace file')
    directory, store = retained(home, 'ended', closed=True)
    before = store.snapshot()
    store.close()
    live_state.save('ended', home, workspace=str(workspace), status='stopped')
    original = {p: p.read_bytes() for p in directory.iterdir() if p.is_file()}
    archive, name = history.open_session(home, 'ended')
    clients = []
    def screen(spec):
        files = spec['connection']['workspace']
        client = Client(files['url'], files['token'])
        clients.append(client)
        assert client.workspace_read('notes.txt')['text'] == 'retained measurement'
        assert base64.b64decode(client.workspace_read('camera.png')['data']) == png
        assert len(client.workspace_list()['entries']) == 3
        for path in ('../secret', 'outside'):
            with pytest.raises(RuntimeError):
                client.workspace_read(path)
        for mutate in (lambda: client.command('enqueue', client_id='x', request_id='1', text='move'), client.end):
            with pytest.raises(RuntimeError, match='read-only'):
                mutate()
        assert client.snapshot() == before
        return {'action': 'quit'}
    assert launcher.chat(archive, name, None, None, screen) == 0
    assert {p: p.read_bytes() for p in original} == original
    assert not (directory / 'endpoint.json').exists()
    with pytest.raises(RuntimeError, match='unavailable'):
        clients[0].workspace_list()


def test_missing_workspace_keeps_transcript_and_closes_server_on_screen_error(tmp_path):
    from openrua.runner.history import ArchiveClient
    from openrua.sessions.client import Client
    from tests.runner.test_history import retained

    directory, store = retained(tmp_path, 'ended', closed=True)
    store.close()
    for root, message in [(None, 'No workspace path'), (tmp_path / 'missing', 'workspace is missing')]:
        archive = ArchiveClient(directory / 'conversation.sqlite', 'Ended', root)
        clients = []
        def screen(spec):
            files = spec['connection']['workspace']
            client = Client(files['url'], files['token'])
            clients.append(client)
            assert spec['connection']['snapshot']['state']['closed']
            with pytest.raises(RuntimeError, match=message):
                client.workspace_list()
            raise RuntimeError('screen failed')
        with pytest.raises(RuntimeError, match='screen failed'):
            launcher.chat(archive, 'ended', None, None, screen)
        with pytest.raises(RuntimeError, match='unavailable'):
            clients[0].workspace_list()


def test_new_plugin_selection_loads_capabilities_without_saving_or_borrowing_auth():
    values = dict(robot='panda', sim='robosuite', bench='', agent='one', model='draft-model',
                  name='id', auth_mode='api', api_key_file='/keys/one.key')
    specs, resolved = [], []
    actions = iter(['plugin', 'one', 'plugin', 'quit'])
    def screen(spec):
        specs.append(spec)
        agent = next(actions)
        if agent == 'quit':
            return {'action': 'quit'}
        return {'action': 'agent', 'agent': agent, 'values': dict(spec['values'])}
    def resolve(name):
        resolved.append(name)
        return {'api': True, 'auth_mode': 'native', 'api_key_file': '', 'model': ''}
    result = launcher.setup(values, {}, 'config', lambda v: pytest.fail('must not save'),
        lambda v: pytest.fail('must not inspect resources'), lambda v: pytest.fail('must not launch'),
        {}, lambda: [], lambda name: None, screen,
        authentication={'one': {'api': True}}, resolve_agent=resolve)
    assert result is None
    assert specs[1]['values']['agent'] == 'plugin'
    assert specs[1]['values']['auth_mode'] == 'native'
    assert specs[1]['values']['api_key_file'] == ''
    assert specs[1]['authentication']['plugin']['api']
    assert specs[2]['values']['model'] == 'draft-model'
    assert specs[2]['values']['api_key_file'] == '/keys/one.key'
    assert specs[3]['values']['api_key_file'] == ''
    assert resolved == ['plugin', 'one', 'plugin']


def test_invalid_plugin_keeps_setup_editable_and_original_selection():
    values = dict(agent='one', model='', auth_mode='native', api_key_file='')
    specs = []
    def screen(spec):
        specs.append(spec)
        return ({'action': 'agent', 'agent': './missing.yaml', 'values': dict(values)}
                if len(specs) == 1 else {'action': 'quit'})
    def resolve(name):
        raise ValueError('plugin file not found')
    launcher.setup(values, {}, 'config', lambda v: pytest.fail('must not save'),
        None, None, {}, None, None, screen, resolve_agent=resolve)
    assert specs[1]['values'] == values
    assert 'plugin file not found' in specs[1]['notice']


def test_docker_and_the_sign_in_are_set_up_when_a_start_needs_them():
    # The menu opens without them (browsing and saving need neither). The
    # terminal is plain between screens: Docker is set up there before the
    # images are built, and the sign-in once the agent's sandbox image
    # exists, both before anything starts.
    values = dict(robot='panda', sim='robosuite', bench='', agent='test', model='', name='unique')
    calls = []
    results = iter([{'action': 'start', 'values': values}])
    launcher.setup(values, {}, 'config', lambda v: calls.append('save'),
        lambda v: (calls.append('check') or True, 'Ready'),
        lambda v: calls.append('launch') or 'client', {}, lambda: [], lambda n: None,
        lambda spec: next(results), prepare=lambda v, progress: calls.append('prepare'),
        ready=lambda v: calls.append('docker'), sign_in=lambda v: calls.append('sign in'))
    assert calls == ['save', 'docker', 'prepare', 'sign in', 'check', 'launch']
    calls.clear()
    results = iter([{'action': 'connect', 'values': values}])
    launcher.setup(values, {}, 'config', lambda v: None, lambda v: pytest.fail('no check'),
        lambda v: pytest.fail('no launch'), {}, lambda: [], lambda n: None,
        lambda spec: next(results), connect=lambda v: calls.append('connect') or 'connected',
        ready=lambda v: calls.append('docker'), sign_in=lambda v: calls.append('sign in'),
        save_connect=lambda v: calls.append('save connecting choices'))
    assert calls == ['save connecting choices', 'docker', 'sign in', 'connect']


def test_a_declined_sign_in_stays_editable_with_the_command():
    from openrua.errors import AuthError
    values = dict(robot='panda', sim='robosuite', bench='', agent='test', model='', name='unique')
    results = iter([{'action': 'connect', 'values': values}, {'action': 'quit'}])
    notices = []
    def screen(spec):
        notices.append(spec['notice'])
        return next(results)
    def sign_in(v):
        raise AuthError('Test is not signed in on this computer', hint='run openrua login test')
    assert launcher.setup(values, {}, 'config', lambda v: None, lambda v: None,
        lambda v: pytest.fail('must not launch'), {}, lambda: [], lambda n: None, screen,
        connect=lambda v: pytest.fail('must not connect'), sign_in=sign_in) is None
    assert 'not signed in' in notices[1] and 'openrua login test' in notices[1]


def test_model_discovery_keeps_unsaved_agent_choices_and_first_setup_step():
    values = dict(robot='panda', sim='robosuite', bench='', agent='selected', model='old', name='id')
    responses = iter([{'action': 'models', 'values': values, 'agent_step': True}, {'action': 'quit'}])
    screens, discovered = [], []
    def screen(spec):
        screens.append(spec)
        return next(responses)
    result = launcher.setup(values, {}, 'project', lambda _: pytest.fail('discovery must not save settings'),
        lambda _: pytest.fail('discovery must not start/check a robot'), lambda _: None, {}, lambda: [],
        lambda _: None, screen, resolve_models=lambda incoming, intent: discovered.append((incoming, intent))
            or [{'value': 'native-model', 'label': 'Native model'}])
    assert result is None and discovered == [(values, 'start')]
    assert screens[1]['view'] == 'models' and screens[1]['agent_step']
    assert screens[1]['model_choices'] == [{'value': 'native-model', 'label': 'Native model'}]


def test_switching_agents_clears_the_previous_native_model_catalog():
    values = dict(robot='panda', sim='robosuite', bench='', agent='first', model='first-model', name='id')
    replies = iter([{'action': 'models', 'values': values},
                    {'action': 'agent', 'values': values, 'agent': 'second'}, {'action': 'quit'}])
    screens = []
    def screen(spec):
        screens.append(spec)
        return next(replies)
    launcher.setup(values, {}, 'project', lambda _: pytest.fail('must not save'),
        lambda _: None, lambda _: None, {}, lambda: [], lambda _: None, screen,
        resolve_models=lambda *_: [{'value': 'first-native-model'}],
        resolve_agent=lambda _: {'model': 'second-model', 'model_selection': False})
    assert screens[1]['model_choices'] == [{'value': 'first-native-model'}]
    assert screens[2]['values']['agent'] == 'second'
    assert screens[2]['values']['model'] == 'second-model'
    assert screens[2]['model_choices'] == []


def test_session_setup_cancel_and_failure_return_to_the_same_conversation():
    initial = SimpleNamespace(url='http://127.0.0.1:1234', token='token', snapshot=lambda: {'state': {'closed': True}})
    replies = iter([{'action': 'setup'}, {'action': 'setup'}, {'action': 'quit'}])
    screens, calls = [], []
    def configure(name, before_launch):
        calls.append(name)
        if len(calls) == 2:
            raise RuntimeError('Project setup failed')
        return None
    def screen(spec):
        screens.append(spec)
        return next(replies)
    assert launcher.chat(initial, 'old', lambda: [], lambda _: None, screen,
                         setup_session=configure) == 0
    assert calls == ['old', 'old']
    assert [spec['name'] for spec in screens] == ['old', 'old', 'old']
    assert all(spec['can_setup'] for spec in screens)
    assert 'Project setup failed' in screens[-1]['notice']


def test_session_setup_enters_the_new_session_and_history_can_return_to_the_old_one():
    old = SimpleNamespace(url='http://127.0.0.1:1234', token='old', snapshot=lambda: {'state': {'closed': False}})
    new = SimpleNamespace(url='http://127.0.0.1:5678', token='new', snapshot=lambda: {'state': {'closed': False}})
    replies = iter([{'action': 'setup'}, {'action': 'answer', 'value': 'yes'}, {'action': 'history'},
                    {'action': 'select', 'id': 'old'}, {'action': 'answer', 'value': 'yes'},
                    {'action': 'quit'}, {'action': 'answer', 'value': 'yes'}])
    screens = []
    def screen(spec):
        screens.append(spec)
        return next(replies)
    assert launcher.chat(old, 'old', lambda: [{'id': 'old'}], lambda _: (old, 'old'), screen,
                         setup_session=lambda name, before_launch: (new, 'new') if before_launch() else None) == 0
    assert [spec['name'] for spec in screens if spec['mode'] == 'chat'] == ['old', 'new', 'old']
    assert [spec['connection']['token'] for spec in screens if spec['mode'] == 'chat'] == ['old', 'new', 'old']


@pytest.mark.parametrize('choice,stops,leaves', [('no', 1, True), ('yes', 0, True), ('cancel', 0, False)])
def test_resource_choice_only_stops_on_explicit_no(choice, stops, leaves):
    calls = []
    client = SimpleNamespace(snapshot=lambda: {'state': {'closed': False}}, end=lambda: calls.append('end'))
    assert launcher.leave(client, 'current', lambda spec: {'action': 'answer', 'value': choice}) is leaves
    assert calls == ['end'] * stops


@pytest.mark.parametrize('readonly,closed', [(True, False), (False, True)])
def test_history_without_live_resources_never_offers_to_stop(readonly, closed):
    client = SimpleNamespace(read_only=readonly, snapshot=lambda: {'state': {'closed': closed}})
    assert launcher.leave(client, 'old', lambda _: pytest.fail('no execution resources'))


def test_cancelling_history_or_selecting_current_conversation_keeps_resources():
    client = SimpleNamespace(url='http://127.0.0.1:1', token='token',
        snapshot=lambda: pytest.fail('no departure has been selected'))
    replies = iter([{'action': 'history'}, {'action': 'quit'},
                    {'action': 'history_all'}, {'action': 'select', 'id': 'current'},
                    {'action': 'quit', 'ended': True}])
    assert launcher.chat(client, 'current', lambda: [], lambda _: pytest.fail('same conversation'),
                         lambda _: next(replies), list_all=lambda: []) == 0


def test_failed_resource_stop_cannot_open_another_conversation():
    def fail():
        raise RuntimeError('driver shutdown unconfirmed')
    client = SimpleNamespace(url='http://127.0.0.1:1', token='token',
        snapshot=lambda: {'state': {'closed': False}}, end=fail)
    replies = iter([{'action': 'history'}, {'action': 'select', 'id': 'other'},
                    {'action': 'answer', 'value': 'no'}, {'action': 'quit'},
                    {'action': 'quit', 'ended': True}])
    screens = []
    def screen(spec):
        screens.append(spec)
        return next(replies)
    assert launcher.chat(client, 'current', lambda: [], lambda _: pytest.fail('stop failed'), screen) == 0
    assert 'driver shutdown unconfirmed' in screens[-2]['notice']


@pytest.mark.parametrize('failed', ['snapshot', 'end'])
def test_confirmed_closed_records_resolve_a_lost_shutdown_reply(failed):
    calls = []

    def unavailable():
        calls.append(failed)
        raise RuntimeError('session service unavailable or result unknown')

    client = SimpleNamespace(url='http://127.0.0.1:1', token='token',
        snapshot=unavailable if failed == 'snapshot' else lambda: {'state': {'closed': False}},
        end=unavailable)
    archive = SimpleNamespace(read_only=True, snapshot=lambda: {'state': {'closed': True}})
    replies = iter([{'action': 'quit'}, {'action': 'answer', 'value': 'no'}])
    assert launcher.chat(client, 'current', lambda: [], lambda _: None,
        lambda _: next(replies), retained_session=lambda _: (archive, 'current')) == 0
    assert calls == [failed]


def test_unconfirmed_records_cannot_turn_a_failed_stop_into_success():
    def unavailable():
        raise RuntimeError('shutdown reply lost')

    client = SimpleNamespace(url='http://127.0.0.1:1', token='token',
        snapshot=lambda: {'state': {'closed': False}}, end=unavailable)
    archive = SimpleNamespace(read_only=True, snapshot=lambda: {'state': {'closed': False}})
    replies = iter([{'action': 'quit'}, {'action': 'answer', 'value': 'no'},
                    {'action': 'quit', 'ended': True}])
    screens = []

    def screen(spec):
        screens.append(spec)
        return next(replies)

    assert launcher.chat(client, 'current', lambda: [], lambda _: None, screen,
        retained_session=lambda _: (archive, 'current')) == 0
    assert 'shutdown reply lost' in screens[-1]['notice']


def test_setup_checks_before_departure_and_does_not_launch_when_departure_is_cancelled():
    values = dict(robot='panda', sim='robosuite', bench='', agent='test', model='', name='fresh')
    calls = []
    result = launcher.setup(values, {}, 'config', lambda _: None,
        lambda _: (calls.append('check') or True, 'Ready'), lambda _: pytest.fail('departure cancelled'),
        {}, lambda: [], lambda _: None, lambda _: {'action': 'start', 'values': values},
        prepare=lambda *_: calls.append('prepare'), before_launch=lambda: calls.append('leave') or False)
    assert result is None
    assert calls == ['prepare', 'check', 'leave']


def test_failed_new_session_after_confirmed_stop_returns_to_retained_history():
    calls = []
    client = SimpleNamespace(url='http://127.0.0.1:1', token='token',
        snapshot=lambda: {'state': {'closed': False}}, end=lambda: calls.append('end'))
    archive = SimpleNamespace(read_only=True, reason='Ended',
        snapshot=lambda: {'state': {'closed': True}, 'cursor': 0}, events=lambda _: [])
    replies = iter([{'action': 'setup'}, {'action': 'answer', 'value': 'no'}, {'action': 'quit'}])
    screens = []
    def screen(spec):
        screens.append(spec)
        return next(replies)
    def configure(name, before_launch):
        assert before_launch()
        assert before_launch()  # A preparation retry does not end the owner twice.
        raise RuntimeError('new environment failed')
    assert launcher.chat(client, 'current', lambda: [], lambda _: None, screen,
        setup_session=configure, retained_session=lambda name: (archive, name)) == 0
    assert calls == ['end']
    assert screens[-1]['connection']['read_only']
    assert 'new environment failed' in screens[-1]['notice']
