"""Reconnect the native owner without restarting resources or replaying work."""

import asyncio
import sys
from types import SimpleNamespace

import pytest

from openrua.agents.conversation import Conversation
from openrua.cli import build_parser
from openrua.cli.commands import serve, start
from openrua.errors import UnavailableError
from openrua.runner import live_state, managed
from openrua.sessions.client import Client
from openrua.sessions.core import Session, initial_state
from openrua.sessions.execution import Execution
from openrua.sessions.store import SQLiteStore
from openrua.terminal import launcher
from tests.sessions.test_execution import FakeProtocol, FakeTransport, setup, until


@pytest.mark.parametrize('entry', ['history', 'terminal'])
def test_resume_live_owner_reconnects_same_native_thread_without_replaying(tmp_path, monkeypatch, entry):
    async def run():
        name = 'recover-native'
        workspace = tmp_path / 'workspace'
        workspace.mkdir()
        program = tmp_path / 'native.py'
        program.write_text('''
import json, sys
from pathlib import Path
workspace = Path(sys.argv[1])
with (workspace / 'starts').open('a') as stream:
    stream.write('started\\n')
def emit(kind, turn=None, **data):
    print(json.dumps(dict(kind=kind, turn_id=turn, data=data)), flush=True)
for line in sys.stdin:
    frame = json.loads(line)
    if 'begin' in frame:
        emit('ready', session_id='retained-native-thread')
        continue
    turn = frame['submit']
    emit('turn_started', turn)
    with (workspace / 'actions').open('a') as stream:
        stream.write(frame['text'] + '\\n')
    if frame['text'] == 'inspect':
        emit('tool_output', turn, output='x' * (5 * 1024 * 1024))
        emit('input_required', turn, request_id='old-question', questions=[])
        print('invalid native output', flush=True)
    else:
        emit('turn_finished', turn, status='completed')
''')
        live_state.save(name, tmp_path, status='running', backend='real', agent='fixture',
                        sandbox='same-sandbox', model='model', proxy='', options={},
                        project=str(workspace), workspace=str(workspace))
        stops, native_ids = [], []
        def conversation(sandbox, model, proxy, **kwargs):
            assert sandbox == 'same-sandbox'
            native_ids.append(kwargs.get('session_id'))
            return Conversation([sys.executable, '-u', str(program), str(workspace)], FakeProtocol())
        robot = SimpleNamespace(power_off=lambda: stops.append('robot'),
                                stopped=lambda: None, stop_motion=lambda: {})
        monkeypatch.setattr(managed, 'open_robot', lambda *a, **kw: robot)
        monkeypatch.setattr(managed.agents, 'get', lambda *a, **kw: SimpleNamespace(conversation=conversation))
        args = build_parser().parse_args(['--home', str(tmp_path), 'serve', '--name', name])
        serving = asyncio.create_task(serve.serve(args))
        endpoint = tmp_path / 'sandboxes' / name / 'endpoint.json'
        try:
            await until(endpoint.exists)
            client = Client.from_file(endpoint)
            async def state_when(predicate):
                while True:
                    state = (await asyncio.to_thread(client.snapshot))['state']
                    if predicate(state):
                        return state
                    await asyncio.sleep(0.01)
            await asyncio.wait_for(state_when(lambda state: state['connected']), 5)
            inspect = await asyncio.to_thread(client.command, 'enqueue', client_id='terminal',
                                             request_id='first', text='inspect')
            await asyncio.wait_for(state_when(lambda state: not state['connected']), 5)
            events = await asyncio.to_thread(client.events)
            raw = [event['data'] for event in events if event['kind'] == 'native_frame'
                   and event['data'].get('kind') == 'tool_output']
            assert len(raw) == 1 and raw[0]['data']['output'] == 'x' * (5 * 1024 * 1024)
            await asyncio.to_thread(client.command, 'enqueue', client_id='terminal',
                                    request_id='second', text='continue')
            if entry == 'terminal':
                screens, opened = [], []
                answers = iter([{'action': 'history'}, {'action': 'select', 'id': name},
                                {'action': 'quit'}, {'action': 'answer', 'value': 'yes'}])
                def screen(spec):
                    screens.append(spec['mode'])
                    return next(answers)
                def select(target):
                    result = start.open_history(tmp_path, target)
                    opened.append(result)
                    return result
                assert await asyncio.to_thread(launcher.chat, client, name,
                    lambda: [{'id': name}], select, screen) == 0
                assert screens == ['chat', 'history', 'chat', 'leave']
                assert len(opened) == 1
                reconnected, selected = opened[0]
            else:
                results = await asyncio.gather(
                    asyncio.to_thread(start.open_history, tmp_path, name),
                    asyncio.to_thread(start.open_history, tmp_path, name))
                reconnected, selected = results[0]
                assert results[1][1] == name
            assert selected == name
            state = await asyncio.wait_for(state_when(lambda state: state['connected']), 5)
            assert state['native_session'] == 'retained-native-thread'
            assert state['paused'] and not state['closed'] and not state['requests']
            assert [message['status'] for message in state['messages']] == ['unknown', 'queued']
            assert native_ids == [None, 'retained-native-thread']
            assert not stops
            assert (workspace / 'actions').read_text() == 'inspect\n'

            # Repeated selection and a retry after a lost response are harmless.
            await asyncio.to_thread(start.open_history, tmp_path, name)
            await asyncio.to_thread(reconnected.request, '/api/actions/reconnect', {})
            assert (workspace / 'starts').read_text() == 'started\nstarted\n'
            assert (workspace / 'actions').read_text() == 'inspect\n'
            await asyncio.to_thread(reconnected.command, 'resolve_unknown', message_id=inspect['id'],
                                    note='The retained file confirms the inspection finished.')
            state = (await asyncio.to_thread(reconnected.snapshot))['state']
            await asyncio.to_thread(reconnected.command, 'resume', pause_id=state['pause_id'])
            state = await asyncio.wait_for(state_when(
                lambda value: value['messages'][-1]['status'] == 'completed'), 5)
            assert [message['status'] for message in state['messages']] == ['reconciled', 'completed']
            assert (workspace / 'actions').read_text() == 'inspect\ncontinue\n'
            assert not stops
            await asyncio.to_thread(reconnected.end)
            await asyncio.wait_for(serving, 5)
            assert stops == ['robot']
        finally:
            if not serving.done():
                serving.cancel()
            await asyncio.gather(serving, return_exceptions=True)
    asyncio.run(run())


@pytest.mark.parametrize('withdrawn', [False, True])
def test_reconnect_before_first_native_identity_keeps_unsent_work_paused(tmp_path, monkeypatch, withdrawn):
    async def run():
        store = SQLiteStore(tmp_path / 'conversation.sqlite', initial_state())
        session = Session(store)
        initial = FakeTransport(session)
        async def no_handshake(frame):
            initial.writes.append(frame)
        initial.write = no_handshake
        async def first_transport():
            return initial
        execution = Execution(session, Conversation([], FakeProtocol()), first_transport, tmp_path / 'owner.lock')
        await execution.start()
        owner = managed.ManagedSession(SimpleNamespace(power_off=lambda: None), execution, store)
        replacement = FakeTransport(session)
        async def reopen(*args, **kwargs):
            async def transport():
                return replacement
            renewed = Execution(session, Conversation([], FakeProtocol()), transport, tmp_path / 'owner.lock')
            await renewed.start()
            return renewed
        monkeypatch.setattr(managed, '_open_execution', reopen)
        try:
            message = await execution.command('enqueue', client_id='person', request_id='1', text='inspect')
            if withdrawn:
                await execution.command('withdraw', message_id=message['id'])
            await initial.frames.put(None)
            await until(lambda: not execution.usable)
            assert store.snapshot()['state']['native_session'] is None
            await owner.reconnect({})
            await until(lambda: store.snapshot()['state']['connected'])
            state = store.snapshot()['state']
            assert state['native_session'] == 'native' and state['paused']
            assert state['messages'][0]['status'] == ('withdrawn' if withdrawn else 'queued')
            assert initial.writes == replacement.writes == [{'begin': True}]
        finally:
            await owner.end()
            store.close()
    asyncio.run(run())


def test_reconnect_waits_for_connection_maintenance_and_shutdown(tmp_path, monkeypatch):
    async def run():
        store, session, transport, execution = await setup(tmp_path)
        owner = managed.ManagedSession(SimpleNamespace(power_off=lambda: None), execution, store)
        operation = asyncio.create_task(asyncio.Event().wait())
        maintenance = SimpleNamespace(task=operation, closing=False)
        owner.maintenance = maintenance
        opened = []
        async def open_execution(*args, **kwargs):
            opened.append(True)
            raise AssertionError('must not replace a native process during maintenance')
        monkeypatch.setattr(managed, '_open_execution', open_execution)
        try:
            # A healthy or handshaking connection needs no replacement.
            await owner.reconnect({})
            await transport.frames.put(None)
            await until(lambda: not execution.usable)
            with pytest.raises(UnavailableError, match='connection is being updated'):
                await owner.reconnect({})
            maintenance.closing = True
            with pytest.raises(UnavailableError, match='stopping or has stopped'):
                await owner.reconnect({})
            assert not opened
        finally:
            operation.cancel()
            await asyncio.gather(operation, return_exceptions=True)
            owner.maintenance = None
            await owner.end()
            store.close()
    asyncio.run(run())


def test_failed_reconnect_preserves_robot_and_records_for_retry(tmp_path, monkeypatch):
    async def run():
        store, session, transport, execution = await setup(tmp_path)
        stops = []
        owner = managed.ManagedSession(SimpleNamespace(power_off=lambda: stops.append('robot')),
                                       execution, store, name='retained', home=tmp_path)
        async def unavailable(*args, **kwargs):
            raise RuntimeError('native startup failed')
        monkeypatch.setattr(managed, '_open_execution', unavailable)
        try:
            message = await execution.command('enqueue', client_id='person', request_id='1', text='inspect')
            await transport.frames.put(None)
            await until(lambda: not store.snapshot()['state']['connected'])
            with pytest.raises(RuntimeError, match='native startup failed'):
                await owner.reconnect({})
            state = store.snapshot()['state']
            assert state['messages'][0]['id'] == message['id']
            assert state['messages'][0]['status'] == 'unknown'
            assert state['native_session'] == 'native'
            assert state['paused'] and not state['closed'] and not stops
        finally:
            await owner.end()
            store.close()
    asyncio.run(run())


def test_reconnect_does_not_compete_with_partial_resource_shutdown(tmp_path):
    async def run():
        store, session, transport, execution = await setup(tmp_path)
        def unavailable():
            raise RuntimeError('driver stop failed')
        owner = managed.ManagedSession(SimpleNamespace(power_off=unavailable), execution, store)
        try:
            with pytest.raises(UnavailableError, match='driver stop failed'):
                await owner.end()
            with pytest.raises(UnavailableError, match='stopping or has stopped'):
                await owner.reconnect({})
        finally:
            await execution.close()
            store.close()
    asyncio.run(run())
