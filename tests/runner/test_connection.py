"""The conversation stays usable across robot startup, faults and replacement."""

import asyncio
from copy import deepcopy
from types import SimpleNamespace

import pytest

from openrua.config.loader import Composed
from openrua.runner import connection as module, live_state
from openrua.runner.connection import Connection
from openrua.runner.live import RobotRequest
from openrua.runner.maintenance import Maintenance
from openrua.runner.managed import ManagedSession
from tests.sessions.test_execution import setup, until


@pytest.fixture
def connection(tmp_path, monkeypatch):
    root = tmp_path / 'project'
    root.mkdir()
    request = RobotRequest(home=tmp_path / 'home', name='fixture', project=root, robot='fixture')
    cfg = {'machine': {'backend': {'kind': 'real', 'sandbox_image': 'tools', 'ros_distro': 'jazzy'},
                       'robot': {'model': 'fixture'}, 'cameras': {}}, 'agent': {'name': 'fixture'}}
    resolved = Composed(cfg, None, None, 'fixture', None, None)
    monkeypatch.setattr(module, 'compose', lambda *a, **kw: deepcopy(resolved))
    monkeypatch.setattr(module, 'normalize_arms', lambda _: None)
    monkeypatch.setattr(module.project, 'prepare', lambda *a: None)
    # Container preparation is exercised separately; these handovers replace
    # their environment factory with a controlled resource owner.
    monkeypatch.setattr(Maintenance, '_prepare_environment', lambda *a: None)
    live_state.save(request.name, request.home, workspace=str(root), sandbox='box', agent='fixture')
    environment = root / '.openrua/runs/initial'
    environment.mkdir(parents=True)
    conn = Connection(request, resolved, environment, environment / 'config.yaml', 7, 'net', 'box')
    calls = []
    fail = {'ready': False, 'stop': False}
    class Driver:
        dead = None
        def wait_ready(self):
            if fail['ready']:
                raise TimeoutError('graph unavailable')
        def shutdown(self):
            calls.append('stop')
            if fail['stop']:
                raise RuntimeError('stop unconfirmed')
        def stopped(self):
            return self.dead
    def start(*args, **kw):
        calls.append('start')
        assert kw['wait_ready'] is False
        return Driver()
    monkeypatch.setattr(module.bringup, 'start_robot', start)
    return conn, calls, fail, resolved


async def owner_for(tmp_path, conn):
    store, session, native, execution = await setup(tmp_path)
    robot = SimpleNamespace(connection=conn, power_off=conn.shutdown, stopped=conn.stopped,
                            stop_motion=lambda: {}, robot_log=conn.robot_log, cameras=({}, None))
    owner = ManagedSession(robot, execution, store, name=conn.request.name, home=conn.request.home)
    owner.maintenance = Maintenance(owner)
    await owner.maintenance.begin(connect=False)
    return owner, native


def test_failed_start_keeps_agent_and_accepts_repair_then_retries_in_place(connection, tmp_path):
    conn, calls, fail, _ = connection
    async def run():
        owner, native = await owner_for(tmp_path, conn)
        original = owner.execution
        try:
            fail['ready'] = True
            first = {'operation': 'start', 'request_id': 'first'}
            accepted = await owner.maintenance.action(first)
            assert accepted['status'] == 'accepted'
            await owner.maintenance.task
            state = owner.store.snapshot()['state']
            assert state['connected'] and not state['closed'] and not state['paused']
            assert state['robot']['status'] == 'failed'
            message = await owner.execution.command('enqueue', client_id='person', request_id='repair', text='inspect the driver log')
            assert owner.store.snapshot()['state']['active'] == message['id']
            await native.frames.put({'kind': 'turn_finished', 'turn_id': message['id'], 'data': {'status': 'completed'}})
            await until(lambda: owner.store.snapshot()['state']['active'] is None)
            fail['ready'] = False
            await owner.maintenance.action({'operation': 'start', 'request_id': 'retry'})
            await owner.maintenance.task
            assert owner.execution is original
            assert owner.store.snapshot()['state']['robot']['status'] == 'ready'
            # A lost acknowledgement cannot repeat even a failed operation.
            assert (await owner.maintenance.action(first))['status'] == 'failed'
            assert calls == ['start', 'stop', 'start']
        finally:
            await owner.end()
            owner.store.close()
    asyncio.run(run())


def test_restart_keeps_native_owner_and_never_starts_after_unconfirmed_stop(connection, tmp_path):
    conn, calls, fail, _ = connection
    async def run():
        owner, native = await owner_for(tmp_path, conn)
        original = owner.execution
        try:
            await owner.maintenance.action({'operation': 'start', 'request_id': 'start'})
            await owner.maintenance.task
            fail['stop'] = True
            await owner.maintenance.action({'operation': 'restart', 'request_id': 'bad-stop'})
            await owner.maintenance.task
            assert calls == ['start', 'stop']
            assert conn.machine is not None and owner.execution is original
            assert owner.store.snapshot()['state']['maintenance']['bad-stop']['status'] == 'failed'
            fail['stop'] = False
            await owner.maintenance.action({'operation': 'restart', 'request_id': 'good-stop'})
            await owner.maintenance.task
            assert calls == ['start', 'stop', 'stop', 'start']
            assert owner.execution is original
            assert owner.store.snapshot()['state']['native_session'] == 'native'
        finally:
            fail['stop'] = False
            await owner.end()
            owner.store.close()
    asyncio.run(run())


def test_driver_fault_interrupts_old_turn_holds_old_queue_and_allows_repair(connection, tmp_path):
    conn, calls, fail, _ = connection
    async def run():
        owner, native = await owner_for(tmp_path, conn)
        try:
            await owner.maintenance.action({'operation': 'start', 'request_id': 'start'})
            await owner.maintenance.task
            first = await owner.execution.command('enqueue', client_id='person', request_id='first', text='inspect')
            await native.frames.put({'kind': 'turn_started', 'turn_id': first['id']})
            await until(lambda: owner.store.snapshot()['state']['messages'][0]['status'] == 'running')
            old = await owner.execution.command('enqueue', client_id='person', request_id='old', text='move to bowl')
            owner.watch_robot(interval_s=0.005)
            conn.machine.dead = 'driver exited with 134'
            await until(lambda: any('interrupt' in f for f in native.writes))
            await native.frames.put({'kind': 'turn_finished', 'turn_id': first['id'], 'data': {'status': 'interrupted'}})
            await until(lambda: owner.store.snapshot()['state']['active'] is None)
            repair = await owner.execution.command('enqueue', client_id='person', request_id='repair', text='read logs')
            state = owner.store.snapshot()['state']
            assert state['active'] == repair['id'] and not state['paused']
            assert next(m for m in state['messages'] if m['id'] == old['id'])['held_reason']
        finally:
            await owner.end()
            owner.store.close()
    asyncio.run(run())


def test_broken_configuration_is_checked_before_a_working_driver_stops(connection, tmp_path, monkeypatch):
    conn, calls, fail, resolved = connection
    async def run():
        owner, native = await owner_for(tmp_path, conn)
        try:
            await owner.maintenance.action({'operation': 'start', 'request_id': 'start'})
            await owner.maintenance.task
            resolved.cfg['machine']['controller_config'] = str(tmp_path / 'missing-controller.yaml')
            await owner.maintenance.action({'operation': 'restart', 'request_id': 'bad-config'})
            await owner.maintenance.task
            assert calls == ['start']
            assert owner.store.snapshot()['state']['robot']['status'] == 'ready'
            assert owner.store.snapshot()['state']['maintenance']['bad-config']['status'] == 'failed'
        finally:
            await owner.end()
            owner.store.close()
    asyncio.run(run())


def test_unfinished_operations_are_reported_not_replayed(connection, tmp_path):
    conn, calls, _, _ = connection
    async def run():
        owner, _ = await owner_for(tmp_path, conn)
        await owner.maintenance.close()
        owner.store.update(lambda state: (state.update(maintenance={'lost': {
            'operation': 'restart', 'request_id': 'lost', 'status': 'running'}}), []))
        owner.maintenance = Maintenance(owner)
        try:
            await owner.maintenance.begin(connect=False)
            assert (await owner.maintenance.action({'operation': 'restart', 'request_id': 'lost'}))['status'] == 'unknown'
            assert not calls
        finally:
            await owner.end()
            owner.store.close()
    asyncio.run(run())


@pytest.mark.parametrize('replacement_fails', [False, True])
def test_environment_handover_retains_http_conversation_and_maintenance_fallback(connection, tmp_path, monkeypatch, replacement_fails):
    from dataclasses import replace
    from openrua.agents.conversation import Conversation
    from openrua.runner import managed, maintenance
    from openrua.sessions.core import Session
    from openrua.sessions.execution import Execution
    from openrua.sessions.http import LocalServer
    from openrua.sessions.client import Client
    from tests.sessions.test_execution import FakeProtocol, FakeTransport

    conn, calls, fail, resolved = connection
    async def run():
        owner, native = await owner_for(tmp_path, conn)
        server = LocalServer(owner.execution, owner.end, 'fixture-token', actions={'robot': owner.maintenance.action})
        owner.on_execution = lambda execution: setattr(server, 'execution', execution)
        server.start()
        client = Client(server.url, server.token)
        replacements = []
        async def open_execution(name, home, store, **kw):
            assert kw == {'resume': True}
            session = Session(store)
            transport = FakeTransport(session)
            async def factory(): return transport
            execution = Execution(session, Conversation(['fake'], FakeProtocol()), factory, tmp_path / 'owner.lock')
            await execution.start()
            await until(lambda: store.snapshot()['state']['connected'])
            return execution
        monkeypatch.setattr(managed, '_open_execution', open_execution)
        def open_environment(request, *, interactive, resolved):
            assert request.continuing and interactive
            distro = resolved.cfg['machine']['backend']['ros_distro']
            replacements.append(distro)
            if replacement_fails and distro == 'humble':
                raise RuntimeError('replacement image unavailable')
            current = Connection(request, resolved, conn.directory, conn.config_path, 7, 'net', 'box')
            live_state.save(request.name, request.home, workspace=str(request.project), sandbox='box', agent='fixture')
            return SimpleNamespace(connection=current, power_off=current.shutdown, stopped=current.stopped,
                                   stop_motion=lambda: {}, robot_log=current.robot_log, cameras=({}, None))
        monkeypatch.setattr(maintenance, 'open_robot', open_environment)
        try:
            await owner.maintenance.action({'operation': 'start', 'request_id': 'start'})
            await owner.maintenance.task
            # Old pending work is held; a fresh instruction after handover remains usable.
            old = owner.execution.session.enqueue('person', 'old', 'move to the bowl')
            resolved.cfg['machine']['backend']['ros_distro'] = 'humble'
            accepted = await asyncio.to_thread(client.request, '/api/actions/robot',
                                                {'operation': 'restart', 'request_id': 'change'})
            assert accepted['result']['status'] == 'accepted'
            await owner.maintenance.task
            state = (await asyncio.to_thread(client.snapshot))['state']
            assert not state['closed'] and state['connected'] and state['native_session'] == 'native'
            assert next(m for m in state['messages'] if m['id'] == old['id'])['held_reason']
            assert state['active'] is None
            if replacement_fails:
                assert replacements == ['humble', 'jazzy']
                assert state['maintenance']['change']['status'] == 'failed'
                assert owner.maintenance.connection.machine is None
            else:
                assert replacements == ['humble'] and state['robot']['status'] == 'ready'
                assert state['maintenance']['change']['status'] == 'completed'
            message = await asyncio.to_thread(client.command, 'enqueue', client_id='person', request_id='repair', text='inspect current connection')
            assert (await asyncio.to_thread(client.snapshot))['state']['active'] == message['id']
        finally:
            await server.close()
            await owner.end()
            owner.store.close()
    asyncio.run(run())


def test_agent_socket_returns_acceptance_without_waiting_for_its_own_turn(connection, tmp_path):
    from pathlib import Path
    from openrua.runner.robot_client import call
    conn, _, _, resolved = connection
    async def run():
        owner, native = await owner_for(tmp_path, conn)
        try:
            await owner.maintenance.action({'operation': 'start', 'request_id': 'start'})
            await owner.maintenance.task
            message = await owner.execution.command('enqueue', client_id='person', request_id='task', text='update environment')
            resolved.cfg['machine']['backend']['ros_distro'] = 'humble'
            path = Path(owner.maintenance.socket_alias.name) / 'session/control.sock'
            accepted = await asyncio.wait_for(asyncio.to_thread(call, path,
                {'operation': 'restart', 'request_id': 'from-agent'}), 1)
            assert accepted['status'] == 'accepted'
            await until(lambda: owner.store.snapshot()['state'].get('handover_pending'))
            status = await asyncio.to_thread(call, path, {'operation': 'status', 'request_id': 'from-agent'})
            assert status['status'] == 'waiting'
            assert owner.store.snapshot()['state']['active'] == message['id']
            # Closing while waiting cancels the planned handover without replay.
        finally:
            await owner.end()
            owner.store.close()
    asyncio.run(run())
