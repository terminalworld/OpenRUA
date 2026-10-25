"""History browsing never starts an owner or mutates retained execution state."""

import asyncio

import pytest

from openrua.runner import history
from openrua.sessions.client import write_endpoint
from openrua.sessions.core import Session, initial_state
from openrua.sessions.store import SQLiteStore
from tests.sessions.test_http import running_server, stop


def retained(home, name, *, closed=False):
    directory = home / 'sandboxes' / name
    state = initial_state()
    state['closed'] = closed
    store = SQLiteStore(directory / 'conversation.sqlite', state)
    return directory, store


def test_old_named_and_new_id_conversations_share_searchable_history(tmp_path):
    first, a = retained(tmp_path, 'old-name')
    Session(a).enqueue(client_id='cli', request_id='1', text='Inspect the camera\nthen the bowl')
    second, b = retained(tmp_path, history.new_id(), closed=True)
    a.close(); b.close()
    (tmp_path / 'sandboxes' / 'alias').symlink_to(first, target_is_directory=True)
    rows = history.entries(tmp_path)
    assert {r['id'] for r in rows} == {first.name, second.name}
    old = next(r for r in rows if r['id'] == first.name)
    assert old['title'] == 'Inspect the camera then the bowl'
    assert old['status'] == 'Execution status unknown'


def test_stale_or_ended_session_opens_readonly_without_replay(tmp_path):
    for name, closed in [('ended', True), ('unknown', False)]:
        directory, store = retained(tmp_path, name, closed=closed)
        before = store.snapshot()
        store.close()
        write_endpoint(directory / 'endpoint.json', 'http://127.0.0.1:1', 'unused')
        client, found = history.open_session(tmp_path, name)
        assert found == name and client.read_only
        assert client.snapshot() == before
        with pytest.raises(RuntimeError, match='cannot send messages'):
            client.command('enqueue', text='must never run')
        assert client.snapshot() == before


def test_resume_live_owner_keeps_queue_and_native_conversation(tmp_path):
    async def run():
        store, session, native, execution, server = await running_server(tmp_path)
        directory = tmp_path / 'sandboxes' / 'old'
        directory.mkdir(parents=True)
        # The directory journal supplies discovery; live state comes from its API.
        archived = SQLiteStore(directory / 'conversation.sqlite', initial_state())
        archived.close()
        write_endpoint(directory / 'endpoint.json', server.url, server.token)
        try:
            await execution.command('enqueue', client_id='before', request_id='1', text='Inspect')
            before = store.snapshot()
            writes = list(native.writes)
            client, name = await asyncio.to_thread(history.open_session, tmp_path, 'old')
            assert not getattr(client, 'read_only', False)
            assert await asyncio.to_thread(client.snapshot) == before
            assert native.writes == writes
        finally:
            await stop(store, execution, server)
    asyncio.run(run())


@pytest.mark.parametrize('entry', ['startup', 'history'])
def test_liveness_probe_does_not_shorten_resource_shutdown(tmp_path, entry):
    from openrua.runner import service

    async def run():
        store, session, native, execution, server = await running_server(tmp_path)
        directory = tmp_path / 'sandboxes' / 'slow'
        directory.mkdir(parents=True)
        SQLiteStore(directory / 'conversation.sqlite', initial_state()).close()
        endpoint = directory / 'endpoint.json'
        write_endpoint(endpoint, server.url, server.token)
        finish = server.end

        async def slow_shutdown():
            await asyncio.sleep(3.2)
            await finish()

        server.end = slow_shutdown
        try:
            if entry == 'startup':
                client = await asyncio.to_thread(service.connect, endpoint)
            else:
                client, _ = await asyncio.to_thread(history.open_session, tmp_path, 'slow')
            result = await asyncio.to_thread(client.end)
            assert result['state']['closed']
        finally:
            await stop(store, execution, server)

    asyncio.run(run())


def test_sessions_still_running_are_marked_and_listed_first(tmp_path):
    async def run():
        store, session, native, execution, server = await running_server(tmp_path)
        live = tmp_path / 'sandboxes' / 'live'
        live.mkdir(parents=True)
        SQLiteStore(live / 'conversation.sqlite', initial_state()).close()
        write_endpoint(live / 'endpoint.json', server.url, server.token)
        gone, newer = retained(tmp_path, 'gone')
        newer.close()
        write_endpoint(gone / 'endpoint.json', 'http://127.0.0.1:1', 'unused')
        import os
        os.utime(live / 'conversation.sqlite', (0, 0))   # older, yet first
        try:
            rows = await asyncio.to_thread(history.entries, tmp_path)
            assert [(r['id'], r['status']) for r in rows] == [('live', 'Running'),
                                                              ('gone', 'Not answering')]
        finally:
            await stop(store, execution, server)
    asyncio.run(run())


def test_the_terminal_knows_where_a_sessions_workspace_is_and_who_is_at_work(tmp_path):
    from openrua.runner import live_state
    live_state.save('s', tmp_path, status='running', robot='panda', robot_model='Franka Panda',
                    agent='claude-code', agent_title='Claude Code', backend='sim', interactive=True,
                    workspace=str(tmp_path / 'sandboxes' / 's' / 'workspace' / 'workspace'))
    facts = history.robot_facts(tmp_path, 's')
    assert facts['agent_title'] == 'Claude Code' and facts['workspace'].endswith('workspace/workspace')
    assert facts['interactive'] is True
