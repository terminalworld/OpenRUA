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
        with pytest.raises(RuntimeError, match='read-only'):
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
