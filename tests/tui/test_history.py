"""History commands are local UI actions, including in read-only transcripts."""

import asyncio

from textual.widgets import Input, OptionList, TextArea

from openrua.runner.history import ArchiveClient
from openrua.sessions.core import initial_state
from openrua.sessions.store import SQLiteStore
from openrua.tui.app import ChatApp
from openrua.tui.setup import SetupApp
from tests.sessions.test_execution import until


def test_archive_history_filters_and_switches_without_enqueuing(tmp_path):
    async def run():
        database = tmp_path / 'conversation.sqlite'
        state = initial_state(); state['closed'] = True
        store = SQLiteStore(database, state); store.close()
        client = ArchiveClient(database, 'Ended conversation')
        rows = [dict(id='first', title='Pick the cup', status='Ended', time=''),
                dict(id='second', title='Inspect the bowl', status='Recorded', time='')]
        opened = []
        def connect(name):
            opened.append(name)
            return client, name
        app = ChatApp(client, list_sessions=lambda: rows, open_session=connect)
        async with app.run_test(size=(100, 38)) as pilot:
            await until(lambda: app.online)
            app.query_one('#message', TextArea).load_text('/resume')
            await pilot.press('ctrl+s')
            await until(lambda: len(app.screen_stack) > 1)
            await until(lambda: app.screen.query_one(OptionList).option_count == 2)
            app.screen.query_one(Input).value = 'bowl'
            await pilot.pause()
            assert app.screen.query_one(OptionList).option_count == 1
            app.screen.query_one(OptionList).focus()
            await pilot.press('enter')
            await until(lambda: app.return_value is not None)
        assert opened == ['second'] and app.return_value == (client, 'second')
        assert client.snapshot()['state'] == state
    asyncio.run(run())


def test_resume_available_before_starting_a_new_robot(tmp_path):
    async def run():
        calls = []
        fields = dict(robot='', sim='', bench='', agent='', model='', name='generated')
        app = SetupApp(fields, {}, 'config.yaml', calls.append, calls.append, calls.append,
                       list_sessions=lambda: [], open_session=calls.append)
        async with app.run_test(size=(80, 38)) as pilot:
            command = app.query_one('#command', Input)
            command.value = '/resume'; command.focus()
            await pilot.press('enter')
            await until(lambda: len(app.screen_stack) > 1)
            await pilot.press('escape')
            assert not calls
            await pilot.press('ctrl+q')
    asyncio.run(run())


def test_live_resume_command_never_reaches_the_agent(tmp_path):
    from openrua.sessions.client import Client
    from tests.sessions.test_http import running_server, stop

    async def run():
        store, session, native, execution, server = await running_server(tmp_path)
        client = Client(server.url, server.token, timeout=2)
        app = ChatApp(client, list_sessions=lambda: [], open_session=lambda name: None)
        try:
            async with app.run_test(size=(100, 38)) as pilot:
                await until(lambda: app.online)
                app.query_one('#message', TextArea).load_text('/resume')
                await pilot.press('ctrl+s')
                await until(lambda: len(app.screen_stack) > 1)
                assert store.snapshot()['state']['messages'] == []
                await pilot.press('escape')
                assert not store.snapshot()['state']['closed']
                await pilot.press('ctrl+q')
        finally:
            await stop(store, execution, server)
    asyncio.run(run())
