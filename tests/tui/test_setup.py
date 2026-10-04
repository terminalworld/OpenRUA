"""First-run guidance saves shared defaults and never builds or logs in."""

import asyncio

from textual.widgets import Input, Static

from openrua.tui.setup import SetupApp
from tests.sessions.test_execution import until


def app_for(tmp_path, check, launch):
    values = dict(robot='panda', sim='robosuite', bench='', agent='one', model='m1', name='guided')
    saved = []
    app = SetupApp(values, {}, str(tmp_path / 'config.yaml'), saved.append, check, launch,
                   agent_models={'two': 'm2'})
    return app, saved


def test_missing_preparation_stays_visible_and_never_launches(tmp_path):
    async def run():
        calls = []
        app, saved = app_for(tmp_path, lambda values: (False, 'Missing image; run openrua build'), calls.append)
        async with app.run_test(size=(80, 38)) as pilot:
            app.query_one('#agent', Input).value = 'two'
            await pilot.pause()
            assert app.query_one('#model', Input).value == 'm2'
            app.query_one('#check').scroll_visible(animate=False)
            await pilot.pause()
            await pilot.click('#check')
            await until(lambda: len(saved) == 1 and not app.busy)
            assert saved[0]['agent'] == 'two'
            assert 'Missing image' in str(app.query_one('#result', Static).render())
            assert not calls
            await pilot.press('ctrl+q')
        assert app.return_value is None
    asyncio.run(run())


def test_start_returns_connected_session_after_successful_check(tmp_path):
    async def run():
        client = object()
        order = []
        def check(values):
            order.append('check')
            return True, 'Ready'
        def launch(values):
            order.append('launch')
            return client
        app, saved = app_for(tmp_path, check, launch)
        async with app.run_test(size=(80, 40)) as pilot:
            app.query_one('#start').scroll_visible(animate=False)
            await pilot.pause()
            await pilot.click('#start')
            await until(lambda: app.return_value is not None)
        assert saved and order == ['check', 'launch']
        assert app.return_value == (client, 'guided')
    asyncio.run(run())
