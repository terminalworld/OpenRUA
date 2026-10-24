"""Local session fixture for the production terminal's pseudoterminal test."""

import asyncio
import json
from pathlib import Path
import sys
import tempfile
import time

from openrua.agents.conversation import Event
from openrua.artifacts import WorkspaceFiles
from openrua.sessions.client import Client
from openrua.terminal.application import Application
from openrua.terminal.launcher import chat, runtime, setup
from tests.sessions.test_http import running_server, stop


async def main():
    with tempfile.TemporaryDirectory(prefix='openrua-ui-owner-') as directory:
        root = Path(directory)
        workspace = root / 'project'
        workspace.mkdir()
        (workspace / 'measurement.txt').write_text('Recorded measurement\n' * 70)
        store, session, native, execution, server = await running_server(root, artifacts=WorkspaceFiles(workspace))
        catalog = [
            {'value': 'demo-a', 'efforts': [{'value': 'high'}]},
            {'value': 'demo-b', 'default_effort': 'medium',
             'efforts': [{'value': 'low'}, {'value': 'medium'}]},
        ]
        for setting, value, choices in [('model', 'demo-a', catalog), ('effort', 'high', [{'value': 'high'}])]:
            session.native(Event('setting', data={'setting': setting, 'value': value,
                'choices': choices, 'effective_from': 'next_turn'}))
        client = Client(server.url, server.token)
        builds, saves = [], []

        def prepare(values, progress):
            builds.append(values.copy())
            progress('Preparing project fixture')
            time.sleep(.15)
            if len(builds) == 1:
                raise RuntimeError('Fixture build failed; review and retry')

        values = dict(robot='', sim='', bench='', agent='fixture', model='demo-a', name='fixture')
        try:
            source = Path(__file__).parents[1] / 'src/main.mjs'
            with Application(runtime, lambda: source) as application:
                result = await asyncio.to_thread(setup, values, {}, str(workspace / '.openrua/settings.json'),
                    lambda v: saves.append(v.copy()), lambda v: (True, 'Ready'), lambda v: client,
                    {}, lambda: [], lambda n: None, screen=application.screen,
                    environments=[{'selection': {'robot': 'panda', 'sim': 'robosuite', 'bench': 'libero_pro'}}],
                    application=application, prepare=prepare, scope=str(workspace))
                assert result == (client, 'fixture'), result
                await asyncio.to_thread(chat, client, 'fixture', lambda: [], lambda n: None,
                    screen=application.screen, facts=lambda n: {'interactive': True,
                        'project': str(workspace), 'robot_model': 'Fixture robot', 'model': 'demo-a'})
            state = store.snapshot()['state']
            Path(sys.argv[1]).write_text(json.dumps({'builds': len(builds), 'saves': len(saves),
                'settings': state['settings'], 'active_settings': state['messages'][0].get('turn_settings'),
                'closed': state['closed']}))
        finally:
            await stop(store, execution, server)


if __name__ == '__main__':
    asyncio.run(main())
