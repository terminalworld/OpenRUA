"""Two local owners for exercising in-session setup through the real terminal."""

import asyncio
import json
from pathlib import Path
import sys
import tempfile

from openrua.sessions.client import Client
from openrua.terminal.application import Application
from openrua.terminal.launcher import chat, runtime, setup
from tests.sessions.test_http import running_server, stop


async def main():
    with tempfile.TemporaryDirectory(prefix='openrua-session-setup-') as directory:
        root = Path(directory)
        owners = []
        for name in ('original', 'fresh'):
            owners.append(await running_server(root / name))
        clients = [Client(owner[-1].url, owner[-1].token) for owner in owners]
        calls, saves = [], []
        rows = [dict(id=name, title=title, status='Running', time='') for name, title in
                [('original', 'Original conversation'), ('fresh', 'New conversation')]]
        try:
            source = Path(__file__).parents[1] / 'src/main.mjs'
            with Application(runtime, lambda: source) as application:
                def configure(name):
                    calls.append(name)
                    return setup(dict(robot='fixture', sim='', bench='', agent='fixture', model='demo', name='fresh'),
                        {}, str(root / 'settings.json'), lambda values: saves.append(values.copy()),
                        lambda values: (True, 'Ready'), lambda values: clients[1], {},
                        lambda: rows, lambda name: (clients[0], name),
                        screen=application.screen, application=application, previous_session=name)
                await asyncio.to_thread(chat, clients[0], 'original', lambda: rows,
                    lambda name: (clients[['original', 'fresh'].index(name)], name),
                    screen=application.screen, setup_session=configure,
                    facts=lambda name: dict(project=str(root), robot='fixture',
                        robot_model='Fixture robot', agent_title='Fixture agent', model='demo'))
            states = [owner[0].snapshot()['state'] for owner in owners]
            Path(sys.argv[1]).write_text(json.dumps(dict(calls=calls, saves=len(saves),
                messages=[[message['text'] for message in state['messages']] for state in states],
                closed=[state['closed'] for state in states],
                interrupted=[any('interrupt' in frame for frame in owner[2].writes) for owner in owners])))
        finally:
            for store, session, native, execution, server in owners:
                await stop(store, execution, server)


if __name__ == '__main__':
    asyncio.run(main())
