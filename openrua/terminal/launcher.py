"""Launch the packaged terminal with private, short-lived input/output files."""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path


def runtime():
    try:
        from nodejs_wheel import node
    except ImportError as exc:
        raise RuntimeError("Pi requires its packaged runtime: pip install 'openrua[pi]'") from exc
    return node


def entrypoint() -> Path:
    entry = Path(__file__).parent / 'assets' / 'src' / 'main.mjs'
    if not entry.is_file():
        raise RuntimeError('Pi frontend assets are missing. Install the released openrua[pi] wheel, '
                           'or follow ui/terminal/README.md to build assets in a source checkout.')
    return entry


def run_screen(spec: dict) -> dict:
    node, entry = runtime(), entrypoint()
    with tempfile.TemporaryDirectory(prefix='openrua-terminal-') as directory:
        source, result = Path(directory) / 'input.json', Path(directory) / 'result.json'
        fd = os.open(source, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            json.dump(spec, stream)
        completed = node([str(entry), '--input', str(source), '--output', str(result)],
                         return_completed_process=True)
        if completed.returncode:
            raise RuntimeError(f'Pi terminal exited with status {completed.returncode}; '
                               'the session may still be running. Reconnect using --resume.')
        if not result.exists():
            return {'action': 'quit'}
        response = json.loads(result.read_text())
        if not isinstance(response, dict) or not isinstance(response.get('action'), str):
            raise RuntimeError('Pi returned an invalid screen result; no action was taken.')
        return response


def connection(client) -> dict:
    if not getattr(client, 'read_only', False):
        return {'url': client.url, 'token': client.token}
    snapshot, events, cursor = client.snapshot(), [], 0
    while True:
        batch = client.events(cursor)
        if not batch:
            break
        events.extend(batch)
        cursor = batch[-1]['seq']
    return {'read_only': True, 'reason': client.reason, 'snapshot': snapshot, 'events': events}


def choose_history(list_sessions, open_session, screen=run_screen):
    notice = ''
    while True:
        result = screen({'mode': 'history', 'rows': list_sessions(), 'notice': notice})
        if result['action'] == 'quit':
            return None
        if result['action'] != 'select' or not isinstance(result.get('id'), str):
            raise RuntimeError('Invalid history selection; no session was opened.')
        try:
            return open_session(result['id'])
        except (OSError, RuntimeError, ValueError) as exc:
            notice = str(exc)


def setup(values, choices, location, save, check, launch, agent_models,
          list_sessions, open_session, screen=run_screen):
    notice = ''
    while True:
        result = screen({'mode': 'setup', 'values': values, 'choices': choices,
                         'location': location, 'models': agent_models, 'notice': notice})
        action = result['action']
        if action == 'quit':
            return None
        if action == 'history':
            selected = choose_history(list_sessions, open_session, screen)
            if selected:
                return selected
            continue
        incoming = result.get('values')
        if action not in {'check', 'start'} or not isinstance(incoming, dict) or set(incoming) != set(values) or not all(isinstance(v, str) for v in incoming.values()):
            raise RuntimeError('Invalid setup result; settings were not saved.')
        values = incoming
        try:
            save(values)
            print('Checking preparation...', flush=True)
            ok, notice = check(values)
            if ok and action == 'start':
                print('Starting the session; this can take a few minutes...', flush=True)
                return launch(values), values['name']
        except Exception as exc:
            notice = str(exc) + '\n' + getattr(exc, 'hint', '')


def chat(client, name, list_sessions, open_session, screen=run_screen):
    while True:
        result = screen({'mode': 'chat', 'name': name, 'connection': connection(client)})
        if result['action'] == 'quit':
            return 0
        if result['action'] != 'history':
            raise RuntimeError('Invalid chat result; no action was taken.')
        selected = choose_history(list_sessions, open_session, screen)
        if selected:
            client, name = selected
