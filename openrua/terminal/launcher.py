"""Launch the packaged terminal with private, short-lived input/output files."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path


def runtime():
    try:
        from nodejs_wheel import node
    except ImportError as exc:
        raise RuntimeError("Terminal runtime is missing: repair with pip install -U openrua") from exc
    return node


def entrypoint() -> Path:
    entry = Path(__file__).parent / 'assets' / 'src' / 'main.mjs'
    if not entry.is_file():
        raise RuntimeError('Pi frontend assets are missing. Install the released openrua wheel, '
                           'or follow ui/terminal/README.md to build assets in a source checkout.')
    # In a source checkout the assets are built from ui/terminal by hand and
    # an editable install never rebuilds them: refuse ones older than the
    # source rather than run yesterday's terminal. The script that builds
    # them is the one that knows how to tell.
    checkout = Path(__file__).resolve().parents[2]
    script = checkout / 'scripts' / 'terminal_assets.py'
    if script.is_file() and (checkout / 'ui' / 'terminal' / 'src').is_dir():
        checked = subprocess.run([sys.executable, str(script), 'verify'], cwd=checkout,
                                 capture_output=True, text=True)
        if checked.returncode:
            raise RuntimeError((checked.stdout + checked.stderr).strip()
                               + f' (in {checkout})')
    return entry


def check_terminal() -> None:
    # Check installation before connecting to execution resources.
    runtime()
    entrypoint()
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise RuntimeError('Terminal chat needs an interactive terminal; use --gui for the browser or session commands for non-interactive operation.')


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


def workspace(directory: str, screen=run_screen) -> str | None:
    """Ask about the exact chosen path before the caller creates any files."""
    notice = ''
    while True:
        result = screen({'mode': 'workspace', 'path': directory, 'notice': notice})
        if result['action'] == 'quit':
            return None
        if result['action'] not in {'trust', 'choose'} or not isinstance(result.get('path'), str):
            raise RuntimeError('Invalid workspace selection; nothing was trusted.')
        candidate = Path(result['path']).expanduser().resolve() if result['path'].strip() else None
        if candidate is None or (candidate.exists() and not candidate.is_dir()):
            notice = 'Choose a directory path.'
            continue
        if result['action'] == 'trust' and str(candidate) == directory:
            return directory
        directory, notice = str(candidate), ''


def choose_history(list_sessions, open_session, screen=run_screen, *, environments=None, prepare=None,
                   list_all=None, scope=None, all_projects=False):
    notice = ''
    while True:
        result = screen({'mode': 'history', 'rows': (list_all if all_projects and list_all else list_sessions)(),
                         'notice': notice, 'scope': 'All projects' if all_projects else scope,
                         'can_all': list_all is not None})
        if result['action'] == 'quit':
            return None
        if result['action'] == 'scope' and list_all is not None:
            all_projects = not all_projects
            continue
        if result['action'] != 'select' or not isinstance(result.get('id'), str):
            raise RuntimeError('Invalid history selection; no session was opened.')
        try:
            return open_session(result['id'])
        except (OSError, RuntimeError, ValueError) as exc:
            notice = str(exc)


def setup(values, choices, location, save, check, launch, agent_models,
          list_sessions, open_session, screen=run_screen, *, environments=None, prepare=None, authentication=None, resolve_agent=None,
          connect=None, ready=None, sign_in=None, list_all=None, scope=None, save_connect=None, application=None, resolve_values=None,
          resolve_models=None, previous_session=None, before_launch=None):
    notice = ''
    authentication = dict(authentication or {})
    drafts = {}
    view, intent = None, 'start'
    model_choices = []
    agent_step = None
    def perform(title, operation, native=False):
        return application.perform(title, operation, native=native) if application else operation()
    while True:
        result = screen({'mode': 'setup', 'values': values, 'choices': choices,
                         'location': location, 'models': agent_models, 'notice': notice,
                         'environments': environments, 'authentication': authentication,
                         'project': scope, 'view': view, 'intent': intent, 'model_choices': model_choices,
                         'previous_session': previous_session,
                         'agent_step': agent_step})
        action = result['action']
        view = result.get('view') or 'review'
        agent_step = result.get('agent_step', view == 'agent-step')
        intent = result.get('intent') or intent
        if action == 'quit':
            return None
        if action == 'history':
            draft = result.get('values')
            if isinstance(draft, dict) and set(draft) == set(values) and all(isinstance(v, str) for v in draft.values()):
                values = draft
            selected = choose_history(list_sessions, open_session, screen, list_all=list_all, scope=scope)
            if selected:
                return selected
            continue
        incoming = result.get('values')
        if (action in {'start', 'check', 'connect', 'models'} and resolve_values and isinstance(incoming, dict)
                and set(incoming) == set(values) and all(isinstance(v, str) for v in incoming.values())):
            try:
                incoming = resolve_values(incoming)
            except Exception as exc:
                values, notice = incoming, str(exc) + '\n' + getattr(exc, 'hint', '')
                continue
        if action == 'connect' and connect is not None and isinstance(incoming, dict):
            if set(incoming) != set(values) or not all(isinstance(v, str) for v in incoming.values()):
                raise RuntimeError('Invalid setup result; settings were not saved.')
            values = incoming
            # Connecting a robot starts from the agent chosen here; the robot
            # is what the session finds out.
            try:
                if save_connect is not None:
                    perform('Saving agent settings', lambda: save_connect(incoming))
                # Native prompts borrow the terminal only when preparation
                # actually needs Docker setup or an agent sign-in.
                if ready is not None:
                    perform('Docker setup', lambda: ready(incoming))
                if sign_in is not None:
                    perform('Agent sign-in', lambda: sign_in(incoming))
                if before_launch is not None and not before_launch():
                    return None
                return perform('Connecting robot', lambda: connect(incoming))
            except Exception as exc:
                notice = str(exc) + '\n' + getattr(exc, 'hint', '')
                continue
        if action not in {'agent', 'check', 'start', 'models'} or not isinstance(incoming, dict) or set(incoming) != set(values) or not all(isinstance(v, str) for v in incoming.values()):
            raise RuntimeError('Invalid setup result; settings were not saved.')
        values = incoming
        if action == 'models':
            view = 'models'
            model_choices = []
            try:
                if resolve_models is None:
                    raise RuntimeError('Native model discovery is not available in this entry point.')
                model_choices = perform('Loading models', lambda: resolve_models(values, intent))
                notice = ''
            except Exception as exc:
                notice = str(exc) + '\n' + getattr(exc, 'hint', '')
            continue
        if action == 'agent':
            name = result.get('agent')
            if not isinstance(name, str) or not name.strip() or resolve_agent is None:
                notice = 'Enter an agent name or the path to its configuration file.'
                continue
            try:
                facts = resolve_agent(name)
            except Exception as exc:
                notice = str(exc) + '\n' + getattr(exc, 'hint', '')
                continue
            fields = [key for key in ('model', 'auth_mode', 'api_key_file') if key in values]
            drafts[values['agent']] = {key: values[key] for key in fields}
            model_choices = []
            authentication = {**authentication, name: facts}
            selected = drafts.get(name, {key: facts[key] for key in fields})
            values = {**values, **selected, 'agent': name}
            notice = ''
            continue
        try:
            perform('Saving project', lambda: save(values))
            if action == 'start' and ready is not None:
                perform('Docker setup', lambda: ready(values))
            if action == 'start' and prepare:
                perform('Preparing environment', lambda: prepare(values, application.progress if application else lambda line: print(line, flush=True)))
            if action == 'start' and sign_in is not None:
                perform('Agent sign-in', lambda: sign_in(values))
            ok, notice = perform('Checking preparation', lambda: check(values))
            if ok and action == 'start':
                if before_launch is not None and not before_launch():
                    return None
                return perform('Starting session', lambda: launch(values)), values['name']
        except Exception as exc:
            notice = str(exc) + '\n' + getattr(exc, 'hint', '')


def leave(client, name, screen=run_screen, *, application=None, after_end=None):
    """Ask before leaving an execution owner, retaining files in either case."""
    if getattr(client, 'read_only', False):
        return True
    state = client.snapshot()['state']
    if state.get('closed'):
        return True
    answer = screen({'mode': 'leave', 'name': name})
    if answer.get('action') == 'quit' or answer.get('value') == 'cancel':
        return False
    if answer.get('action') != 'answer' or answer.get('value') not in {'no', 'yes'}:
        raise RuntimeError('Invalid resource choice; the current session was not ended.')
    if answer['value'] == 'no':
        if application:
            application.perform('Stopping this session', client.end)
        else:
            client.end()
        if after_end:
            after_end()
    return True


def chat(client, name, list_sessions, open_session, screen=run_screen, *, facts=None,
         list_all=None, scope=None, setup_session=None, version='', application=None, retained_session=None):
    notice = ''
    while True:
        spec = {'mode': 'chat', 'name': name, 'connection': connection(client),
                'facts': facts(name) if facts else {}, 'version': version,
                'can_setup': setup_session is not None, 'notice': notice, 'manage_leave': True}
        def display(files):
            spec['connection']['workspace'] = files
            return screen(spec)
        result = (client.with_workspace(display)
                  if getattr(client, 'read_only', False) and hasattr(client, 'with_workspace')
                  else screen(spec))
        if result.get('ended') and result['action'] != 'quit' and retained_session:
            client, _ = retained_session(name)
        departure_confirmed = False
        def ended():
            nonlocal client, departure_confirmed
            departure_confirmed = True
            if retained_session:
                client, _ = retained_session(name)
        def leaving():
            nonlocal client, departure_confirmed
            if departure_confirmed:
                return True
            try:
                departure_confirmed = leave(client, name, screen, application=application, after_end=ended)
            except Exception:
                # A lost reply does not undo closure. Read the recorded outcome
                # without retrying end or starting another execution owner.
                confirmed = False
                if retained_session:
                    try:
                        recorded, _ = retained_session(name)
                        confirmed = (getattr(recorded, 'read_only', False)
                                     and recorded.snapshot()['state'].get('closed') is True)
                    except Exception:
                        pass
                if not confirmed:
                    raise
                client, departure_confirmed = recorded, True
            return departure_confirmed
        def select(target):
            if target == name and not getattr(client, 'read_only', False):
                return client, name
            return open_session(target) if leaving() else None
        if result['action'] == 'quit':
            try:
                if result.get('ended') or leaving():
                    return 0
                notice = ''
            except Exception as exc:
                notice = f'Could not confirm that this session stopped. {exc}\nCheck its status: openrua session --name {name} status'
            continue
        if result['action'] == 'setup' and setup_session is not None:
            try:
                selected = setup_session(name, leaving)
                if selected:
                    client, name = selected
                notice = ''
            except Exception as exc:
                notice = str(exc) + '\n' + getattr(exc, 'hint', '')
            continue
        if result['action'] not in {'history', 'history_all'}:
            raise RuntimeError('Invalid chat result; no action was taken.')
        selected = choose_history(list_sessions, select, screen, list_all=list_all,
                                  scope=(facts(name).get('project') if facts else scope),
                                  all_projects=result['action'] == 'history_all')
        if selected:
            client, name = selected
