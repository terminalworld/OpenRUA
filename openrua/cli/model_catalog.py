"""Read a plugin's native model metadata in a temporary, robot-free terminal."""
from __future__ import annotations

import json
from contextlib import suppress
import os
from pathlib import Path
import selectors
import signal
import subprocess
import tempfile
import time
from uuid import uuid4

from openrua import agents
from openrua.cli import agent_settings
from openrua.config import paths
from openrua.errors import UnavailableError


def read(connection, *, timeout: float = 30) -> list[dict]:
    """Initialize metadata only, without submitting a user or model turn."""
    if connection is None:
        raise UnavailableError('This plugin does not expose native model discovery.')
    with tempfile.TemporaryFile() as errors:
        child = subprocess.Popen(connection.argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=errors, start_new_session=True)
        selector = selectors.DefaultSelector()
        selector.register(child.stdout, selectors.EVENT_READ)
        pending = b''
        deadline = time.monotonic() + timeout
        def send(update):
            for frame in update.outbound:
                child.stdin.write((json.dumps(frame) + '\n').encode())
            child.stdin.flush()
        try:
            send(connection.protocol.begin())
            while time.monotonic() < deadline:
                if not selector.select(min(1, max(0, deadline - time.monotonic()))):
                    continue
                chunk = os.read(child.stdout.fileno(), 65536)
                if not chunk:
                    errors.seek(0)
                    raise UnavailableError('Native model discovery ended before returning its list.',
                                           hint=errors.read().decode(errors='replace')[-2000:])
                pending += chunk
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    update = connection.protocol.receive(json.loads(line))
                    for event in update.events:
                        if event.kind == 'protocol_error':
                            raise UnavailableError(str(event.data))
                        if event.kind == 'setting' and event.data.get('setting') == 'model':
                            if event.data.get('error'):
                                raise UnavailableError(str(event.data['error']))
                            if 'choices' in event.data and not update.outbound:
                                if not event.data['choices']:
                                    raise UnavailableError('The native CLI returned no model choices.')
                                return event.data['choices']
                    send(update)
            raise UnavailableError('Native model discovery timed out.', hint='check the agent login and retry')
        finally:
            selector.close()
            with suppress(OSError):
                child.stdin.close()
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                with suppress(ProcessLookupError):
                    os.killpg(child.pid, signal.SIGTERM)
                try: child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    with suppress(ProcessLookupError):
                        os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
            child.stdout.close()


def choices(home: Path, values: dict, image: str) -> list[dict]:
    adapter, selected = agent_settings.resolve(home, values['agent'] or None)
    auth = selected.get('auth') or {'mode': 'native'}
    if values.get('auth_mode'):
        auth = {'mode': values['auth_mode'], 'key_file': values.get('api_key_file') or None}
    key = agents.api_key_file(adapter, auth)
    with tempfile.TemporaryDirectory(prefix='openrua-model-catalog-') as directory:
        root = Path(directory)
        workspace = root / 'workspace'; workspace.mkdir()
        if key is not None:
            profile = adapter.prepare_api_profile(key, root / 'profile')
        else:
            source = agents.resolve_profile(adapter, selected.get('credentials_dir'),
                paths.credentials_dir(home) / adapter.name, user_home=Path.home(), environment=os.environ)
            profile = adapter.prepare_profile(source, root / 'profile')
        name = 'openrua-model-catalog-' + uuid4().hex
        command = ['docker', 'run', '-d', '--rm', '--network', 'none', '--cap-drop', 'ALL',
                   '--security-opt', 'no-new-privileges', '--name', name, '-v', f'{workspace}:/workspace']
        for mount in profile.mounts:
            command.extend(['-v', mount])
        command.extend([image, 'sleep', 'infinity'])
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=30)
            if result.returncode:
                raise UnavailableError(result.stderr.strip() or 'Could not open the native metadata terminal.')
            return read(adapter.model_catalog(name, values.get('model') or selected.get('model')
                                              or adapter.default_model, '', options=selected.get('options')))
        finally:
            subprocess.run(['docker', 'rm', '-f', name], capture_output=True, timeout=30)
