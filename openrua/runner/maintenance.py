"""Session-owned connection operations, shared by clients and the agent terminal."""

from __future__ import annotations

import asyncio
from copy import deepcopy
from dataclasses import replace
import fcntl
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
from uuid import uuid4

from openrua import agents
from openrua.config import paths
from openrua.runner import live_state
from openrua.runner.connection import EnvironmentChanged
from openrua.runner.live import open_robot
from openrua.runner.robot_client import prepare_client
from openrua.sandbox.check import cli_ready

OPERATIONS = {'start', 'stop', 'restart', 'build'}


class Maintenance:
    def __init__(self, owned):
        self.owned = owned
        self.directory = paths.sandbox_dir(owned.name, owned.home) / 'maintenance'
        self.task = None
        self.server = None
        self.closing = False
        self.socket_alias = None

    @property
    def connection(self):
        connection = getattr(self.owned.robot, 'connection', None)
        if connection is None:
            raise ValueError('this session is still connecting a robot; finish its connection checks first')
        return connection

    def jobs(self):
        return self.owned.store.snapshot()['state'].get('maintenance', {})

    def _record(self, identity, **changes):
        def change(state):
            job = state.setdefault('maintenance', {})[identity]
            job.update(changes)
            return deepcopy(job), [{'kind': 'maintenance', 'data': deepcopy(job)}]
        job = self.owned.store.update(change)
        print(f"[robot] {job['operation']}: {job['status']}; {job.get('summary', '')}", flush=True)
        return job

    async def begin(self, *, connect=True):
        if not (self.directory / 'robot.py').is_file():
            prepare_client(self.directory)
        # The resource owner lock proves no other owner can use this path.
        path = self.directory / 'control.sock'
        path.unlink(missing_ok=True)
        # Unix socket addresses are limited to about 100 bytes. A private,
        # short alias binds the inode in the mounted session directory even
        # when the user's OpenRUA home has a long path.
        self.socket_alias = tempfile.TemporaryDirectory(prefix='oru-')
        alias = Path(self.socket_alias.name) / 'session'
        alias.symlink_to(self.directory, target_is_directory=True)
        self.server = await asyncio.start_unix_server(self._client, path=str(alias / 'control.sock'), limit=32768)
        # Sandbox images use the host user's UID. Other host users must not
        # gain lifecycle access through a world-writable socket.
        path.chmod(0o600)
        # A restarted service reports unfinished work; it never repeats it.
        for identity, job in self.jobs().items():
            if job['status'] in {'accepted', 'running', 'waiting'}:
                self._record(identity, status='unknown', summary='previous owner ended; inspect connection before retrying')
        self.owned.store.update(lambda state: (state.pop('handover_pending', None), []))
        if connect and getattr(self.owned.robot, 'connection', None) is not None:
            await self.action({'operation': 'start', 'request_id': uuid4().hex})

    async def _client(self, reader, writer):
        try:
            body = json.loads(await asyncio.wait_for(reader.readline(), 10))
            result = await self.action(body, origin='agent')
            response = {'result': result}
        except Exception as exc:
            response = {'error': str(exc)}
        try:
            writer.write((json.dumps(response) + '\n').encode())
            await writer.drain()
        finally:
            writer.close()
            await writer.wait_closed()

    async def action(self, body, *, origin='client'):
        if not isinstance(body, dict) or not isinstance(body.get('operation'), str):
            raise ValueError('provide a connection operation')
        operation = body['operation']
        if operation == 'status':
            if set(body) - {'operation', 'request_id'}:
                raise ValueError('status accepts only request_id')
            state = self.owned.store.snapshot()['state']
            if body.get('request_id'):
                job = state.get('maintenance', {}).get(body['request_id'])
                if job is None:
                    raise ValueError('unknown operation request')
                return job
            facts = live_state.load(self.owned.name, self.owned.home, require_running=False)
            return {'robot': state.get('robot'), 'configuration': facts.get('configuration'),
                    'operations': list(self.jobs().values())[-10:]}
        if operation == 'logs':
            if set(body) != {'operation'}:
                raise ValueError('logs takes no other arguments')
            files = [self.connection.robot_log, self.directory / 'build.log']
            return {'logs': [{'name': str(path), 'text': self._tail(path)} for path in files]}
        if operation not in OPERATIONS or set(body) != {'operation', 'request_id'}:
            raise ValueError('use start, stop, restart or build with a request_id')
        identity = body['request_id']
        if not isinstance(identity, str) or not 1 <= len(identity) <= 128:
            raise ValueError('request_id must be a nonempty string of at most 128 characters')
        prior = self.jobs().get(identity)
        if prior:
            if prior['operation'] != operation:
                raise ValueError('request_id already belongs to another operation')
            return prior
        if self.closing or self.owned._ended:
            raise ValueError('the session is ending')
        if self.task is not None and not self.task.done():
            raise ValueError('a connection operation is already running; inspect status')
        self.connection
        def accept(state):
            job = {'request_id': identity, 'operation': operation, 'status': 'accepted',
                   'summary': 'accepted; inspect status with the same request_id'}
            state.setdefault('maintenance', {})[identity] = job
            return deepcopy(job), [{'kind': 'maintenance', 'data': deepcopy(job)}]
        accepted = self.owned.store.update(accept)
        # Acceptance is returned without waiting for the calling agent's turn.
        self.task = asyncio.create_task(self._run(identity, operation, origin))
        return accepted

    @staticmethod
    def _tail(path):
        if not path.exists():
            return '(no log yet)'
        with path.open('rb') as stream:
            stream.seek(max(0, path.stat().st_size - 20000))
            return stream.read(20000).decode(errors='replace')

    async def _run(self, identity, operation, origin):
        self._record(identity, status='running', summary=f'{operation} in progress')
        changed_resources = False
        try:
            if operation == 'build':
                await asyncio.to_thread(self._build)
            else:
                connection = self.connection
                resolved = None
                prepared = None
                if operation == 'start' and connection.machine is None:
                    await self._status('starting', 'preparing the robot connection; chat remains available')
                    changed_resources = True
                if operation in {'start', 'restart'}:
                    await asyncio.to_thread(self._build, True)
                    try:
                        resolved = await asyncio.to_thread(connection.prepare)
                    except EnvironmentChanged as changed:
                        await self._handover(identity, changed.resolved, origin)
                        self._record(identity, status='completed', summary='environment updated; connection ready')
                        return
                    prepared = await asyncio.to_thread(connection.stage, resolved)
                if operation == 'start' and connection.machine is not None:
                    raise ValueError('a robot connection is already owned; use restart')
                await self._status(
                    'updating' if operation == 'restart' else 'starting' if operation == 'start' else 'stopped',
                    f'{operation} requested', hold=operation in {'stop', 'restart'},
                    interrupt=origin == 'client' and operation in {'stop', 'restart'})
                changed_resources = True
                if operation in {'stop', 'restart'}:
                    await asyncio.to_thread(connection.shutdown)
                if operation in {'start', 'restart'}:
                    await asyncio.to_thread(connection.start, resolved, prepared)
                    await self._status('ready', 'connection ready; acquire current observations')
                else:
                    await self._status('stopped', 'connection stopped; chat and project remain available')
            self._record(identity, status='completed', summary=f'{operation} completed')
        except Exception as exc:
            details = str(exc) + (f"; {exc.hint}" if getattr(exc, 'hint', '') else '')
            summary = (details.splitlines() or ['operation failed'])[0][:400]
            self._record(identity, status='failed', summary=summary, details=details)
            if changed_resources or self.jobs()[identity].get('resources_changed'):
                await self._status('failed', summary)
        finally:
            if not self.closing:
                self.owned.store.update(lambda state: (state.pop('handover_pending', None), []))
                await self.owned.execution.wake()

    async def _status(self, status, reason, **delivery):
        connection = self.connection
        self.owned.robot.cameras = connection.cameras if status == 'ready' else ({}, None)
        self.owned.robot.robot_log = connection.robot_log
        await self.owned.execution.robot_status(status, reason, **delivery)

    async def _handover(self, identity, resolved, origin):
        old = self.connection
        await asyncio.to_thread(self._prepare_environment, resolved)
        # Wait only for environment replacement, never for driver-only updates.
        self.owned.store.update(lambda state: (state.update(handover_pending=True), []))
        self._record(identity, status='waiting', summary='tool environment changed; handover after the active turn')
        await self._status('waiting',
            'tool environment changed; finish this turn for an internal handover', hold=True,
            interrupt=origin == 'client')
        while self.owned.store.snapshot()['state']['active']:
            if self.closing:
                raise RuntimeError('session ending before handover')
            await asyncio.sleep(0.1)
        request = replace(old.request, continuing=True)
        previous = deepcopy(old.resolved)
        self._record(identity, resources_changed=True)
        try:
            await self.owned.move_to(lambda: open_robot(request, interactive=True, resolved=resolved),
                                     'updating the robot tool environment')
        except Exception:
            # A failed replacement keeps the same frontend and native identity.
            # Recreate the last working tool environment without starting hardware.
            await self.owned.move_to(lambda: open_robot(request, interactive=True, resolved=previous),
                                     'restoring the maintenance environment')
            raise
        self._record(identity, status='running', summary='tool environment ready; starting the robot')
        await asyncio.to_thread(self.connection.start, resolved)
        await self._status('ready', 'environment updated; acquire current observations')

    def _prepare_environment(self, resolved):
        """Prepare tools before replacing the old, still usable environment."""
        facts = live_state.load(self.owned.name, self.owned.home)
        adapter = agents.get(facts.get('agent_spec', facts['agent']), self.owned.home,
                             version=facts.get('agent_version'))
        backend = resolved.cfg['machine']['backend']
        image = backend['sandbox_image']
        check = adapter.sandbox_cli_check()
        probe = check[1] if check else adapter.version_probe()
        key = hashlib.sha256(image.encode()).hexdigest()[:16]
        locks = paths.home(self.owned.home) / 'preparation'
        locks.mkdir(parents=True, exist_ok=True)
        with (locks / f'{key}.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            exists = subprocess.run(['docker', 'image', 'inspect', image], capture_output=True, timeout=30).returncode == 0
            if exists and not cli_ready(image, 'test "$ROS_DISTRO" = ' + shlex.quote(backend['ros_distro'])):
                raise ValueError('target tool image ROS_DISTRO does not match the configuration; '
                                 'choose a matching sandbox_image or a new tag before restarting')
            install_changed = backend.get('sandbox_install') != self.connection.environment['backend'].get('sandbox_install')
            if exists and not install_changed and (not probe or cli_ready(image, probe)):
                return
            chosen = facts.get('agent_spec', facts['agent']) + ('@' + facts['agent_version'] if facts.get('agent_version') else '')
            argv = [sys.executable, '-m', 'openrua', '--home', str(self.owned.home), 'build', 'sandbox', '--agent', chosen]
            if backend.get('sandbox_install'):
                argv += ['--robot', resolved.robot]
            else:
                argv += ['--distro', backend['ros_distro'], '--tag', image]
            if exists:
                argv += ['--base-image', image]
            with (self.directory / 'build.log').open('a') as output:
                result = subprocess.run(argv, stdout=output, stderr=subprocess.STDOUT)
            if result.returncode or (probe and not cli_ready(image, probe)):
                raise RuntimeError('tool environment preparation failed; the existing environment was retained. '
                                   + self._tail(self.directory / 'build.log'))

    def _build(self, only_missing=False):
        resolved = self.connection.read_config()
        backend = resolved.cfg['machine']['backend']
        commands = []
        if backend['kind'] == 'sim':
            selector = ['--bench', resolved.benchmark] if resolved.benchmark else ['--sim', resolved.simulator]
            commands.append((backend['image'], ['-m', 'openrua', '--home', str(self.owned.home), 'build', *selector]))
        commands.extend((image, ['-m', 'openrua.robot.real.build', '--image', image, '--context', str(context)])
                        for image, context in (backend.get('build') or {}).items())
        if not commands and not only_missing:
            raise ValueError('the selected robot declares no backend.build images')
        locks = paths.home(self.owned.home) / 'preparation'
        locks.mkdir(parents=True, exist_ok=True)
        with (self.directory / 'build.log').open('a') as output:
            for image, argv in commands:
                key = hashlib.sha256(image.encode()).hexdigest()[:16]
                with (locks / f'{key}.lock').open('a') as lock:
                    fcntl.flock(lock, fcntl.LOCK_EX)
                    if only_missing and subprocess.run(['docker', 'image', 'inspect', image],
                                                       capture_output=True, timeout=30).returncode == 0:
                        continue
                    result = subprocess.run([sys.executable, *argv], stdout=output, stderr=subprocess.STDOUT)
                    if result.returncode:
                        raise RuntimeError(f'build failed for {image}; inspect robot logs: {self._tail(self.directory / "build.log")}')

    async def close(self):
        self.closing = True
        self.owned.store.update(lambda state: (state.update(handover_pending=True), []))
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        if self.task is not None and not self.task.done():
            # Do not abandon a thread that may own a live driver.
            await self.task
        (self.directory / 'control.sock').unlink(missing_ok=True)
        if self.socket_alias:
            self.socket_alias.cleanup()
