"""One terminal frontend across configuration, preparation and conversation."""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import json
import os
from pathlib import Path
import signal
import socket
import sys
import tempfile
import threading


class _Lines:
    def __init__(self, emit, terminal):
        self.emit, self.pending, self.terminal = emit, '', terminal

    def isatty(self):
        return self.terminal.isatty()

    def write(self, text):
        self.pending += text
        while '\n' in self.pending:
            line, self.pending = self.pending.split('\n', 1)
            self.emit(line)
        return len(text)

    def flush(self):
        if self.pending:
            self.emit(self.pending)
            self.pending = ''


class Application:
    def __init__(self, runtime, entrypoint):
        self.runtime, self.entrypoint = runtime, entrypoint
        self.socket = self.reader = self.thread = self.directory = None
        self.pid = None
        self.sequence = 0
        self.failure = None
        self.lock = threading.Lock()
        self.stdout, self.stderr = sys.stdout, sys.stderr

    def __enter__(self):
        return self

    def _start(self):
        if self.socket is not None:
            return
        self.directory = tempfile.TemporaryDirectory(prefix='openrua-interface-')
        self.socket, child = socket.socketpair()
        self.socket.settimeout(30)
        self.reader = self.socket.makefile('r', encoding='utf-8')
        def run():
            try:
                result = self.runtime()([str(self.entrypoint()), '--control-fd', str(child.fileno()),
                                         '--parent-pid', str(os.getpid())],
                                        pass_fds=(child.fileno(),), return_completed_process=True)
                if result.returncode:
                    self.failure = RuntimeError(f'Terminal frontend exited with status {result.returncode}; inspect the retained session before retrying.')
            except BaseException as exc:
                self.failure = exc
            finally:
                child.close()
        self.thread = threading.Thread(target=run, name='openrua-terminal', daemon=True)
        self.thread.start()
        ready = self._receive()
        if ready.get('kind') != 'ready' or type(ready.get('pid')) is not int:
            raise RuntimeError('Terminal frontend did not initialize')
        self.pid = ready['pid']
        self.socket.settimeout(None)

    def _receive(self):
        line = self.reader.readline()
        if not line:
            raise RuntimeError(str(self.failure or 'Terminal frontend disconnected; inspect the retained session before retrying.'))
        return json.loads(line)

    def _send(self, message):
        self._start()
        with self.lock:
            self.socket.sendall((json.dumps(message) + '\n').encode())

    def _request(self, kind, **params):
        self.sequence += 1
        identity = self.sequence
        self._send({'kind': kind, 'id': identity, **params})
        response = self._receive()
        if response.get('id') != identity:
            raise RuntimeError('Terminal frontend returned a response for a different operation')
        if response.get('error'):
            raise RuntimeError(response['error'])
        return response.get('result')

    def screen(self, spec):
        self._start()
        path = Path(self.directory.name) / 'screen.json'
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(spec, stream)
            result = self._request('screen', input=str(path))
            if not isinstance(result, dict) or not isinstance(result.get('action'), str):
                raise RuntimeError('Terminal returned an invalid action; no action was taken')
            return result
        finally:
            path.unlink(missing_ok=True)

    def progress(self, line='', **details):
        self._send({'kind': 'progress', 'line': str(line), **details})

    def perform(self, title, operation, *, native=False):
        self._send({'kind': 'progress', 'title': title, 'line': ''})
        if native:
            self._request('suspend')
            try:
                with redirect_stdout(self.stdout), redirect_stderr(self.stderr):
                    return operation()
            finally:
                self._request('resume')
        stream = _Lines(self.progress, self.stdout)
        try:
            with redirect_stdout(stream), redirect_stderr(stream):
                return operation()
        finally:
            stream.flush()

    def __exit__(self, kind, value, traceback):
        if self.socket is not None:
            try:
                self.socket.sendall(b'{"kind":"close"}\n')
            except OSError:
                pass
            self.thread.join(timeout=5)
            if self.thread.is_alive() and self.pid:
                try:
                    os.kill(self.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                self.thread.join(timeout=5)
            self.reader.close()
            self.socket.close()
            self.directory.cleanup()
