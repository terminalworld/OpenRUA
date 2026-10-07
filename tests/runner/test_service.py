"""A detached service has one startup owner and reports failures verbatim."""

import fcntl
import os
import sys
import time

import pytest

from openrua.errors import UnavailableError
from openrua.runner import service

WORKER = '''
from http.server import BaseHTTPRequestHandler, HTTPServer
import json, os, sys, threading
from pathlib import Path
from openrua.sessions.client import write_endpoint
class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200); self.end_headers()
        self.wfile.write(json.dumps({'state': {'closed': False}, 'pid': os.getpid(), 'sid': os.getsid(0)}).encode())
    def do_POST(self):
        self.send_response(200); self.end_headers()
        self.wfile.write(b'{"result": {"state": {"closed": true}}}')
        threading.Thread(target=self.server.shutdown).start()
    def log_message(self, *args): pass
server = HTTPServer(('127.0.0.1', 0), Handler)
write_endpoint(Path(sys.argv[1]), f'http://127.0.0.1:{server.server_port}', 'local-test')
server.serve_forever()
server.server_close()
'''


def test_service_survives_launcher_return_and_existing_endpoint_is_reused(tmp_path):
    endpoint, log, lock = (tmp_path / name for name in ('endpoint.json', 'service.log', 'start.lock'))
    client = service.start([sys.executable, '-c', WORKER, str(endpoint)], endpoint, log, lock, timeout=10)
    try:
        state = client.snapshot()
        assert state['pid'] != os.getpid() and state['sid'] != os.getsid(0)
        same = service.start(['must-not-be-executed'], endpoint, log, lock)
        assert same.snapshot()['pid'] == state['pid']
        assert log.stat().st_mode & 0o777 == 0o600
    finally:
        client.end()


def test_start_failure_preserves_original_error(tmp_path):
    endpoint, log, lock = (tmp_path / name for name in ('endpoint.json', 'service.log', 'start.lock'))
    with pytest.raises(UnavailableError, match='original startup detail'):
        service.start([sys.executable, '-c', "raise RuntimeError('original startup detail')"], endpoint, log, lock, timeout=10)
    assert 'Traceback' in log.read_text()
    assert not endpoint.exists()


def test_concurrent_launch_and_stale_endpoint_never_spawn(tmp_path, monkeypatch):
    endpoint, log, lock = (tmp_path / name for name in ('endpoint.json', 'service.log', 'start.lock'))
    monkeypatch.setattr(service.subprocess, 'Popen', lambda *a, **kw: pytest.fail('must not spawn'))
    with lock.open('a') as held:
        fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(UnavailableError, match='another client'):
            service.start(['unused'], endpoint, log, lock)
    endpoint.write_text('{}')
    with pytest.raises(UnavailableError):
        service.start(['unused'], endpoint, log, lock)


def test_startup_timeout_does_not_kill_or_replay_the_owner(tmp_path):
    endpoint, log, lock = (tmp_path / name for name in ('endpoint.json', 'service.log', 'start.lock'))
    with pytest.raises(UnavailableError, match='still unresolved'):
        service.start([sys.executable, '-c', 'import time; time.sleep(0.3)\n' + WORKER, str(endpoint)],
                      endpoint, log, lock, timeout=0.05)
    deadline = time.monotonic() + 10
    while not endpoint.exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert endpoint.exists()
    client = service.connect(endpoint)
    try:
        assert client.snapshot()['state']['closed'] is False
    finally:
        client.end()
