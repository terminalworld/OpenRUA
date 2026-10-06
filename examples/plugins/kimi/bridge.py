"""Pipe the pinned Kimi local server to OpenRUA without implementing an agent loop.

Run with --help. stdin/stdout carry JSON frames; the private server log stays in
its profile because the native startup banner contains a bearer token.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import quote
from urllib.request import ProxyHandler, Request, build_opener


class Bridge:
    def __init__(self, home, cwd, command):
        self.home, self.cwd, self.command = home, cwd, command
        self.process = None
        self.base = None
        self.session = None
        self.prompt = None
        self.last = None
        self.http = build_opener(ProxyHandler({}))

    def request(self, path, body=None, *, method=None, accepted_codes=(0,)):
        token = (self.home / 'server.token').read_text().strip()
        request = Request(self.base + '/api/v1/' + path,
            data=None if body is None else json.dumps(body).encode(), method=method,
            headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
        with self.http.open(request, timeout=5) as response:
            envelope = json.load(response)
        if envelope['code'] not in accepted_codes:
            raise RuntimeError(f"Kimi error {envelope['code']}: {envelope.get('msg', '')}")
        return envelope['data']

    def start(self):
        self.home.mkdir(parents=True, exist_ok=True, mode=0o700)
        env = {**os.environ, 'KIMI_CODE_HOME': str(self.home)}
        with open(self.home / 'server.log', 'ab', opener=lambda p, f: os.open(p, f, 0o600)) as log:
            self.process = subprocess.Popen([*self.command, 'web', '--no-open', '--host', '127.0.0.1', '--port', '0'],
                cwd=self.cwd, env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError('Kimi server exited; inspect the private profile/server.log')
            for path in (self.home / 'server/instances').glob('*.json'):
                info = json.loads(path.read_text())
                if info['pid'] == self.process.pid and info['host'] == '127.0.0.1' and info['port'] > 0:
                    self.base = f"http://127.0.0.1:{info['port']}"
                    meta = self.request('meta')
                    if meta['server_version'] != '2.1.1':
                        raise RuntimeError('Kimi server version mismatch; install the pinned 2.1.1 release')
                    return
            time.sleep(.05)
        raise TimeoutError('Kimi server did not start; inspect the private profile/server.log')

    def dispatch(self, method, params):
        if method == 'open':
            if self.session or params['cwd'] != str(self.cwd):
                raise ValueError('connection already opened or workspace differs')
            sid = params.get('session_id')
            result = self.request('sessions/' + quote(sid, safe='')) if sid else self.request('sessions', {'metadata': {'cwd': str(self.cwd)}})
            if result.get('metadata', {}).get('cwd') != str(self.cwd):
                raise ValueError('native session belongs to a different workspace')
            self.session = result['id']
            active = self.request(self.path('prompts'))
            if active['active'] or active['queued']:
                raise ValueError('native session still has pending work; inspect it before reconnecting')
            return result
        if not self.session:
            raise ValueError('open a native session first')
        if method == 'submit':
            if self.prompt:
                raise ValueError('native prompt is still active')
            result = self.request(self.path('prompts'), {'prompt_id': params['prompt_id'],
                'model': params['model'], 'permission_mode': 'manual',
                'content': [{'type': 'text', 'text': params['text']}]})
            self.prompt = result['prompt_id']
            self.last = None
            return result
        if method == 'cancel':
            # Completion can precede an in-flight cancellation. Never cancel a different prompt.
            if self.prompt is None:
                return {'aborted': False}
            if params['prompt_id'] != self.prompt:
                raise ValueError('cancellation does not match active prompt')
            return self.request(self.path('prompts/' + quote(self.prompt, safe='') + ':abort'), {}, accepted_codes=(0, 40903))
        if method == 'respond':
            if params['kind'] not in ('approvals', 'questions'):
                raise ValueError('unknown native interaction')
            return self.request(self.path(params['kind'] + '/' + quote(params['request_id'], safe='')), params['body'])
        raise ValueError('unknown bridge method')

    def path(self, suffix):
        return 'sessions/' + quote(self.session, safe='') + '/' + suffix

    def snapshot(self):
        if not self.prompt:
            return None
        transcript = self.request(self.path('transcript?agent_id=main&page_size=100'))
        turns = [t for t in transcript['items'] if t.get('triggerPromptId') == self.prompt]
        frame = {'type': 'snapshot', 'session_id': self.session, 'prompt_id': self.prompt,
                 'transcript': {'items': turns},
                 'approvals': self.request(self.path('approvals?status=pending'))['items'],
                 'questions': self.request(self.path('questions?status=pending'))['items']}
        if any(t['state'] in ('completed', 'failed', 'cancelled') for t in turns):
            self.prompt = None
        if frame == self.last:
            return None
        self.last = frame
        return frame

    def close(self):
        if self.process and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--home', type=Path, required=True)
    parser.add_argument('--cwd', type=Path, required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not args.command:
        parser.error('provide the native Kimi command after --')
    command = args.command[1:] if args.command[0] == '--' else args.command
    bridge = Bridge(args.home, args.cwd, command)
    inbox = queue.Queue()
    def read():
        try:
            for line in sys.stdin:
                inbox.put(json.loads(line))
        except Exception as exc:
            inbox.put(exc)
        finally:
            inbox.put(None)
    def emit(frame):
        print(json.dumps(frame), flush=True)
    def stopped(*unused):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stopped)
    try:
        bridge.start()
        threading.Thread(target=read, daemon=True).start()
        while True:
            try:
                frame = inbox.get(timeout=.2)
            except queue.Empty:
                frame = 'poll'
            if frame is None:
                return 0
            if isinstance(frame, Exception):
                raise frame
            if frame != 'poll':
                try:
                    result = bridge.dispatch(frame['method'], frame['params'])
                except Exception as exc:
                    emit({'id': frame['id'], 'error': str(exc)})
                    return 1
                emit({'id': frame['id'], 'result': result})
            snapshot = bridge.snapshot()
            if snapshot:
                emit(snapshot)
            if bridge.process.poll() is not None:
                raise RuntimeError('native Kimi server stopped')
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        bridge.close()


if __name__ == '__main__':
    raise SystemExit(main())
