"""Private frontend handoff and native terminal borrowing."""

from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import socket
import sys
from types import SimpleNamespace

from openrua.terminal.application import Application


def test_private_persistent_transport_and_native_output_restore():
    seen, operations = [], []

    def node(argv, **kwargs):
        fd = int(argv[argv.index('--control-fd') + 1])
        assert kwargs['pass_fds'] == (fd,)
        assert 'private-token' not in argv
        with socket.socket(fileno=os.dup(fd)) as peer:
            def reply(value): peer.sendall((json.dumps(value) + '\n').encode())
            reply({'kind': 'ready', 'pid': 99999999})
            with peer.makefile('r') as lines:
                for line in lines:
                    value = json.loads(line); operations.append(value['kind'])
                    if value['kind'] == 'close': break
                    if value['kind'] == 'screen':
                        source = Path(value['input']); seen.append(source.parent)
                        assert source.stat().st_mode & 0o777 == 0o600
                        assert source.parent.stat().st_mode & 0o777 == 0o700
                        assert json.loads(source.read_text())['token'] == 'private-token'
                        reply({'id': value['id'], 'result': {'action': 'quit'}})
                    elif 'id' in value: reply({'id': value['id'], 'result': None})
        return SimpleNamespace(returncode=0)

    class Terminal(io.StringIO):
        def isatty(self): return True

    stdout = Terminal()
    with redirect_stdout(stdout):
        with Application(lambda: node, lambda: Path('main.mjs')) as application:
            application.screen({'token': 'private-token'})
            def prepare():
                assert sys.stdout.isatty()
                print('progress line')
                application.perform('Sign in', lambda: print('native prompt'), native=True)
            application.perform('Preparing', prepare)
            application.screen({'token': 'private-token'})
    assert stdout.getvalue() == 'native prompt\n'
    assert operations.count('screen') == 2
    assert operations.count('suspend') == operations.count('resume') == 1
    assert all(not path.exists() for path in seen)
