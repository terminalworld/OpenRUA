"""Install a built wheel in isolation and check its public entry points and assets."""

from __future__ import annotations

import argparse
import json
import os
import select
import time
from pathlib import Path
import subprocess
import tempfile
import venv


def invalid_config_smoke(executable: Path, home: Path, environment: dict) -> None:
    """Check startup error handling in the installed wheel with existing files."""
    project = home / 'project'
    project.mkdir(parents=True)
    config = home / 'config.yaml'
    config.write_text('robot: panda\nsimulator: robosuite\nbenchmark: libero_pro\nagent: codex\n')
    original = config.read_bytes()
    argv = [str(executable), '--home', str(home), '--workspace', str(project),
            '--trust-workspace', '--cli']
    def start(*extra):
        return subprocess.run([*argv, *extra], cwd=project, env=environment,
                              capture_output=True, text=True, timeout=15)
    failed = start()
    assert failed.returncode == 78, failed.stderr
    assert 'config schema --user' in failed.stderr
    assert 'session may still be running' not in failed.stderr
    assert config.read_bytes() == original
    schema = subprocess.run([str(executable), '--home', str(home), 'config', 'schema', '--user'],
                            capture_output=True, text=True, check=True, env=environment)
    assert set(json.loads(schema.stdout)['properties']) == {'agent', 'agents', 'sandbox'}
    config.write_text('agent: codex\n')
    settings = project / '.openrua/settings.json'
    settings.parent.mkdir()
    settings.write_text('{')
    failed = start()
    assert failed.returncode == 78 and str(settings) in failed.stderr, failed.stderr
    settings.unlink()
    failed = start('--robot', str(project / 'missing.yaml'))
    assert failed.returncode == 66, failed.stderr
    assert not (home / 'sandboxes').exists()


def terminal_smoke(executable: Path, home: Path, environment: dict, manifest: Path | None = None,
                   *, agent: str = 'codex') -> None:
    """Exercise real TTY startup and detachment without starting robot resources."""
    import pty
    import fcntl
    import struct
    import termios
    master, slave = pty.openpty()
    # Check the full external manifest path in a known viewport.
    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 30, 180, 0, 0))
    child = subprocess.Popen([str(executable), '--home', str(home), '--workspace', str(home / 'project')],
                             stdin=slave, stdout=slave, stderr=slave, env=environment)
    os.close(slave)
    output, frame = bytearray(), bytearray()
    def wait(text):
        deadline = time.monotonic() + 30
        while text not in frame:
            if select.select([master], [], [], .05)[0]:
                try: data = os.read(master, 65536)
                except OSError: break
                output.extend(data); frame.extend(data)
            assert child.poll() is None, output.decode(errors='replace')
            assert time.monotonic() < deadline, output.decode(errors='replace')
        assert text in frame, output.decode(errors='replace')
    def key(data):
        time.sleep(.2)
        while select.select([master], [], [], 0)[0]:
            try: output.extend(os.read(master, 65536))
            except OSError: break
        frame.clear(); os.write(master, data)
    try:
        wait(b'Use and trust this robot workspace?'); key(b'\r')
        wait(b'Choose your environment'); key(b'2')
        wait(b'Choose a simulation'); key(b'1')
        wait(b'Choose an environment'); key(b'1')
        wait(b'Choose your coding agent')
        for title in (b'Claude Code', b'Codex', b'Kimi Code (preview)', b'ZCode (preview)', b'Other agent or manifest'):
            assert title in frame, output.decode(errors='replace')
        if manifest:
            key(b'5')
            wait(b'Coding agent name or manifest path'); key(str(manifest).encode() + b'\r')
        else:
            key({'claude-code': b'1', 'codex': b'2', 'kimi': b'3', 'zcode': b'4'}[agent])
        wait(b'Agent settings'); key(b'4')
        wait(b'API key (billed separately)'); key(b'2')
        wait(b'Agent settings'); key(b'5')
        wait(b'Do not paste the key here'); key(b'\x04')
        child.wait(timeout=5)
        while select.select([master], [], [], 0)[0]:
            try: output.extend(os.read(master, 65536))
            except OSError: break
        assert child.returncode == 0, output.decode(errors='replace')
        assert output.count(b'\x1b[?1049h') == output.count(b'\x1b[?1049l') == 1
        assert not (home / 'sandboxes').exists(), 'Setup cancellation started resources'
        assert not (home / 'config.yaml').exists(), 'Setup cancellation saved an API choice'
    finally:
        if child.poll() is None: child.kill()
        child.wait()
        os.close(master)


def archive_smoke(executable: Path, home: Path, environment: dict) -> None:
    """Open retained files through the packaged TUI without robot resources."""
    import pty
    python = executable.parent / 'python'
    subprocess.run([str(python), '-c', """
import sys
from pathlib import Path
from openrua.runner.live_state import save
from openrua.sessions.core import initial_state
from openrua.sessions.store import SQLiteStore
home = Path(sys.argv[1])
workspace = home / 'retained-files'
workspace.mkdir(parents=True)
(workspace / 'notes.txt').write_text('retained measurement')
state = initial_state(); state['closed'] = True
store = SQLiteStore(home / 'sandboxes' / 'ended' / 'conversation.sqlite', state)
store.close()
save('ended', home, status='stopped', workspace=str(workspace))
""", str(home)], check=True, env=environment)
    master, slave = pty.openpty()
    child = subprocess.Popen([str(executable), '--home', str(home), '--resume', 'ended'],
                             stdin=slave, stdout=slave, stderr=slave, env=environment)
    os.close(slave)
    output = bytearray()
    stage = 0
    try:
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline and child.poll() is None:
            if select.select([master], [], [], 0.1)[0]:
                try:
                    output.extend(os.read(master, 65536))
                except OSError:
                    break
            if stage == 0 and b'Read-only history' in output:
                os.write(master, b'/files\r'); stage = 1
            elif stage == 1 and b'notes.txt' in output:
                os.write(master, b'\x1b[B\r'); stage = 2
            elif stage == 2 and b'retained measurement' in output:
                for key in (b'\x1b', b'\x1b', b'\x04'):
                    os.write(master, key); time.sleep(0.2)
                stage = 3
        child.wait(timeout=5)
        assert stage == 3 and child.returncode == 0, output.decode(errors='replace')
        assert not (home / 'sandboxes' / 'ended' / 'endpoint.json').exists()
    finally:
        if child.poll() is None:
            child.kill()
        child.wait()
        os.close(master)


def end_smoke(executable: Path, home: Path, environment: dict) -> None:
    """End through the packaged TUI after the HTTP owner closes immediately."""
    import pty
    import fcntl
    import struct
    import termios
    home.mkdir(parents=True)
    endpoint = home / 'sandboxes' / 'exit-check' / 'endpoint.json'
    program = '''
import json, sys, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from openrua.sessions.client import write_endpoint
from openrua.sessions.core import initial_state
home = Path(sys.argv[1])
state = initial_state(); state['connected'] = True
class Handler(BaseHTTPRequestHandler):
    def reply(self, value):
        raw = json.dumps(value).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(raw)))
        self.end_headers(); self.wfile.write(raw)
    def do_GET(self):
        self.reply([] if self.path.startswith('/api/events') else {'state': state, 'cursor': 0})
    def do_POST(self):
        assert self.path == '/api/end'
        state['closed'] = True
        with (home / 'end-calls').open('a') as stream: stream.write('end\\n')
        self.reply({'result': {'state': state, 'cursor': 0}})
        threading.Thread(target=self.server.shutdown).start()
    def log_message(self, *args): pass
server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
endpoint = home / 'sandboxes' / 'exit-check' / 'endpoint.json'
write_endpoint(endpoint, f'http://127.0.0.1:{server.server_port}', 'exit-check')
server.serve_forever(); server.server_close(); endpoint.unlink()
'''
    with (home / 'owner.log').open('w') as log:
        owner = subprocess.Popen([str(executable.parent / 'python'), '-u', '-c', program, str(home)],
                                 stdout=log, stderr=log, env=environment)
    child = None
    master, slave = pty.openpty()
    try:
        deadline = time.monotonic() + 10
        while not endpoint.exists():
            assert owner.poll() is None, (home / 'owner.log').read_text()
            assert time.monotonic() < deadline, 'test owner did not start'
            time.sleep(.02)
        fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 40, 120, 0, 0))
        child = subprocess.Popen([str(executable), '--home', str(home), 'chat', '--name', 'exit-check', '--tui'],
                                 stdin=slave, stdout=slave, stderr=slave, env=environment)
        os.close(slave); slave = None
        output, stage = bytearray(), 0
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline and child.poll() is None:
            if select.select([master], [], [], .1)[0]:
                try: output.extend(os.read(master, 65536))
                except OSError: break
            if stage == 0 and b'Enter Send' in output:
                os.write(master, b'/end\r'); stage = 1
            elif stage == 1 and b'End execution and release resources?' in output:
                os.write(master, b'\x1b[B\r'); stage = 2
        child.wait(timeout=5)
        while select.select([master], [], [], 0)[0]:
            try: output.extend(os.read(master, 65536))
            except OSError: break
        assert stage == 2 and child.returncode == 0, output.decode(errors='replace')
        assert b'leave it in the background' not in output, output.decode(errors='replace')
        assert b'Session exit-check ended' in output, output.decode(errors='replace')
        assert (home / 'end-calls').read_text() == 'end\n'
        assert owner.wait(timeout=5) == 0, (home / 'owner.log').read_text()
    finally:
        for process in (child, owner):
            if process is not None:
                if process.poll() is None: process.kill()
                process.wait()
        os.close(master)
        if slave is not None: os.close(slave)


def update_smoke(executable: Path, home: Path, environment: dict) -> None:
    """Exercise the packaged update page and retained logs with a local installer."""
    import pty
    home.mkdir(parents=True)
    program = home / 'entry.py'
    program.write_text('''
import sys, time
from pathlib import Path
if '--installer' in sys.argv:
    print('[update] Installing selected fixture version', flush=True)
    print('PACKAGED INSTALLER LOG', flush=True)
    time.sleep(.2)
    raise SystemExit(0)
from openrua.cli import main, updates
from openrua.cli.updates import Offer
home=Path(sys.argv[1])
updates.offers=lambda *a: [Offer('fixture', '1.0', '2.0', False,
    [sys.executable,__file__,'--installer'],False,'Fixture Agent','2.0')]
raise SystemExit(main(['--home',str(home),'--workspace',str(home/'project'),'--trust-workspace']))
''')
    master, slave = pty.openpty()
    child = subprocess.Popen([str(executable.parent / 'python'), str(program), str(home)],
                             stdin=slave, stdout=slave, stderr=slave, env=environment)
    os.close(slave)
    output, frame = bytearray(), bytearray()
    def wait(marker):
        deadline = time.monotonic() + 25
        while marker not in frame:
            if select.select([master], [], [], .05)[0]:
                try: data = os.read(master, 65536)
                except OSError: break
                output.extend(data); frame.extend(data)
            assert child.poll() is None, output.decode(errors='replace')
            assert time.monotonic() < deadline, output.decode(errors='replace')
        assert marker in frame, output.decode(errors='replace')
    def key(value):
        time.sleep(.2)
        while select.select([master], [], [], 0)[0]:
            try: output.extend(os.read(master, 65536))
            except OSError: break
        frame.clear(); os.write(master, value)
    try:
        wait(b'Update now'); key(b'1')
        wait(b'Update verified')
        assert b'PACKAGED INSTALLER LOG' not in output
        key(b'2'); wait(b'PACKAGED INSTALLER LOG')
        key(b'\x1b'); wait(b'Update verified')
        key(b'1'); wait(b'Choose your environment'); key(b'\x04')
        child.wait(timeout=5)
        while select.select([master], [], [], 0)[0]:
            try: output.extend(os.read(master, 65536))
            except OSError: break
        assert child.returncode == 0, output.decode(errors='replace')
        assert output.count(b'\x1b[?1049h') == output.count(b'\x1b[?1049l') == 1
        assert len(list((home / 'updates').glob('*/output.log'))) == 1
        assert not (home / 'sandboxes').exists()
    finally:
        if child.poll() is None: child.kill()
        child.wait(); os.close(master)


def check(dist: Path, version: str) -> None:
    wheels = list(dist.resolve().glob("*.whl"))
    if len(wheels) != 1:
        raise ValueError("provide a distribution directory containing exactly one wheel")
    with tempfile.TemporaryDirectory(prefix="openrua-install-") as directory:
        root = Path(directory)
        environment = root / "venv"
        venv.create(environment, with_pip=True)
        python = str(environment / "bin" / "python")
        def run(*args):
            subprocess.run([python, *args], cwd=root, check=True)
        run("-m", "pip", "install", str(wheels[0]))
        run("-c", "from openrua import __version__; import sys; assert __version__ == sys.argv[1]", version)
        for args in (("--version",), ("chat", "--help"), ("session", "--help"), ("update", "--help")):
            subprocess.run([str(environment / "bin" / "openrua"), *args], cwd=root, check=True)
        run("-c", """
from importlib.resources import files
from pathlib import Path
from openrua import __version__
from openrua.robot.sim.install import render
from openrua.web import assets
assert all(content for mime, content in assets().values())
dockerfile, _ = render({'python': '3.12'}, 'installed-release', Path.cwd())
assert f'--no-deps --refresh-package openrua openrua=={__version__}' in dockerfile
root = files('openrua')
for path in ('sandbox/workspace/README.md', 'configs/agents/codex.yaml',
             'configs/agents/kimi.yaml', 'configs/agents/zcode.yaml',
             'plugins/agents/kimi_server.py', 'terminal/assets/src/update.mjs',
             'configs/agents/claude-code.yaml', 'terminal/assets/src/workspace.mjs',
             'terminal/assets/src/exit.mjs', 'terminal/assets/src/project.mjs',
             'terminal/assets/src/schedules.mjs', 'cli/project_skills.py', 'cli/skill_tasks.py',
             'cli/skill_schedules.py', 'cli/skill_scheduler.py', 'sandbox/extend.Dockerfile'):
    assert root.joinpath(path).read_bytes(), path
""")
        run("-c", """
import os, shutil
from openrua.terminal.launcher import entrypoint, runtime
from importlib.metadata import requires
assert not any('textual' in dep.lower() for dep in requires('openrua'))
env = dict(os.environ, PATH=os.path.dirname(__import__('sys').executable))
assert shutil.which('node', path=env['PATH']) is None
assert runtime()([str(entrypoint()), '--help'], env=env, return_completed_process=True).returncode == 0
""")
        invalid_config_smoke(environment / 'bin' / 'openrua', root / 'invalid-home',
                             dict(os.environ, PATH=str(environment / 'bin')))
        terminal_smoke(environment / 'bin' / 'openrua', root / 'empty-home',
                       dict(os.environ, PATH=str(environment / 'bin')))
        for agent in ('claude-code', 'kimi', 'zcode'):
            terminal_smoke(environment / 'bin' / 'openrua', root / f'{agent}-home',
                           dict(os.environ, PATH=str(environment / 'bin')), agent=agent)
        manifest = root / 'external-agent.yaml'
        run('-c', "from importlib.resources import files; from pathlib import Path; import sys; "
            "Path(sys.argv[1]).write_bytes(files('openrua').joinpath('configs/agents/codex.yaml').read_bytes())", str(manifest))
        terminal_smoke(environment / 'bin' / 'openrua', root / 'plugin-home',
                       dict(os.environ, PATH=str(environment / 'bin')), manifest)
        update_smoke(environment / 'bin' / 'openrua', root / 'update-home',
                     dict(os.environ, PATH=str(environment / 'bin')))
        archive_smoke(environment / 'bin' / 'openrua', root / 'archive-home',
                      dict(os.environ, PATH=str(environment / 'bin')))
        end_smoke(environment / 'bin' / 'openrua', root / 'exit-home',
                  dict(os.environ, PATH=str(environment / 'bin')))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dist", type=Path, required=True)
    parser.add_argument("--version", required=True)
    args = parser.parse_args()
    check(args.dist, args.version)


if __name__ == "__main__":
    main()
