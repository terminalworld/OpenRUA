"""Install a built wheel in isolation and check its public entry points and assets."""

from __future__ import annotations

import argparse
import os
import select
import time
from pathlib import Path
import subprocess
import tempfile
import venv


def terminal_smoke(executable: Path, home: Path, environment: dict) -> None:
    """Exercise real TTY startup and detachment without starting robot resources."""
    import pty
    master, slave = pty.openpty()
    child = subprocess.Popen([str(executable), '--home', str(home)],
                             stdin=slave, stdout=slave, stderr=slave, env=environment)
    os.close(slave)
    output = bytearray()
    sent = False
    try:
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and child.poll() is None:
            if select.select([master], [], [], 0.1)[0]:
                try:
                    output.extend(os.read(master, 65536))
                except OSError:
                    break
            if not sent and b'Configure a new robot conversation' in output:
                os.write(master, b'\x04')
                sent = True
        if child.poll() is None:
            child.wait(timeout=5)
        assert sent and child.returncode == 0, output.decode(errors='replace')
        assert not (home / 'sandboxes').exists(), 'Setup cancellation started resources'
    finally:
        if child.poll() is None:
            child.kill()
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
        for args in (("--version",), ("chat", "--help"), ("session", "--help")):
            subprocess.run([str(environment / "bin" / "openrua"), *args], cwd=root, check=True)
        run("-c", """
from importlib.resources import files
from openrua.web import assets
assert all(content for mime, content in assets().values())
root = files('openrua')
for path in ('sandbox/workspace/README.md', 'configs/agents/codex.yaml',
             'configs/agents/claude-code.yaml', 'terminal/assets/src/workspace.mjs'):
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
        terminal_smoke(environment / 'bin' / 'openrua', root / 'empty-home',
                       dict(os.environ, PATH=str(environment / 'bin')))
        archive_smoke(environment / 'bin' / 'openrua', root / 'archive-home',
                      dict(os.environ, PATH=str(environment / 'bin')))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dist", type=Path, required=True)
    parser.add_argument("--version", required=True)
    args = parser.parse_args()
    check(args.dist, args.version)


if __name__ == "__main__":
    main()
