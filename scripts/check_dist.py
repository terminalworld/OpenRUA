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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dist", type=Path, required=True)
    parser.add_argument("--version", required=True)
    args = parser.parse_args()
    check(args.dist, args.version)


if __name__ == "__main__":
    main()
