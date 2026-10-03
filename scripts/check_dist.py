"""Install a built wheel in isolation and check its public entry points and assets."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import tempfile
import venv


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
             'configs/agents/claude-code.yaml', 'tui/chat.tcss'):
    assert root.joinpath(path).read_bytes(), path
""")
        run("-m", "pip", "install", f"{wheels[0]}[tui]")
        run("-c", """
import asyncio
from openrua.tui.app import ChatApp
from openrua.sessions.client import Client
async def check():
    app = ChatApp(Client('http://127.0.0.1:1', 'unused', timeout=0.1))
    async with app.run_test() as pilot:
        await pilot.pause()
        assert app.query_one('#message')
asyncio.run(check())
""")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dist", type=Path, required=True)
    parser.add_argument("--version", required=True)
    args = parser.parse_args()
    check(args.dist, args.version)


if __name__ == "__main__":
    main()
