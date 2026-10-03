"""Run a local shared session independently of its connected clients."""

from __future__ import annotations

import asyncio
import secrets
import signal

from openrua.cli.commands import up
from openrua.web import assets
from openrua.config import paths
from openrua.errors import UnavailableError
from openrua.runner import managed
from openrua.sessions.client import write_endpoint
from openrua.sessions.http import LocalServer


async def serve(args) -> None:
    owned = await managed.start(up.robot_request(args))
    server = None
    published = False
    endpoint = paths.sandbox_dir(args.name, args.home) / "endpoint.json"
    loop = asyncio.get_running_loop()
    stopped = asyncio.Event()
    signals = []
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, stopped.set)
            signals.append(sig)
        server = LocalServer(owned.execution, owned.end, secrets.token_urlsafe(32), port=args.port, assets=assets())
        write_endpoint(endpoint, server.url, server.token)
        published = True
        server.start()
        print(f"[serve] {args.name} at {server.url}\n"
              f"[serve] open another terminal: openrua --home {args.home} chat --name {args.name}\n"
              f"[serve] browser: openrua --home {args.home} session --name {args.name} web\n"
              "[serve] closing a chat client keeps this session running; Ctrl-C here ends it", flush=True)
        while True:
            ending = [asyncio.create_task(stopped.wait()), asyncio.create_task(server.ended.wait())]
            try:
                await asyncio.wait(ending, return_when=asyncio.FIRST_COMPLETED)
            finally:
                for task in ending:
                    task.cancel()
                await asyncio.gather(*ending, return_exceptions=True)
            if server.ended.is_set():
                break
            try:
                await owned.end()
                break
            except Exception as exc:
                print(f"[serve] shutdown incomplete: {exc}; service kept open for retry", flush=True)
                stopped.clear()
    finally:
        for sig in signals:
            loop.remove_signal_handler(sig)
        try:
            if server:
                await server.close()
        finally:
            try:
                await owned.end()
            finally:
                owned.store.close()
                if published:
                    endpoint.unlink(missing_ok=True)


def run(args) -> int:
    try:
        asyncio.run(serve(args))
    except (OSError, RuntimeError, ValueError) as exc:
        raise UnavailableError(str(exc), hint="inspect the retained session logs; use a new --name to start again") from exc
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("serve", help="own a robot and shared native conversation for local clients",
                       description=__doc__)
    p.add_argument("robot", nargs="?", default=None)
    up.add_options(p)
    p.add_argument("--model", default=None, help="model (default: the configured agent's)")
    p.add_argument("--port", type=int, default=0, help="local port (default: choose a free port)")
    p.set_defaults(fn=run)
