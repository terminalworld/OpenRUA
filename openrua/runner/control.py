"""Let another client end resources through the process that owns their handles.

Native terminal sessions need resource control without a managed conversation.
The existing local HTTP transport handles authentication and request lifetime.
"""

from __future__ import annotations

import asyncio
import secrets
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Callable

from openrua.sessions.client import write_endpoint
from openrua.sessions.http import LocalServer


class ResourceControl:
    def __init__(self, stop: Callable[[], None]):
        self._stop = stop
        self._gate = asyncio.Lock()
        self.closed = False

    def snapshot(self) -> dict:
        return {"state": {"closed": self.closed, "mode": "native-terminal"}}

    async def end(self) -> None:
        async with self._gate:
            if not self.closed:
                await asyncio.to_thread(self._stop)
                self.closed = True


@asynccontextmanager
async def serve_control(stop: Callable[[], None], directory: Path):
    """Publish a resource-only endpoint and retain all other session files.

    The caller supplies an already owned stop operation. A failed stop leaves
    the endpoint record for inspection; it never implies a successful shutdown.
    """
    control = ResourceControl(stop)
    server = None
    endpoint = directory / "control.json"
    published = False
    try:
        server = LocalServer(None, control.end, secrets.token_urlsafe(32), snapshot=control.snapshot)
        write_endpoint(endpoint, server.url, server.token)
        published = True
        server.start()
        yield control, server
    finally:
        try:
            if server:
                await server.close()
        finally:
            await control.end()
            if published:
                endpoint.unlink(missing_ok=True)
