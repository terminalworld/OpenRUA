"""Real OpenRUA HTTP/SQLite owner with controlled native frames, for UI tests."""
import asyncio
import json
import sys
import tempfile
from pathlib import Path

from openrua.sessions.client import write_endpoint
from openrua.artifacts import WorkspaceFiles
from tests.sessions.test_http import running_server, stop


async def main():
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary)
        workspace = path / 'workspace'
        workspace.mkdir()
        (workspace / 'notes #1.txt').write_text('measurement\n' + 'line\n' * 80 + '\x1b]52;c;payload\x07')
        import base64
        (workspace / 'camera.png').write_bytes(base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWZkAAAAASUVORK5CYII='))
        (workspace / 'blocked').symlink_to(path / 'endpoint.json')
        store, session, native, execution, server = await running_server(path, artifacts=WorkspaceFiles(workspace))
        endpoint = path / "endpoint.json"
        write_endpoint(endpoint, server.url, server.token)
        print(json.dumps({"endpoint": str(endpoint)}), flush=True)
        try:
            while line := await asyncio.to_thread(sys.stdin.readline):
                request = json.loads(line)
                if request["op"] == "frame":
                    await native.frames.put(request["frame"])
                elif request["op"] == "writes":
                    print(json.dumps({"writes": native.writes}), flush=True)
                    continue
                elif request["op"] == "stop":
                    break
                print(json.dumps({"ok": True}), flush=True)
        finally:
            await stop(store, execution, server)


if __name__ == "__main__":
    asyncio.run(main())
