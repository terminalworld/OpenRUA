"""Real OpenRUA HTTP/SQLite owner with controlled native frames, for UI tests."""
import asyncio
import json
import sys
import tempfile
from pathlib import Path

from openrua.sessions.client import write_endpoint
from tests.sessions.test_http import running_server, stop


async def main():
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary)
        store, session, native, execution, server = await running_server(path)
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
