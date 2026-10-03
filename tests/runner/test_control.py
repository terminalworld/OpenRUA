"""Native resource control reuses HTTP without creating an agent conversation."""

import asyncio

import pytest

from openrua.runner.control import serve_control
from openrua.sessions.client import Client


def test_resource_control_retries_shutdown_without_exposing_a_message_queue(tmp_path):
    async def run():
        failed, calls = [True], []
        def stop():
            calls.append("stop")
            if failed[0]:
                raise RuntimeError("driver still running")
        async with serve_control(stop, tmp_path) as (control, server):
            endpoint = tmp_path / "control.json"
            client = Client.from_file(endpoint)
            with pytest.raises(RuntimeError, match="resources only"):
                await asyncio.to_thread(client.command, "enqueue", client_id="a", request_id="b", text="move")
            with pytest.raises(RuntimeError, match="driver still running"):
                await asyncio.to_thread(client.end)
            assert not control.closed and not server.ended.is_set()
            assert endpoint.exists()
            failed[0] = False
            await asyncio.to_thread(client.end)
            await asyncio.wait_for(server.ended.wait(), 2)
            assert (await asyncio.to_thread(client.snapshot))["state"]["closed"]
        assert calls == ["stop", "stop"]
        assert not endpoint.exists()
    asyncio.run(run())
