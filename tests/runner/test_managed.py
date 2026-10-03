"""An explicit session end stops all owned resources before closing records."""

import asyncio
from types import SimpleNamespace

import pytest

from openrua.errors import UnavailableError
from openrua.runner.managed import ManagedSession
from tests.sessions.test_execution import setup


def test_end_failure_keeps_records_open_and_allows_retry(tmp_path):
    async def run():
        store, session, native, execution = await setup(tmp_path)
        failed = [True]
        calls = []
        def stop_robot():
            calls.append("stop")
            if failed[0]:
                raise RuntimeError("driver did not stop")
        owner = ManagedSession(SimpleNamespace(power_off=stop_robot), execution, store)
        try:
            await execution.command("enqueue", client_id="cli", request_id="a", text="grasp")
            await execution.command("enqueue", client_id="web", request_id="b", text="place")
            with pytest.raises(UnavailableError, match="driver did not stop"):
                await owner.end()
            state = store.snapshot()["state"]
            assert not state["closed"] and state["paused"]
            assert [m["status"] for m in state["messages"]] == ["unknown", "queued"]
            failed[0] = False
            await owner.end()
            await owner.end()
            assert calls == ["stop", "stop"]
            assert store.snapshot()["state"]["closed"]
        finally:
            store.close()
    asyncio.run(run())


def test_native_cleanup_error_retains_open_paused_session_until_retry(tmp_path):
    async def run():
        store, session, native, execution = await setup(tmp_path)
        failed, calls = [True], []
        async def close_native():
            calls.append("native")
            if failed[0]:
                raise RuntimeError("pipe cleanup failed")
        native.close = close_native
        owner = ManagedSession(SimpleNamespace(power_off=lambda: calls.append("robot")), execution, store)
        try:
            await execution.command("enqueue", client_id="cli", request_id="a", text="inspect")
            await execution.command("enqueue", client_id="web", request_id="b", text="grasp")
            with pytest.raises(UnavailableError, match="pipe cleanup failed"):
                await owner.end()
            state = store.snapshot()["state"]
            assert calls == ["native", "robot"]
            assert not state["closed"] and state["paused"] and not state["connected"]
            assert [m["status"] for m in state["messages"]] == ["unknown", "queued"]
            failed[0] = False
            await owner.end()
            await owner.end()
            assert calls == ["native", "robot", "native"]
            assert store.snapshot()["state"]["closed"]
        finally:
            failed[0] = False
            await execution.close()
            store.close()
    asyncio.run(run())


def test_start_keeps_custom_plugin_spec_and_native_configuration(tmp_path, monkeypatch):
    from openrua.agents.conversation import Conversation
    from openrua.runner import managed, live_state
    from openrua.runner.live import RobotRequest
    from tests.sessions.test_execution import FakeProtocol, until
    async def run():
        calls = []
        directory = tmp_path / "sandboxes/custom"
        workspace = tmp_path / "custom-workspace"
        workspace.mkdir()
        (workspace / "README.md").write_text("Robot documentation")
        live_state.save("custom", tmp_path, agent="display-name", agent_spec="/plugins/custom.yaml",
                        workspace=str(workspace), agent_version="1.2", sandbox="box", model="model", proxy="proxy", options={"effort": "high"})
        monkeypatch.setattr(managed, "open_robot", lambda request: SimpleNamespace(power_off=lambda: calls.append("stopped")))
        def conversation(*args, **kwargs):
            assert args == ("box", "model", "proxy")
            assert kwargs == {"options": {"effort": "high"}}
            return Conversation(["native-agent"], FakeProtocol())
        def get(spec, home, version=None):
            assert (spec, home, version) == ("/plugins/custom.yaml", tmp_path, "1.2")
            return SimpleNamespace(conversation=conversation)
        monkeypatch.setattr(managed.agents, "get", get)
        class Transport:
            def __init__(self):
                self.frames = asyncio.Queue()
            async def write(self, frame):
                await self.frames.put({"kind": "ready", "data": {"session_id": "native"}})
            async def read(self):
                return await self.frames.get()
            async def close(self):
                pass
        async def start_transport(argv, stderr_path):
            assert argv == ["native-agent"]
            assert stderr_path == directory / "agent.stderr.log"
            return Transport()
        monkeypatch.setattr(managed.StdioTransport, "start", start_transport)
        owner = await managed.start(RobotRequest(home=tmp_path, name="custom"))
        try:
            await until(lambda: owner.store.snapshot()["state"]["connected"])
            assert owner.artifacts.read("README.md")["text"] == "Robot documentation"
            await owner.end()
            assert calls == ["stopped"]
        finally:
            owner.store.close()
    asyncio.run(run())


def test_http_end_retries_a_live_native_process_after_cleanup_failure(tmp_path):
    import sys
    from openrua.agents.conversation import Conversation
    from openrua.sessions.client import Client
    from openrua.sessions.core import Session, initial_state
    from openrua.sessions.execution import Execution, StdioTransport
    from openrua.sessions.http import LocalServer
    from openrua.sessions.store import SQLiteStore
    from tests.sessions.test_execution import FakeProtocol, until

    async def run():
        # A real child process stands in for the native CLI. It only exchanges
        # lifecycle frames and exits when its owner closes stdin.
        program = """
import json, sys
for line in sys.stdin:
    frame = json.loads(line)
    if 'begin' in frame:
        reply = {'kind': 'ready', 'data': {'session_id': 'native-child'}}
    else:
        reply = {'kind': 'turn_started', 'turn_id': frame['submit']}
    print(json.dumps(reply), flush=True)
"""
        store = SQLiteStore(tmp_path / "conversation.sqlite", initial_state())
        session = Session(store)
        transport = await StdioTransport.start([sys.executable, "-u", "-c", program], tmp_path / "native.log")
        original_close = transport.close
        failed, stops = [True], []
        async def fail_once():
            if failed[0]:
                raise OSError("native cleanup unavailable")
            await original_close()
        transport.close = fail_once
        async def factory():
            return transport
        execution = Execution(session, Conversation([], FakeProtocol()), factory, tmp_path / "owner.lock")
        owner = ManagedSession(SimpleNamespace(power_off=lambda: stops.append("robot")), execution, store)
        server = None
        try:
            await execution.start()
            await until(lambda: store.snapshot()["state"]["connected"])
            server = LocalServer(execution, owner.end, "test-token")
            server.start()
            client = Client(server.url, server.token)
            await asyncio.to_thread(client.command, "enqueue", client_id="cli", request_id="1", text="inspect")
            await asyncio.to_thread(client.command, "enqueue", client_id="web", request_id="1", text="grasp")
            with pytest.raises(RuntimeError, match="native cleanup unavailable"):
                await asyncio.to_thread(client.request, "/api/end", {})
            snapshot = await asyncio.to_thread(Client(server.url, server.token).snapshot)
            assert not snapshot["state"]["closed"] and snapshot["state"]["paused"]
            assert [m["status"] for m in snapshot["state"]["messages"]] == ["unknown", "queued"]
            assert transport.process.returncode is None
            assert not server.ended.is_set()
            with pytest.raises(RuntimeError, match="not accepting commands"):
                await asyncio.to_thread(client.command, "enqueue", client_id="web", request_id="2", text="place")
            failed[0] = False
            result = await asyncio.to_thread(client.request, "/api/end", {})
            assert result["result"]["state"]["closed"]
            await asyncio.wait_for(server.ended.wait(), 2)
            assert transport.process.returncode == 0
            assert stops == ["robot"]
        finally:
            failed[0] = False
            if server:
                await server.close()
            await owner.end()
            store.close()
    asyncio.run(run())
