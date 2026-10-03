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


def test_native_cleanup_error_still_attempts_robot_shutdown(tmp_path):
    async def run():
        store, session, native, execution = await setup(tmp_path)
        calls = []
        async def fail_close():
            raise RuntimeError("pipe cleanup failed")
        native.close = fail_close
        owner = ManagedSession(SimpleNamespace(power_off=lambda: calls.append("robot stopped")), execution, store)
        try:
            with pytest.raises(UnavailableError, match="pipe cleanup failed"):
                await owner.end()
            assert calls == ["robot stopped"]
            assert store.snapshot()["state"]["closed"]
        finally:
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
