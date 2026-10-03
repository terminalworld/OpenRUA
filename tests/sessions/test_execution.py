"""Execution ownership and native side-effect ordering using a fake transport."""

import asyncio

import pytest

from openrua.agents.conversation import Conversation, ConversationProtocol, Event, Update
from openrua.sessions.core import Session, initial_state
from openrua.sessions.execution import Execution
from openrua.sessions.store import SQLiteStore


class FakeProtocol(ConversationProtocol):
    def __init__(self):
        self.ready, self.active = False, None
    @property
    def can_submit(self):
        return self.ready and self.active is None
    def begin(self):
        return Update([{"begin": True}])
    def submit(self, turn_id, text):
        assert self.can_submit
        self.active = turn_id
        return Update([{"submit": turn_id, "text": text}])
    def interrupt(self, turn_id):
        assert turn_id == self.active
        return Update([{"interrupt": turn_id}])
    def receive(self, frame):
        if frame["kind"] == "ready":
            self.ready = True
        if frame["kind"] == "turn_finished":
            self.active = None
        return Update(events=[Event(frame["kind"], frame.get("turn_id"), frame.get("data", {}))])
    def respond(self, request_id, answers):
        return Update([{"reply": request_id, "answers": answers}])


class FakeTransport:
    def __init__(self, session):
        self.session = session
        self.frames = asyncio.Queue()
        self.writes = []
        self.fail_send = False
    async def read(self):
        return await self.frames.get()
    async def write(self, frame):
        if "submit" in frame:
            state = self.session.store.snapshot()["state"]
            assert state["active"] == frame["submit"]
            assert next(m for m in state["messages"] if m["id"] == frame["submit"])["status"] == "dispatching"
            if self.fail_send:
                raise OSError("lost write acknowledgement")
        self.writes.append(frame)
        if "begin" in frame:
            await self.frames.put({"kind": "ready", "data": {"session_id": "native"}})
    async def close(self):
        pass


async def until(predicate):
    async def poll():
        while not predicate():
            await asyncio.sleep(0.001)
    await asyncio.wait_for(poll(), 2)


async def setup(tmp_path):
    store = SQLiteStore(tmp_path / "session.sqlite", initial_state())
    session = Session(store)
    transport = FakeTransport(session)
    async def start_transport():
        return transport
    execution = Execution(session, Conversation(["fake"], FakeProtocol()), start_transport, tmp_path / "owner.lock")
    await execution.start()
    await until(lambda: store.snapshot()["state"]["connected"])
    return store, session, transport, execution


def test_multiple_clients_queue_while_native_execution_continues(tmp_path):
    async def run():
        store, session, transport, execution = await setup(tmp_path)
        try:
            first = await execution.command("enqueue", client_id="cli", request_id="1", text="inspect")
            second = await execution.command("enqueue", client_id="web", request_id="1", text="grasp")
            assert [f["text"] for f in transport.writes if "submit" in f] == ["inspect"]
            # No client is attached here; the independent reader continues.
            await transport.frames.put({"kind": "turn_finished", "turn_id": first["id"],
                                        "data": {"status": "completed"}})
            await until(lambda: len([f for f in transport.writes if "submit" in f]) == 2)
            assert store.snapshot()["state"]["active"] == second["id"]
        finally:
            await execution.close()
            store.close()
    asyncio.run(run())


def test_lost_submission_is_not_retried(tmp_path):
    async def run():
        store, session, transport, execution = await setup(tmp_path)
        try:
            transport.fail_send = True
            accepted = await execution.command("enqueue", client_id="web", request_id="1", text="grasp")
            assert store.snapshot()["state"]["messages"][0]["status"] == "unknown"
            retry = await execution.command("enqueue", client_id="web", request_id="1", text="grasp")
            assert accepted["id"] == retry["id"]
            assert not [f for f in transport.writes if "submit" in f]
        finally:
            await execution.close()
            store.close()
    asyncio.run(run())


def test_interrupt_waits_for_native_start_and_keeps_next_message_paused(tmp_path):
    async def run():
        store, session, transport, execution = await setup(tmp_path)
        try:
            first = await execution.command("enqueue", client_id="web", request_id="1", text="grasp")
            await execution.command("enqueue", client_id="cli", request_id="2", text="place")
            await execution.command("interrupt", message_id=first["id"])
            assert not [f for f in transport.writes if "interrupt" in f]
            await transport.frames.put({"kind": "turn_started", "turn_id": first["id"]})
            await until(lambda: any("interrupt" in f for f in transport.writes))
            await transport.frames.put({"kind": "turn_finished", "turn_id": first["id"],
                                        "data": {"status": "interrupted"}})
            await until(lambda: store.snapshot()["state"]["active"] is None)
            assert len([f for f in transport.writes if "submit" in f]) == 1
            await execution.command("resume", pause_id=store.snapshot()["state"]["pause_id"])
            assert len([f for f in transport.writes if "submit" in f]) == 2
        finally:
            await execution.close()
            store.close()
    asyncio.run(run())


def test_second_owner_is_rejected_before_starting_native_process(tmp_path):
    async def run():
        store, session, transport, execution = await setup(tmp_path)
        async def forbidden():
            pytest.fail("second owner must not start a native process")
        second = Execution(session, Conversation(["fake"], FakeProtocol()), forbidden, tmp_path / "owner.lock")
        try:
            with pytest.raises(RuntimeError, match="already has an execution owner"):
                await second.start()
        finally:
            await execution.close()
            store.close()
    asyncio.run(run())


def test_failed_transport_cleanup_keeps_owner_lock_until_retry(tmp_path):
    async def run():
        store, session, transport, execution = await setup(tmp_path)
        failed = [True]
        async def close():
            if failed[0]:
                raise OSError("transport cleanup failed")
        transport.close = close
        async def replacement():
            return FakeTransport(session)
        next_owner = Execution(session, Conversation(["fake"], FakeProtocol()),
                               replacement, tmp_path / "owner.lock")
        try:
            await execution.command("enqueue", client_id="cli", request_id="a", text="inspect")
            with pytest.raises(OSError, match="cleanup"):
                await execution.close()
            state = store.snapshot()["state"]
            assert state["paused"] and not state["connected"]
            assert state["messages"][0]["status"] == "unknown"
            with pytest.raises(RuntimeError, match="already has an execution owner"):
                await next_owner.start()
            with pytest.raises(RuntimeError, match="not accepting"):
                await execution.command("enqueue", client_id="web", request_id="b", text="grasp")
            failed[0] = False
            await execution.close()
            await next_owner.start()
        finally:
            failed[0] = False
            await execution.close()
            await next_owner.close()
            store.close()
    asyncio.run(run())


def test_stdio_transport_starts_in_explicit_workspace(tmp_path):
    import sys
    from openrua.sessions.execution import StdioTransport
    async def run():
        transport = await StdioTransport.start(
            [sys.executable, "-c", "import os,json; print(json.dumps({'cwd':os.getcwd()}))"],
            tmp_path / "stderr.log", cwd=tmp_path)
        try:
            frame = await transport.read()
            assert frame == {"cwd": str(tmp_path.resolve())}
            assert await transport.read() is None
        finally:
            await transport.close()
    asyncio.run(run())
