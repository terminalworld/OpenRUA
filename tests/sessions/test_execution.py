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
    def configure(self, setting, value):
        return Update([{"configure": setting, "value": value}],
                      [Event("setting", data={"setting": setting, "value": value})])
    def select_model(self, model, effort):
        return Update([{'select_model': model, 'effort': effort}],
                      [Event('setting', data={'setting': 'model', 'value': model, 'effective_from': 'next_turn'}),
                       Event('setting', data={'setting': 'effort', 'value': effort, 'effective_from': 'next_turn'})])


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


def test_interrupt_waits_for_native_start_then_the_next_message_goes(tmp_path):
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
            # the queue goes on: the next message is submitted without a resume
            await until(lambda: len([f for f in transport.writes if "submit" in f]) == 2)
            assert not store.snapshot()["state"]["paused"]
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


def test_stdio_transport_ignores_blank_lines_without_losing_native_frames(tmp_path):
    import sys
    from openrua.sessions.execution import StdioTransport
    async def run():
        transport = await StdioTransport.start(
            [sys.executable, '-c', "print('  \\n\\r\\n{\"ready\": true}')"], tmp_path / 'agent.stderr.log')
        try:
            assert await transport.read() == {'ready': True}
            assert await transport.read() is None
        finally:
            await transport.close()
    asyncio.run(run())


@pytest.mark.parametrize('line', ['startup banner', '[1, 2]', '{broken json'])
def test_stdio_transport_retains_invalid_output_and_does_not_skip_it(tmp_path, line):
    import sys
    from openrua.sessions.execution import StdioTransport
    async def run():
        transport = await StdioTransport.start(
            [sys.executable, '-c', 'import sys; print(sys.argv[1]); print(\'{"ready":true}\')', line],
            tmp_path / 'agent.stderr.log')
        try:
            with pytest.raises(ValueError, match='cannot reconnect') as caught:
                await transport.read()
            saved = tmp_path / 'agent.stderr.invalid-output.log'
            assert str(saved) in str(caught.value)
            assert saved.read_text() == line + '\n'
            assert saved.stat().st_mode & 0o777 == 0o600
        finally:
            await transport.close()
    asyncio.run(run())


def test_a_native_process_that_ends_says_why(tmp_path):
    # The agent CLI exited on an unknown flag; the session showed only
    # "agent disconnected", and the reason sat in agent.stderr.log.
    async def run():
        store = SQLiteStore(tmp_path / "session.sqlite", initial_state())
        session = Session(store)
        transport = FakeTransport(session)
        async def start_transport():
            return transport
        execution = Execution(session, Conversation(["fake"], FakeProtocol()), start_transport,
                              tmp_path / "owner.lock",
                              diagnostics=lambda: "error: unknown option '--autocompact'")
        await execution.start()
        try:
            await until(lambda: store.snapshot()["state"]["connected"])
            await transport.frames.put(None)
            await until(lambda: not store.snapshot()["state"]["connected"])
            assert "unknown option '--autocompact'" in store.snapshot()["state"]["pause_reason"]
        finally:
            await execution.close()
            store.close()
    asyncio.run(run())


def test_configure_reaches_the_agent_at_once(tmp_path):
    async def run():
        store, session, transport, execution = await setup(tmp_path)
        await execution.command("configure", setting="permissions", value="ask")
        assert {"configure": "permissions", "value": "ask"} in transport.writes
        assert store.snapshot()["state"]["settings"] == {"permissions": "ask"}
        await execution.close()
    asyncio.run(run())


def test_model_and_effort_selection_is_complete_before_the_next_dispatch(tmp_path):
    async def run():
        store, session, transport, execution = await setup(tmp_path)
        try:
            await execution.command('select_model', model='selected-model', effort='medium')
            message = await execution.command('enqueue', client_id='person', request_id='next', text='Inspect')
            snapshot = store.snapshot()['state']
            assert snapshot['settings'] == {'model': 'selected-model', 'effort': 'medium'}
            assert snapshot['messages'][0]['turn_settings'] == snapshot['settings']
            selection, dispatched = [i for i, frame in enumerate(transport.writes)
                                     if 'select_model' in frame or frame.get('submit') == message['id']]
            assert selection < dispatched
        finally:
            await execution.close()
            store.close()
    asyncio.run(run())


def test_an_agent_without_settings_says_so(tmp_path):
    class Fixed(FakeProtocol):
        configure = ConversationProtocol.configure
    async def run():
        store = SQLiteStore(tmp_path / "session.sqlite", initial_state())
        session = Session(store)
        transport = FakeTransport(session)
        async def start_transport():
            return transport
        execution = Execution(session, Conversation(["fake"], Fixed()), start_transport,
                              tmp_path / "owner.lock")
        await execution.start()
        await until(lambda: store.snapshot()["state"]["connected"])
        with pytest.raises(NotImplementedError):
            await execution.command("configure", setting="model", value="m")
        await execution.close()
    asyncio.run(run())


def test_planned_handover_still_accepts_new_dialogue_without_sending_to_closed_native(tmp_path):
    async def run():
        store, session, transport, execution = await setup(tmp_path)
        try:
            await execution.close(handover='tool environment update')
            message = await execution.command('enqueue', client_id='person', request_id='during', text='inspect this connection')
            state = store.snapshot()['state']
            assert state['messages'][0]['id'] == message['id']
            assert state['messages'][0]['status'] == 'queued' and not state['closed']
            assert not [f for f in transport.writes if 'submit' in f]
            session.close()
            with pytest.raises(ValueError, match='closed'):
                await execution.command('enqueue', client_id='person', request_id='late', text='too late')
        finally:
            store.close()
    asyncio.run(run())


def test_late_active_turn_refuses_planned_handover_without_closing_agent(tmp_path):
    async def run():
        store, session, transport, execution = await setup(tmp_path)
        try:
            first = await execution.command('enqueue', client_id='person', request_id='first', text='inspect')
            with pytest.raises(ValueError, match='turn is running'):
                await execution.close(handover='update')
            assert store.snapshot()['state']['connected']
            assert not execution._closing
            assert not execution._handover
            await execution.command('interrupt', message_id=first['id'])
        finally:
            await execution.close()
            store.close()
    asyncio.run(run())
