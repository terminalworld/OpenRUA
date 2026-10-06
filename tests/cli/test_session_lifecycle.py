"""Exercise the shared lifecycle with HTTP, SQLite and a controlled child process.

The child is a protocol fixture, not a model or a physical robot.
"""

import asyncio
import sys
from types import SimpleNamespace

from openrua.agents.conversation import Conversation
from openrua.cli import build_parser
from openrua.cli.commands import clean, down, serve
from openrua.runner import history, live_state, managed
from openrua.sessions.client import Client
from tests.sessions.test_execution import FakeProtocol, until


def test_chat_reconnect_end_browse_and_explicit_delete(tmp_path, monkeypatch):
    async def run():
        name = "lifecycle"
        directory = tmp_path / "sandboxes" / name
        workspace = tmp_path / "external-workspace"
        workspace.mkdir()
        program = tmp_path / "native.py"
        program.write_text('''
import json, sys, time
from pathlib import Path
workspace = Path(sys.argv[1])
def emit(kind, turn=None, **data):
    print(json.dumps(dict(kind=kind, turn_id=turn, data=data)), flush=True)
for line in sys.stdin:
    frame = json.loads(line)
    if "begin" in frame:
        emit("ready", session_id="fixture-native-session")
        continue
    turn = frame["submit"]
    emit("turn_started", turn)
    if frame["text"] == "inspect":
        deadline = time.monotonic() + 10
        while not (workspace / "release").exists():
            if time.monotonic() > deadline:
                raise RuntimeError("test did not release the first turn")
            time.sleep(0.01)
    with (workspace / "observations.txt").open("a") as stream:
        stream.write(frame["text"] + "\\n")
    emit("text_delta", turn, text=frame["text"], item_id=turn)
    emit("turn_finished", turn, status="completed")
''')
        live_state.save(name, tmp_path, status="running", backend="sim", agent="fixture",
                        sandbox="fixture", model="fixture", proxy="", options={},
                        workspace=str(workspace))
        stops = []

        def stop_robot():
            stops.append(True)
            live_state.stopped(name, tmp_path)

        monkeypatch.setattr(managed, "open_robot", lambda request:
                            SimpleNamespace(power_off=stop_robot))
        monkeypatch.setattr(managed.agents, "get", lambda *a, **kw: SimpleNamespace(
            conversation=lambda *a, **kw: Conversation(
                [sys.executable, "-u", str(program), str(workspace)], FakeProtocol())))
        # There are no Docker resources in this fixture; native process cleanup
        # still goes through the real StdioTransport and ManagedSession.
        monkeypatch.setattr(clean, "running", lambda _: False)
        args = build_parser().parse_args(["--home", str(tmp_path), "serve", "--name", name])
        serving = asyncio.create_task(serve.serve(args))
        endpoint = directory / "endpoint.json"
        try:
            await until(endpoint.exists)
            first = Client.from_file(endpoint)
            second = Client.from_file(endpoint)
            await asyncio.to_thread(first.command, "enqueue", client_id="terminal",
                                    request_id="1", text="inspect")
            queued = await asyncio.to_thread(second.command, "enqueue", client_id="browser",
                                             request_id="1", text="continue")
            assert queued["status"] == "queued"
            cursor = (await asyncio.to_thread(second.snapshot))["cursor"]
            del first, second
            (workspace / "release").touch()

            reconnected, found = await asyncio.to_thread(history.open_session, tmp_path, name)
            assert found == name and not getattr(reconnected, "read_only", False)
            async def finished():
                while True:
                    state = (await asyncio.to_thread(reconnected.snapshot))["state"]
                    if all(m["status"] == "completed" for m in state["messages"]):
                        return
                    await asyncio.sleep(0.01)
            await asyncio.wait_for(finished(), 5)
            assert await asyncio.to_thread(reconnected.events, cursor)
            assert (await asyncio.to_thread(reconnected.workspace_read,
                                           "observations.txt"))["text"] == "inspect\ncontinue\n"
            end_args = build_parser().parse_args(["--home", str(tmp_path), "down", "--name", name])
            assert await asyncio.to_thread(down.run, end_args) == 0
            await asyncio.wait_for(serving, 5)
            assert stops == [True] and not endpoint.exists()

            archived, _ = history.open_session(tmp_path, name)
            assert archived.read_only and archived.snapshot()["state"]["closed"]
            assert len(archived.snapshot()["state"]["messages"]) == 2
            assert archived.artifacts.read("observations.txt")["text"] == "inspect\ncontinue\n"
            clean_args = build_parser().parse_args(["--home", str(tmp_path), "clean", "--name", name])
            assert clean.run(clean_args) == 0
            assert not directory.exists()
            assert (workspace / "observations.txt").read_text() == "inspect\ncontinue\n"
        finally:
            if not serving.done():
                serving.cancel()
            await asyncio.gather(serving, return_exceptions=True)
    asyncio.run(run())
