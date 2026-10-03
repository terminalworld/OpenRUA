"""Existing commands cannot bypass or delete a managed conversation."""

import json

import pytest

from openrua.cli import build_parser
from openrua.cli.commands import agent, clean, down
from openrua.errors import UnavailableError


def endpoint(tmp_path):
    directory = tmp_path / "sandboxes/managed"
    directory.mkdir(parents=True)
    path = directory / "endpoint.json"
    path.write_text(json.dumps({"url": "http://127.0.0.1:1234", "token": "private"}))
    return path


def test_native_terminal_cannot_bypass_shared_queue(tmp_path, monkeypatch):
    endpoint(tmp_path)
    monkeypatch.setattr(agent.os, "execvp", lambda *a: pytest.fail("must not start second agent"))
    args = build_parser().parse_args(["--home", str(tmp_path), "agent", "--name", "managed"])
    with pytest.raises(UnavailableError, match="bypass"):
        agent.run(args)


def test_clean_keeps_shared_service_materials_even_without_containers(tmp_path, monkeypatch):
    path = endpoint(tmp_path)
    monkeypatch.setattr(clean, "running", lambda *a: False)
    args = build_parser().parse_args(["--home", str(tmp_path), "clean", "--name", "managed", "--all"])
    with pytest.raises(UnavailableError, match="files kept"):
        clean.run(args)
    assert path.exists()


def test_down_uses_shared_owner_instead_of_independent_shutdown(tmp_path, monkeypatch):
    endpoint(tmp_path)
    calls = []
    monkeypatch.setattr(down.Client, "end", lambda self: calls.append("end"))
    monkeypatch.setattr(down, "sandbox_down", lambda *a: pytest.fail("must not bypass owner"))
    args = build_parser().parse_args(["--home", str(tmp_path), "down", "--name", "managed"])
    assert down.run(args) == 0
    assert calls == ["end"]


def test_closed_session_records_remain_readable_without_service(tmp_path, capsys):
    from openrua.sessions.core import Session, initial_state
    from openrua.sessions.store import SQLiteStore
    from openrua.cli.commands import session
    path = tmp_path / "sandboxes/saved/conversation.sqlite"
    store = SQLiteStore(path, initial_state())
    core = Session(store)
    core.enqueue("cli", "1", "inspect")
    core.close()
    store.close()
    args = build_parser().parse_args(["--home", str(tmp_path), "session", "--name", "saved", "status"])
    assert session.run(args) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["state"]["closed"]
    assert result["state"]["messages"][0]["text"] == "inspect"


def test_serve_chat_and_down_share_one_resource_owner(tmp_path, monkeypatch, capsys):
    import asyncio
    from types import SimpleNamespace
    from openrua.cli.commands import chat, serve
    from openrua.runner.managed import ManagedSession
    from openrua.sessions.store import SQLiteStore
    from tests.sessions.test_execution import setup, until

    async def run():
        directory = tmp_path / "sandboxes/shared"
        store, core, native, execution = await setup(directory)
        stopped = []
        owned = ManagedSession(SimpleNamespace(power_off=lambda: stopped.append(True)), execution, store)
        async def start(request):
            assert request.name == "shared"
            return owned
        monkeypatch.setattr(serve.managed, "start", start)
        args = build_parser().parse_args(["--home", str(tmp_path), "serve", "--name", "shared"])
        serving = asyncio.create_task(serve.serve(args))
        try:
            await until(lambda: (directory / "endpoint.json").exists())
            chat_args = build_parser().parse_args(["--home", str(tmp_path), "chat", "--name", "shared", "inspect"])
            chatting = asyncio.create_task(asyncio.to_thread(chat.run, chat_args))
            await until(lambda: bool(store.snapshot()["state"]["active"]))
            turn_id = store.snapshot()["state"]["active"]
            await native.frames.put({"kind": "text_delta", "turn_id": turn_id, "data": {"text": "inspection complete", "item_id": "reply"}})
            await native.frames.put({"kind": "turn_finished", "turn_id": turn_id, "data": {"status": "completed"}})
            assert await asyncio.wait_for(chatting, 3) == 0
            assert stopped == [] and not serving.done()
            end_args = build_parser().parse_args(["--home", str(tmp_path), "down", "--name", "shared"])
            assert await asyncio.to_thread(down.run, end_args) == 0
            await asyncio.wait_for(serving, 3)
            assert stopped == [True]
            assert not (directory / "endpoint.json").exists()
            retained = SQLiteStore.open_readonly(directory / "session.sqlite")
            try:
                assert retained.snapshot()["state"]["closed"]
            finally:
                retained.close()
        finally:
            if not serving.done():
                serving.cancel()
                await asyncio.gather(serving, return_exceptions=True)
    asyncio.run(run())
    assert "inspection complete" in capsys.readouterr().out


@pytest.mark.parametrize("no_open", [True, False])
def test_web_opens_only_local_url_and_explicitly_shows_token(tmp_path, monkeypatch, capsys, no_open):
    from openrua.cli.commands import session
    endpoint(tmp_path)
    opened = []
    monkeypatch.setattr(session.Client, "snapshot", lambda self: {})
    monkeypatch.setattr(session.webbrowser, "open", opened.append)
    argv = ["--home", str(tmp_path), "session", "--name", "managed", "web"]
    if no_open:
        argv.append("--no-open")
    assert session.run(build_parser().parse_args(argv)) == 0
    assert opened == ([] if no_open else ["http://127.0.0.1:1234"])
    output = capsys.readouterr().out
    assert "private" in output and "http://127.0.0.1:1234" in output
