"""Terminal interactions over the real shared API, without model or robot costs."""

import asyncio

import pytest

pytest.importorskip("textual")
from textual.widgets import Button, Static, TextArea

from openrua.sessions.client import Client
from openrua.tui.app import ChatApp
from tests.sessions.test_execution import until
from tests.sessions.test_http import running_server, stop


async def scenario(tmp_path, check):
    store, session, native, execution, server = await running_server(tmp_path)
    client = Client(server.url, server.token, timeout=2)
    app = ChatApp(client, "test robot")
    try:
        async with app.run_test(size=(110, 36)) as pilot:
            await until(lambda: app.online)
            await check(app, pilot, store, session, native, client)
    finally:
        await stop(store, execution, server)


async def send(app, pilot, text):
    app.query_one("#message", TextArea).load_text(text)
    await pilot.press("ctrl+s")
    await until(lambda: not app.busy and app.query_one("#message", TextArea).text == "")


async def emit(native, event_kind, turn_id, **data):
    await native.frames.put({"kind": event_kind, "turn_id": turn_id, "data": data})


def test_streaming_queue_cross_client_edit_interrupt_resume_and_detach(tmp_path):
    async def check(app, pilot, store, session, native, client):
        await send(app, pilot, "Inspect the scene")
        first = store.snapshot()["state"]["active"]
        await emit(native, "turn_started", first)
        await emit(native, "text_delta", first, item_id="reply", text="Checking ")
        await emit(native, "text_delta", first, item_id="reply", text="[bold]the camera[/bold]")
        await until(lambda: bool(app.query(".agent-output")))
        await send(app, pilot, "Move to the bowl")
        assert store.snapshot()["state"]["messages"][1]["status"] == "queued"
        second = store.snapshot()["state"]["messages"][1]
        await pilot.press("ctrl+p")
        await until(lambda: bool(app.query(".edit-queued")))
        await pilot.pause()
        assert app.query_one('#panel').display
        assert await pilot.click('.edit-queued')
        await pilot.pause()
        app.screen.query_one("#edit-text", TextArea).load_text("Inspect the bowl")
        await pilot.click("#save")
        await until(lambda: store.snapshot()["state"]["messages"][1]["text"] == "Inspect the bowl")
        await pilot.click("#interrupt")
        await until(lambda: any("interrupt" in frame for frame in native.writes))
        await emit(native, "turn_finished", first, status="interrupted")
        await until(lambda: not app.query_one("#resume", Button).disabled)
        assert store.snapshot()["state"]["paused"]
        await pilot.click("#resume")
        await pilot.click("#confirm")
        await until(lambda: store.snapshot()["state"]["active"] == second["id"])
        await asyncio.to_thread(client.command, "enqueue", client_id="web", request_id="another", text="From the browser")
        await until(lambda: any(m["text"] == "From the browser" for m in app.queue_state))
        await pilot.press("ctrl+q")
        assert not store.snapshot()["state"]["closed"]
        assert store.snapshot()["state"]["active"] == second["id"]
    asyncio.run(scenario(tmp_path, check))


def test_lost_ack_retry_is_deduplicated_and_unicode_draft_survives_streaming(tmp_path):
    async def check(app, pilot, store, session, native, client):
        original = client.command
        original_snapshot = client.snapshot
        def lost(operation, **params):
            original(operation, **params)
            raise RuntimeError("Lost acknowledgement")
        def disconnected():
            raise RuntimeError("Disconnected")
        client.command = lost
        client.snapshot = disconnected
        app.query_one("#message", TextArea).load_text("inspect once")
        await pilot.press("ctrl+s")
        await until(lambda: len(store.snapshot()["state"]["messages"]) == 1 and not app.busy)
        assert app.pending is not None
        client.command = original
        # Retry with the retained request even before an acceptance is observed.
        app.online = True
        app.action_send()
        await until(lambda: app.pending is None and not app.busy)
        assert len(store.snapshot()["state"]["messages"]) == 1
        client.snapshot = original_snapshot
        await until(lambda: app.online)
        app.query_one("#message", TextArea).load_text("Inspect the cup\nthen wait")
        first = store.snapshot()["state"]["active"]
        await emit(native, "text_delta", first, item_id="reply", text="Looking")
        await until(lambda: bool(app.query(".agent-output")))
        assert app.query_one("#message", TextArea).text == "Inspect the cup\nthen wait"
        await pilot.resize_terminal(60, 28)
        await pilot.press("ctrl+p")
        await pilot.pause()
        assert app.query_one("#panel").region.right <= 60
        assert app.query_one("#thread").size.height >= 5
        app.save_screenshot(str(tmp_path / "chat.svg"))
    asyncio.run(scenario(tmp_path, check))


def test_questions_stale_edits_and_end_confirmation(tmp_path):
    from textual.widgets import Input

    async def check(app, pilot, store, session, native, client):
        await send(app, pilot, "Inspect")
        first = store.snapshot()["state"]["active"]
        await emit(native, "input_required", first, request_id="q", questions=[
            {"id": "target", "text": "Which object?", "choices": [], "secret": False}])
        await until(lambda: not app.query_one("#questions", Button).disabled)
        await pilot.click("#questions")
        app.screen.query_one(Input).value = "the cup"
        await pilot.click("#answer")
        await until(lambda: any("reply" in f for f in native.writes))
        assert native.writes[-1] == {"reply": "q", "answers": {"target": ["the cup"]}}
        assert len(store.snapshot()["state"]["messages"]) == 1
        await send(app, pilot, "Next task")
        second = store.snapshot()["state"]["messages"][1]
        await until(lambda: bool(app.query(".edit-queued")))
        await pilot.press("ctrl+p")
        await pilot.pause()
        await pilot.click(".edit-queued")
        await asyncio.to_thread(client.command, "edit", message_id=second["id"], revision=0, text="Edited elsewhere")
        app.screen.query_one(TextArea).load_text("Stale edit")
        await pilot.click("#save")
        await until(lambda: "changed" in str(app.query_one("#notice", Static).content))
        assert store.snapshot()["state"]["messages"][1]["text"] == "Edited elsewhere"
        await pilot.click("#end")
        await pilot.click("#cancel")
        assert not store.snapshot()["state"]["closed"]
        original_end = client.end
        def end_and_disconnect():
            final = original_end()
            def unavailable():
                raise RuntimeError("Service has stopped")
            client.snapshot = unavailable
            return final
        client.end = end_and_disconnect
        await pilot.click("#end")
        await pilot.click("#confirm")
        await until(lambda: app.state["closed"] and app.query_one("#send", Button).disabled)
        assert store.snapshot()["state"]["messages"][1]["status"] == "queued"
        assert app.query_one("#send", Button).disabled
        await pilot.pause(0.3)
        assert "Ended" in str(app.query_one("#status", Static).content)
    asyncio.run(scenario(tmp_path, check))


def test_reconnect_replays_output_and_unknown_execution_keeps_queue_paused(tmp_path):
    async def check(app, pilot, store, session, native, client):
        await send(app, pilot, "Inspect the robot")
        first = store.snapshot()["state"]["active"]
        await emit(native, "item", first, item_id="tool", kind="command", phase="completed", text="Read sensors", output="Camera ready")
        await emit(native, "text_delta", first, item_id="reply", text="The camera is ready.")
        await send(app, pilot, "Move next")
        original = client.snapshot
        def disconnected():
            raise RuntimeError("Connection lost")
        client.snapshot = disconnected
        await until(lambda: not app.online)
        await emit(native, "text_delta", first, item_id="reply", text=" Checking the gripper.")
        client.snapshot = original
        await until(lambda: app.online and "Checking" in str(app.query_one(".agent-output", Static).content))
        assert len([f for f in native.writes if "submit" in f]) == 1
        app.query_one("#message", TextArea).load_text("\u68c0\u67e5\u676f\u5b50\nthen wait")
        await pilot.pause()
        app.save_screenshot(str(tmp_path / "desktop.svg"))
        await native.frames.put(None)
        await until(lambda: app.state["paused"])
        assert app.query_one("#resume", Button).disabled
        assert store.snapshot()["state"]["messages"][1]["status"] == "queued"
        assert app.query_one("#message", TextArea).text.startswith("\u68c0\u67e5")
        # A fresh client replays retained events without submitting any turn.
        async with ChatApp(client, "reconnected").run_test() as fresh:
            await until(lambda: fresh.app.online)
            assert "Checking" in str(fresh.app.query_one(".agent-output", Static).content)
            assert len([f for f in native.writes if "submit" in f]) == 1
    asyncio.run(scenario(tmp_path, check))


def test_workspace_keyboard_preview_preserves_draft_and_does_not_enqueue(tmp_path):
    from openrua.artifacts import WorkspaceFiles
    from textual.widgets import OptionList
    async def run():
        root = tmp_path / 'workspace'
        root.mkdir()
        (root / 'notes #1.txt').write_text('target coordinates\n' + 'line\n' * 80)
        store, session, native, execution, server = await running_server(tmp_path, artifacts=WorkspaceFiles(root))
        app = ChatApp(Client(server.url, server.token, timeout=2))
        try:
            async with app.run_test(size=(80, 28)) as pilot:
                await until(lambda: app.online)
                app.query_one('#message', TextArea).load_text('Keep this instruction')
                await pilot.press('ctrl+o')
                await until(lambda: bool(app.screen.query(OptionList)) and app.screen.query_one(OptionList).option_count == 1)
                await pilot.press('enter')
                await until(lambda: app.screen.query_one('#workspace-preview', TextArea).display)
                assert 'target coordinates' in app.screen.query_one(TextArea).text
                await pilot.press('pagedown')
                await pilot.resize_terminal(60, 20)
                await pilot.press('escape')
                await until(lambda: app.screen.query_one(OptionList).display)
                (root / 'notes #1.txt').unlink()
                await pilot.press('enter')
                await until(lambda: 'no longer exists' in str(app.screen.query_one('#workspace-notice', Static).content))
                await pilot.press('escape')
                await until(lambda: len(app.screen_stack) == 1)
                assert app.query_one('#message', TextArea).text == 'Keep this instruction'
                assert not store.snapshot()['state']['messages']
                assert not [w for w in native.writes if 'submit' in w]
        finally:
            await stop(store, execution, server)
    asyncio.run(run())
