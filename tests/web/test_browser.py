"""Optional Chromium checks over the real local API and a controlled native agent.

Install openrua[browser-test] and run playwright install chromium first.
No robot resources or paid model calls are started.
"""

import asyncio

import pytest

playwright = pytest.importorskip("playwright.async_api")

from openrua.sessions.client import Client
from openrua.web import assets
from tests.sessions.test_execution import until
from tests.sessions.test_http import running_server, stop


async def browser_case(tmp_path, check, **options):
    store, session, native, execution, server = await running_server(tmp_path, assets=assets(), **options)
    try:
        async with playwright.async_playwright() as driver:
            browser = await driver.chromium.launch()
            try:
                context = await browser.new_context(viewport={"width": 1280, "height": 900})
                page = await context.new_page()
                errors = []
                page.on("pageerror", lambda error: errors.append(str(error)))
                await page.goto(server.url)
                await page.locator("#token").fill(server.token)
                await page.get_by_role("button", name="Connect", exact=True).click()
                await playwright.expect(page.locator("#state")).to_have_text("Ready")
                await check(page, store, native, server)
                assert errors == []
            finally:
                await browser.close()
    finally:
        await stop(store, execution, server)


async def send(page, text):
    await page.locator("#message").fill(text)
    await page.locator("#send").click()
    await playwright.expect(page.locator("#message")).to_have_value("")


async def emit(native, event_kind, turn_id, **data):
    await native.frames.put({"kind": event_kind, "turn_id": turn_id, "data": data})


def test_browser_and_cli_queue_interrupt_edit_and_end(tmp_path):
    async def check(page, store, native, server):
        await send(page, "Inspect the scene")
        first = store.snapshot()["state"]["active"]
        await emit(native, "turn_started", first)
        await emit(native, "text_delta", first, item_id="reply", text="Checking ")
        await emit(native, "text_delta", first, item_id="reply", text="the camera")
        await emit(native, "item", first, item_id="tool", kind="command", phase="completed", text="read sensors", output="camera ready")
        await playwright.expect(page.locator(".agent-output")).to_have_text("Checking the camera")
        await page.locator(".tool summary").click()
        await playwright.expect(page.locator(".tool pre")).to_have_text("Output\ncamera ready")
        cli = Client(server.url, server.token)
        second = await asyncio.to_thread(cli.command, "enqueue", client_id="cli", request_id="second", text="Move to the bowl")
        await playwright.expect(page.locator("#queue-count")).to_have_text("1")
        await page.locator("#queue").get_by_role("button", name="Edit", exact=True).click()
        await page.locator("#edit-text").fill("Inspect the bowl first")
        await page.locator("#edit-form").get_by_role("button", name="Save").click()
        await playwright.expect(page.locator("#queue")).to_contain_text("Inspect the bowl first")
        await send(page, "Discard this instruction")
        await page.locator("#queue .queue-item").filter(has_text="Discard this instruction").get_by_role("button", name="Remove").click()
        await playwright.expect(page.locator("#queue-count")).to_have_text("1")
        await page.locator("#interrupt").click()
        await until(lambda: any("interrupt" in frame for frame in native.writes))
        await emit(native, "turn_finished", first, status="interrupted")
        # The interrupt stops the turn; the queued instruction goes next,
        # with nothing to resume.
        await until(lambda: store.snapshot()["state"]["active"] == second["id"])
        await playwright.expect(page.locator("#resume")).to_be_disabled()
        page.on("dialog", lambda dialog: dialog.accept())
        assert [f["text"] for f in native.writes if "submit" in f] == ["Inspect the scene", "Inspect the bowl first"]
        await page.locator("#end").click()
        await playwright.expect(page.locator("#state")).to_have_text("Ended")
        await playwright.expect(page.locator("article").filter(has_text="Inspect the bowl first").locator(".turn-heading")).to_contain_text("unknown")
        assert store.snapshot()["state"]["closed"]
        await playwright.expect(page.locator("#send")).to_be_disabled()
    asyncio.run(browser_case(tmp_path, check))


def test_browser_reload_after_lost_ack_does_not_submit_again(tmp_path):
    async def check(page, store, native, server):
        async def drop_response(route):
            await route.fetch()
            await route.abort()
        await page.route("**/api/commands", drop_response)
        await page.route("**/api/session", lambda route: route.abort())
        await page.locator("#message").fill("One instruction only")
        await page.locator("#send").click()
        await until(lambda: bool(store.snapshot()["state"]["active"]))
        await playwright.expect(page.locator("#connection")).to_contain_text("Disconnected")
        await page.unroute("**/api/session")
        await page.unroute("**/api/commands")
        await page.reload()
        assert len([f for f in native.writes if "submit" in f]) == 1
        await page.locator("#token").fill(server.token)
        await page.get_by_role("button", name="Connect", exact=True).click()
        await playwright.expect(page.locator("#message")).to_have_value("")
        await playwright.expect(page.locator(".user-text")).to_have_text("One instruction only")
        await page.locator("#disconnect").click()
        first = store.snapshot()["state"]["active"]
        await emit(native, "turn_finished", first, status="completed", text="Done while disconnected")
        await until(lambda: store.snapshot()["state"]["active"] is None)
        assert not store.snapshot()["state"]["closed"]
        await page.locator("#token").fill(server.token)
        await page.get_by_role("button", name="Connect", exact=True).click()
        await playwright.expect(page.locator(".turn-result")).to_have_text("Done while disconnected")
        assert len([f for f in native.writes if "submit" in f]) == 1
        # Credentials are not persisted into either browser storage area or URLs.
        assert server.token not in await page.evaluate("JSON.stringify({...sessionStorage,...localStorage})")
        assert page.url == server.url + "/"
    asyncio.run(browser_case(tmp_path, check))


def test_browser_questions_are_separate_from_tasks_and_text_is_not_html(tmp_path):
    async def check(page, store, native, server):
        payload = '<img src=x onerror="window.injected=true">'
        await send(page, payload)
        first = store.snapshot()["state"]["active"]
        await emit(native, "text_delta", first, item_id="reply", text=payload)
        await emit(native, "input_required", first, request_id="question", questions=[
            {"id": "decision", "text": "Allow this action?", "choices": ["Allow", "Deny"], "secret": False}])
        await playwright.expect(page.locator("#questions")).to_be_visible()
        await page.get_by_label("Allow this action?").select_option("Deny")
        await page.get_by_role("button", name="Send answer", exact=True).click()
        await until(lambda: any("reply" in f for f in native.writes))
        assert native.writes[-1] == {"reply": "question", "answers": {"decision": ["Deny"]}}
        assert len(store.snapshot()["state"]["messages"]) == 1
        await playwright.expect(page.locator("#questions")).to_be_hidden()
        await playwright.expect(page.locator(".agent-output")).to_have_text(payload)
        assert await page.locator("#thread img").count() == 0
        assert await page.evaluate("window.injected") is None
        await page.set_viewport_size({"width": 390, "height": 844})
        assert await page.evaluate("document.documentElement.scrollWidth <= window.innerWidth")
        await page.screenshot(path=str(tmp_path / "mobile.png"), full_page=True)
    asyncio.run(browser_case(tmp_path, check))


def test_browser_async_choices_and_free_text_use_the_message_queue(tmp_path, monkeypatch):
    from tests.sessions.test_execution import FakeProtocol
    replies = []

    def format_reply(self, request_id, answers):
        replies.append((request_id, answers))
        return '<native-answer>opaque reply format</native-answer>'

    monkeypatch.setattr(FakeProtocol, 'response_message', format_reply)

    async def check(page, store, native, server):
        await send(page, 'Inspect the scene')
        first = store.snapshot()['state']['active']
        await emit(native, 'turn_started', first)
        await emit(native, 'input_required', first, request_id='async', reply_as_message=True, questions=[
            {'id': 'target', 'text': 'Which target?', 'choices': ['Cup', 'Bowl'], 'allow_other': True},
            {'id': 'place', 'text': 'Where should it go?', 'choices': [], 'allow_other': True}])
        await playwright.expect(page.locator('#questions')).to_be_visible()
        await playwright.expect(page.locator('#state')).to_have_text('Working')
        await page.locator('#question-list select').select_option('Cup')
        await page.get_by_label('Where should it go?', exact=True).fill('On the tray')
        await page.get_by_role('button', name='Send answer', exact=True).click()
        await until(lambda: len(store.snapshot()['state']['messages']) == 2)
        assert replies == [('async', {'target': ['Cup'], 'place': ['On the tray']})]
        assert not any('reply' in frame for frame in native.writes)
        assert not store.snapshot()['state']['paused']
        await playwright.expect(page.locator('#questions')).to_be_hidden()
        await playwright.expect(page.locator('#queue')).to_contain_text('Answer: Cup')
        await playwright.expect(page.locator('#queue')).not_to_contain_text('native-answer')
        await emit(native, 'turn_finished', first, status='completed')
        await until(lambda: len([frame for frame in native.writes if 'submit' in frame]) == 2)
        assert native.writes[-1]['text'] == '<native-answer>opaque reply format</native-answer>'
        await playwright.expect(page.locator('#thread')).to_contain_text('Answer: On the tray')
        await playwright.expect(page.locator('#thread')).not_to_contain_text('native-answer')
        await playwright.expect(page.locator('#message')).to_have_value('')
    asyncio.run(browser_case(tmp_path, check))


def test_browser_restores_an_unfinished_async_answer_when_the_turn_ends(tmp_path):
    async def check(page, store, native, server):
        await send(page, 'Inspect the scene')
        first = store.snapshot()['state']['active']
        await emit(native, 'input_required', first, request_id='async', reply_as_message=True, questions=[
            {'id': 'target', 'text': 'Which target?', 'choices': ['Cup'], 'allow_other': True}])
        await playwright.expect(page.locator('#questions')).to_be_visible()
        await page.get_by_label('Which target? (another answer)', exact=True).fill('The blue mug, but')
        await page.locator('#message').fill('My existing draft')
        await emit(native, 'turn_finished', first, status='completed')
        await playwright.expect(page.locator('#questions')).to_be_hidden()
        await playwright.expect(page.locator('#message')).to_have_value('My existing draft\n\nWhich target?\nThe blue mug, but')
        assert len(store.snapshot()['state']['messages']) == 1
        assert not any('reply' in frame for frame in native.writes)
    asyncio.run(browser_case(tmp_path, check))


def test_browser_does_not_claim_shutdown_on_error_and_keeps_unknown_queue_paused(tmp_path):
    async def check(page, store, native, server):
        await send(page, "Check the table")
        await send(page, "Then move the cup")
        await native.frames.put(None)
        await playwright.expect(page.locator("#unknown-panel")).to_be_visible()
        await playwright.expect(page.locator("#resume")).to_be_disabled()
        await page.get_by_role("button", name="Record what happened").click()
        await page.locator("#edit-text").fill("Inspected robot and workspace; no completion confirmed")
        await page.locator("#edit-form").get_by_role("button", name="Save").click()
        await playwright.expect(page.locator("#unknown-panel")).to_be_hidden()
        await playwright.expect(page.locator("#resume")).to_be_disabled()
        assert store.snapshot()["state"]["messages"][1]["status"] == "queued"
        end = server.end
        async def failed_end():
            raise RuntimeError("robot shutdown failed")
        server.end = failed_end
        page.on("dialog", lambda dialog: dialog.accept())
        await page.locator("#end").click()
        await playwright.expect(page.locator("#notice")).to_contain_text("robot shutdown failed")
        await playwright.expect(page.locator("#state")).to_have_text("Paused")
        assert not store.snapshot()["state"]["closed"]
        server.end = end
        await page.locator("#end").click()
        await playwright.expect(page.locator("#state")).to_have_text("Ended")
        assert store.snapshot()["state"]["messages"][1]["status"] == "queued"
    asyncio.run(browser_case(tmp_path, check))


def test_browser_previews_saved_files_without_submitting_tasks(tmp_path):
    import base64
    from openrua.artifacts import WorkspaceFiles
    workspace = tmp_path / "workspace"
    workspace.mkdir()
    (workspace / "snaps").mkdir()
    image = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+jRZkAAAAASUVORK5CYII=")
    (workspace / "snaps/camera.png").write_bytes(image)
    code = '<script>window.injected=true</script>'
    (workspace / "control.py").write_text(code)
    (workspace / "depth.npy").write_bytes(b"\x93NUMPY\0bytes")
    async def check(page, store, native, server):
        await page.locator("#files-list").get_by_role("button", name="control.py", exact=True).click()
        await playwright.expect(page.locator("#file-content pre")).to_have_text(code)
        assert await page.evaluate("window.injected") is None
        await page.get_by_role("button", name="Close preview").click()
        # A slow listing must not overwrite a directory the user is typing.
        held, release = asyncio.Event(), asyncio.Event()
        async def delayed_directory(route):
            response = await route.fetch()
            held.set()
            await release.wait()
            await route.fulfill(response=response)
        await page.route("**/api/workspace/list?path=", delayed_directory)
        await page.get_by_role("button", name="Refresh", exact=True).click()
        await asyncio.wait_for(held.wait(), 2)
        await page.locator("#files-path").fill("snaps")
        release.set()
        await playwright.expect(page.locator("#files-status")).to_contain_text("Select a file")
        await playwright.expect(page.locator("#files-path")).to_have_value("snaps")
        await page.unroute("**/api/workspace/list?path=")
        await page.get_by_role("button", name="Refresh", exact=True).click()
        await page.locator("#files-list").get_by_role("button", name="camera.png", exact=True).click()
        await playwright.expect(page.locator("#file-note")).to_contain_text("This is a saved image, not live video. Use Cameras to watch live video.")
        await page.wait_for_function("document.querySelector('#file-content img')?.naturalWidth === 1")
        async with page.expect_download() as download:
            await page.locator("#file-download").click()
        downloaded = await download.value
        assert downloaded.suggested_filename == "camera.png"
        from pathlib import Path
        assert Path(await downloaded.path()).read_bytes() == image
        await page.set_viewport_size({"width": 390, "height": 844})
        assert await page.evaluate("document.documentElement.scrollWidth <= window.innerWidth")
        await page.screenshot(path=str(tmp_path / "workspace-mobile.png"), full_page=True)
        await page.get_by_role("button", name="Close preview").click()
        (workspace / "snaps/new.txt").write_text("New observation")
        await page.get_by_role("button", name="Refresh", exact=True).click()
        await page.locator("#files-list").get_by_role("button", name="new.txt", exact=True).click()
        await playwright.expect(page.locator("#file-content pre")).to_have_text("New observation")
        await page.get_by_role("button", name="Close preview").click()
        await page.locator("#disconnect").click()
        await page.locator("#token").fill(server.token)
        await page.get_by_role("button", name="Connect", exact=True).click()
        await playwright.expect(page.locator("#files-list")).to_contain_text("new.txt")
        assert store.snapshot()["state"]["messages"] == []
        assert not any("submit" in f for f in native.writes)
    asyncio.run(browser_case(tmp_path, check, artifacts=WorkspaceFiles(workspace)))


def test_browser_multiple_questions_and_optional_text_preserve_native_answers(tmp_path):
    async def check(page, store, native, server):
        await send(page, 'Inspect')
        first = store.snapshot()['state']['active']
        await emit(native, 'input_required', first, request_id='multi', questions=[
            {'id': 'locations', 'text': 'Which locations?', 'choices': ['Near', 'Far'],
             'multiple': True, 'allow_other': True},
            {'id': 'color', 'text': 'Which color?', 'choices': ['Red', 'Blue'], 'allow_other': True}])
        await page.locator('#question-list select').nth(0).select_option(['Near', 'Far'])
        await page.get_by_label('Which locations? (another answer)', exact=True).fill('Shelf')
        await page.locator('#question-list select').nth(1).select_option('Red')
        await page.get_by_label('Which color? (another answer)', exact=True).fill('Green')
        await playwright.expect(page.locator('#question-list select').nth(1)).to_have_value('')
        # A subsequent predefined single choice replaces the custom response.
        await page.locator('#question-list select').nth(1).select_option('Blue')
        await playwright.expect(page.get_by_label('Which color? (another answer)', exact=True)).to_have_value('')
        await page.get_by_label('Which color? (another answer)', exact=True).fill('Green')
        assert not any('reply' in f for f in native.writes)
        await page.get_by_role('button', name='Send answer', exact=True).click()
        await until(lambda: any('reply' in f for f in native.writes))
        assert native.writes[-1] == {'reply': 'multi', 'answers': {
            'locations': ['Near', 'Far', 'Shelf'], 'color': ['Green']}}
        assert len(store.snapshot()['state']['messages']) == 1
    asyncio.run(browser_case(tmp_path, check))


def test_browser_tool_completion_preserves_details_and_empty_output(tmp_path):
    async def check(page, store, native, server):
        await send(page, "Inspect interfaces")
        first = store.snapshot()["state"]["active"]
        await emit(native, "item", first, item_id="tool", kind="tool", phase="started",
                   text="Bash", details={"command": "ros2 topic list"})
        await page.locator(".tool summary").click()
        await playwright.expect(page.locator(".tool pre")).to_contain_text("ros2 topic list")
        await emit(native, "item", first, item_id="tool", kind="tool", phase="completed",
                   text="Bash", output="/camera")
        await playwright.expect(page.locator(".tool summary")).to_contain_text("completed")
        await playwright.expect(page.locator(".tool pre")).to_contain_text("ros2 topic list")
        await playwright.expect(page.locator(".tool pre")).to_contain_text("/camera")
        await emit(native, "item", first, item_id="tool", kind="tool", phase="completed",
                   text="Bash", output="")
        await playwright.expect(page.locator(".tool pre")).not_to_contain_text("/camera")
        await playwright.expect(page.locator(".tool pre")).to_contain_text("ros2 topic list")
        await playwright.expect(page.locator(".tool pre")).to_contain_text("Output")
        await page.reload()
        await page.locator("#token").fill(server.token)
        await page.get_by_role("button", name="Connect", exact=True).click()
        await page.locator(".tool summary").click()
        await playwright.expect(page.locator(".tool pre")).to_contain_text("ros2 topic list")
        await playwright.expect(page.locator(".tool pre")).not_to_contain_text("/camera")
        assert len([f for f in native.writes if "submit" in f]) == 1
    asyncio.run(browser_case(tmp_path, check))


def test_the_page_offers_every_command_the_shared_list_gives_it(tmp_path):
    # One list of commands for every client (openrua/web/session.mjs); a
    # command added there without a handler here fails this.
    async def check(page, store, native, server):
        seen = await page.evaluate("""async () => {
            const shared = await import('/session.mjs'), page = await import('/app.js');
            return {want: shared.commandsFor('browser').map(c => c.name).sort(),
                    have: Object.keys(page.handlers).sort()};
        }""")
        assert seen["want"] == seen["have"]
    asyncio.run(browser_case(tmp_path, check))


def test_the_page_changes_the_agents_settings_and_says_what_events_say(tmp_path):
    from openrua.sessions.core import Session

    async def check(page, store, native, server):
        await emit(native, 'setting', None, setting='permissions', value='default', choices=[
            {'value': 'default', 'label': 'Default'}, {'value': 'auto', 'label': 'Auto'}])
        await playwright.expect(page.locator('#permissions')).to_be_enabled()
        await send(page, "/permissions default")
        await playwright.expect(page.locator("#notes")).to_contain_text("Permissions: Default")
        await page.locator("#permissions").select_option("auto")
        await playwright.expect(page.locator("#notes")).to_contain_text("Permissions: Auto")
        await send(page, "/model m2")
        await playwright.expect(page.locator("#agent-model")).to_have_text("m2")
        await playwright.expect(page.locator("#notice")).to_contain_text("Model changed to m2")
        Session(store).motion_canceled({"/arm/fjt": True, "/arm/grip": False})
        await playwright.expect(page.locator("#notice")).to_contain_text("Motion canceled: /arm/fjt")
        await playwright.expect(page.locator("#notice")).to_contain_text("use the emergency stop")
        await send(page, "/help")
        await playwright.expect(page.locator("#notice")).to_contain_text("/permissions:")
        await send(page, "/quit")
        await playwright.expect(page.locator("#notice")).to_have_text("/quit is a terminal command.")
        assert store.snapshot()["state"]["messages"] == []   # none of it went to the agent
    asyncio.run(browser_case(tmp_path, check))


def test_a_typed_line_answers_the_question_the_agent_waits_on(tmp_path):
    # As in the terminal: free text answers it; a line that is none of a
    # closed question's choices sends you to its form; nothing is queued.
    async def check(page, store, native, server):
        await send(page, "Turn joint 7")
        first = store.snapshot()["state"]["active"]
        await emit(native, "input_required", first, request_id="how", questions=[
            {"id": "mode", "text": "Position or velocity?", "secret": False}])
        await playwright.expect(page.locator("#notes")).to_contain_text("Type a reply")
        await send(page, "position, then try the other joints")
        await until(lambda: any("reply" in f for f in native.writes))
        assert native.writes[-1] == {"reply": "how", "answers": {"mode": ["position, then try the other joints"]}}
        await emit(native, "input_required", first, request_id="ok", questions=[
            {"id": "go", "text": "Allow this action?", "choices": ["Allow", "Deny"], "secret": False}])
        await playwright.expect(page.locator("#questions")).to_be_visible()
        await page.locator("#message").fill("maybe")
        await page.locator("#send").click()
        await playwright.expect(page.locator("#notice")).to_have_text("Answer this question in its form.")
        assert len(store.snapshot()["state"]["messages"]) == 1
        await emit(native, "item", first, item_id="t", kind="command", phase="completed",
                   text="Bash", details={"command": "ros2 topic list\nmore"}, output="/joint_states")
        await playwright.expect(page.locator(".tool summary")).to_have_text("completed · Bash(ros2 topic list)")
        # The closing text repeats what the turn already showed: not twice.
        await emit(native, "text_delta", first, item_id="reply", text="Joint 7 turned.")
        await emit(native, "turn_finished", first, status="completed", text="Joint 7 turned.")
        await playwright.expect(page.locator(".turn-result")).to_have_text("")
        await playwright.expect(page.locator(".agent-output")).to_have_text("Joint 7 turned.")
    asyncio.run(browser_case(tmp_path, check))


def test_the_page_shows_a_camera_live_once_watched(tmp_path):
    import base64
    asked = []

    class View:
        def names(self):
            return ["scene", "wrist"]
        def frame(self, name):
            asked.append(name)
            # a 1x1 PNG, as a tap would hand over a frame
            png = base64.b64encode(bytes.fromhex(
                "89504e470d0a1a0a0000000d4948445200000001000000010806000000"
                "1f15c4890000000d49444154789c6360f8cfc0f01f0005fe02fea7d6a4"
                "940000000049454e44ae426082")).decode()
            return {"name": name, "t": 1.0, "age_s": 0.2, "mime": "image/png", "data": png}

    async def check(page, store, native, server):
        await playwright.expect(page.locator("#camera-frame")).to_be_hidden()
        assert asked == []                     # nothing is read until someone watches
        await page.locator("#camera-watch").click()
        await playwright.expect(page.locator("#camera-frame")).to_be_visible()
        await playwright.expect(page.locator("#camera-status")).to_have_text("Live · now")
        assert await page.locator("#camera-name option").all_text_contents() == ["scene", "wrist"]
        await page.locator("#camera-name").select_option("wrist")
        await until(lambda: asked and asked[-1] == "wrist")
        await page.locator("#camera-watch").click()   # Stop
        stopped = len(asked)
        await page.wait_for_timeout(1000)
        assert len(asked) <= stopped + 1
    asyncio.run(browser_case(tmp_path, check, cameras=View()))


def test_browser_robot_fault_holds_old_work_and_allows_repair_without_leaving(tmp_path):
    async def check(page, store, native, server):
        old = server.execution.session.enqueue('cli', 'old', 'move to the bowl')
        await server.execution.robot_status('stopped', 'driver exited', hold=True)
        await playwright.expect(page.locator('#state')).to_have_text('Ready to chat')
        await playwright.expect(page.locator('#queue')).to_contain_text('Not sent yet')
        await send(page, 'read the driver log')
        await until(lambda: store.snapshot()['state']['active'] is not None)
        repair = store.snapshot()['state']['active']
        assert repair != old['id']
        assert all(frame.get('submit') != old['id'] for frame in native.writes)
        await emit(native, 'turn_finished', repair, status='completed')
        await until(lambda: store.snapshot()['state']['active'] is None)
        page.on('dialog', lambda dialog: dialog.accept())
        await page.get_by_role('button', name='Send this instruction now').click()
        await until(lambda: store.snapshot()['state']['active'] == old['id'])
        assert page.url == server.url + '/'
    asyncio.run(browser_case(tmp_path, check))


def test_browser_robot_request_survives_lost_ack_and_reload_with_same_identity(tmp_path):
    async def check(page, store, native, server):
        calls, jobs = [], {}
        async def action(body):
            calls.append(dict(body))
            return jobs.setdefault(body['request_id'], {**body, 'status': 'completed', 'summary': 'restarted'})
        server.actions['robot'] = action
        page.on('dialog', lambda dialog: dialog.accept())
        async def lose(route):
            await route.fetch()
            await route.abort()
        await page.route('**/api/actions/robot', lose)
        await send(page, '/robot restart')
        await playwright.expect(page.locator('#notice')).to_contain_text('not confirmed')
        assert len(calls) == len(jobs) == 1
        await page.unroute('**/api/actions/robot')
        await page.reload()
        await page.locator('#token').fill(server.token)
        await page.get_by_role('button', name='Connect', exact=True).click()
        await playwright.expect(page.locator('#state')).to_have_text('Ready')
        await send(page, '/robot retry')
        await playwright.expect(page.locator('#notice')).to_contain_text('completed')
        assert calls[0] == calls[1] and len(jobs) == 1
        assert not [frame for frame in native.writes if 'submit' in frame]
    asyncio.run(browser_case(tmp_path, check))
