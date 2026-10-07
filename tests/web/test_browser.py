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


def test_browser_and_cli_queue_interrupt_edit_resume_and_end(tmp_path):
    async def check(page, store, native, server):
        await send(page, "Inspect the scene")
        first = store.snapshot()["state"]["active"]
        await emit(native, "turn_started", first)
        await emit(native, "text_delta", first, item_id="reply", text="Checking ")
        await emit(native, "text_delta", first, item_id="reply", text="the camera")
        await emit(native, "item", first, item_id="tool", kind="command", phase="completed", text="read sensors", output="camera ready")
        await playwright.expect(page.locator(".agent-output")).to_have_text("Checking the camera")
        await page.locator(".tool summary").click()
        await playwright.expect(page.locator(".tool pre")).to_have_text("camera ready")
        cli = Client(server.url, server.token)
        second = await asyncio.to_thread(cli.command, "enqueue", client_id="cli", request_id="second", text="Move to the bowl")
        await playwright.expect(page.locator("#queue-count")).to_have_text("1")
        await page.locator("#queue").get_by_role("button", name="Edit", exact=True).click()
        await page.locator("#edit-text").fill("Inspect the bowl first")
        await page.locator("#edit-form").get_by_role("button", name="Save").click()
        await playwright.expect(page.locator("#queue")).to_contain_text("Inspect the bowl first")
        await send(page, "Discard this instruction")
        await page.locator("#queue .queue-item").filter(has_text="Discard this instruction").get_by_role("button", name="Withdraw").click()
        await playwright.expect(page.locator("#queue-count")).to_have_text("1")
        await page.locator("#interrupt").click()
        await until(lambda: any("interrupt" in frame for frame in native.writes))
        await emit(native, "turn_finished", first, status="interrupted")
        await playwright.expect(page.locator("#resume")).to_be_enabled()
        assert len([f for f in native.writes if "submit" in f]) == 1
        page.on("dialog", lambda dialog: dialog.accept())
        await page.locator("#resume").click()
        await until(lambda: store.snapshot()["state"]["active"] == second["id"])
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
        await playwright.expect(page.locator("#file-note")).to_contain_text("not a live camera feed")
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
