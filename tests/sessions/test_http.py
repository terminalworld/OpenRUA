"""Actual local HTTP clients share one execution and reconnect to retained events."""

import asyncio
import http.client
import json
import socket
from urllib.parse import urlsplit

import pytest

from openrua.sessions.client import Client, write_endpoint
from openrua.sessions.http import LocalServer
from tests.sessions.test_execution import setup, until


async def running_server(tmp_path, **options):
    store, session, transport, execution = await setup(tmp_path)
    async def end():
        await execution.close()
        session.close()
    server = LocalServer(execution, end, "private-test-token", **options)
    server.start()
    return store, session, transport, execution, server


async def stop(store, execution, server):
    await server.close()
    await execution.close()
    store.close()


def test_two_http_clients_share_queue_and_replay_after_disconnect(tmp_path):
    async def run():
        store, session, native, execution, server = await running_server(tmp_path)
        cli, web = Client(server.url, server.token), Client(server.url, server.token)
        try:
            first = await asyncio.to_thread(cli.command, "enqueue", client_id="cli", request_id="a", text="inspect")
            cursor = (await asyncio.to_thread(cli.snapshot))["cursor"]
            second = await asyncio.to_thread(web.command, "enqueue", client_id="web", request_id="b", text="grasp")
            # Neither HTTP client is connected while native execution completes.
            await native.frames.put({"kind": "turn_finished", "turn_id": first["id"], "data": {"status": "completed"}})
            await until(lambda: store.snapshot()["state"]["active"] == second["id"])
            fresh = Client(server.url, server.token)
            events = await asyncio.to_thread(fresh.events, cursor)
            assert any(e["kind"] == "dispatch_started" and e["data"]["message_id"] == second["id"] for e in events)
            duplicate = await asyncio.to_thread(fresh.command, "enqueue", client_id="cli", request_id="a", text="inspect")
            assert duplicate["id"] == first["id"]
            assert len([f for f in native.writes if "submit" in f]) == 2
        finally:
            await stop(store, execution, server)
    asyncio.run(run())


def test_lost_http_response_does_not_cancel_or_repeat_delivery(tmp_path):
    async def run():
        store, session, native, execution, server = await running_server(tmp_path)
        try:
            body = json.dumps({"operation": "enqueue", "params": {"client_id": "lost", "request_id": "once", "text": "inspect"}}).encode()
            def send_and_disconnect():
                port = urlsplit(server.url).port
                with socket.create_connection(("127.0.0.1", port)) as stream:
                    stream.sendall((f"POST /api/commands HTTP/1.0\r\nHost: {server.address}\r\nAuthorization: Bearer {server.token}\r\nContent-Type: application/json\r\nContent-Length: {len(body)}\r\n\r\n").encode() + body)
            await asyncio.to_thread(send_and_disconnect)
            await until(lambda: len(store.snapshot()["state"]["messages"]) == 1)
            retry = await asyncio.to_thread(Client(server.url, server.token).command, "enqueue", client_id="lost", request_id="once", text="inspect")
            assert retry["id"] == store.snapshot()["state"]["active"]
            assert len([f for f in native.writes if "submit" in f]) == 1
        finally:
            await stop(store, execution, server)
    asyncio.run(run())


def test_http_rejects_foreign_origin_missing_token_and_invalid_commands(tmp_path):
    async def run():
        store, session, native, execution, server = await running_server(tmp_path)
        def raw(headers, body=None, path="/api/session"):
            connection = http.client.HTTPConnection(server.address)
            try:
                connection.request("POST" if body is not None else "GET", path, body=body, headers=headers)
                response = connection.getresponse()
                response.read()
                return response.status
            finally:
                connection.close()
        try:
            assert await asyncio.to_thread(raw, {}) == 401
            auth = {"Authorization": "Bearer " + server.token}
            assert await asyncio.to_thread(raw, {**auth, "Origin": "https://elsewhere.example"}) == 403
            assert await asyncio.to_thread(raw, {**auth, "Host": "elsewhere.example"}) == 403
            client = Client(server.url, server.token)
            with pytest.raises(RuntimeError, match="argument type"):
                await asyncio.to_thread(client.command, "enqueue", client_id="a", request_id="1", text={})
            assert store.snapshot()["state"]["messages"] == []
        finally:
            await stop(store, execution, server)
    asyncio.run(run())


def test_http_end_waits_for_shutdown_and_retains_records(tmp_path):
    async def run():
        store, session, native, execution, server = await running_server(tmp_path)
        client = Client(server.url, server.token)
        try:
            message = await asyncio.to_thread(client.command, "enqueue", client_id="a", request_id="1", text="inspect")
            closed = await asyncio.to_thread(client.request, "/api/end", {})
            assert closed["result"]["state"]["closed"]
            assert closed["result"]["state"]["messages"][0]["status"] == "unknown"
            await asyncio.wait_for(server.ended.wait(), 2)
            snapshot = await asyncio.to_thread(client.snapshot)
            assert snapshot["state"]["closed"]
            assert snapshot["state"]["messages"][0]["id"] == message["id"]
            assert snapshot["state"]["messages"][0]["status"] == "unknown"
        finally:
            await stop(store, execution, server)
    asyncio.run(run())


def test_endpoint_token_is_private_and_cannot_overwrite_another_owner(tmp_path):
    path = tmp_path / "endpoint.json"
    write_endpoint(path, "http://127.0.0.1:1234", "secret")
    assert path.stat().st_mode & 0o777 == 0o600
    assert Client.from_file(path).token == "secret"
    with pytest.raises(FileExistsError):
        write_endpoint(path, "http://127.0.0.1:1234", "different")
    assert Client.from_file(path).token == "secret"
    assert list(tmp_path.iterdir()) == [path]
    with pytest.raises(ValueError, match="local"):
        Client("http://remote.example:1234", "secret")



def test_endpoint_is_invisible_until_connection_facts_are_complete(tmp_path, monkeypatch):
    from openrua.sessions import client as module
    path = tmp_path / "endpoint.json"
    dump = module.json.dump

    def interrupted_write(facts, stream):
        stream.write("{")
        stream.flush()
        assert not path.exists(), "a reconnect must not see incomplete connection facts"
        stream.seek(0)
        stream.truncate()
        dump(facts, stream)

    monkeypatch.setattr(module.json, "dump", interrupted_write)
    write_endpoint(path, "http://127.0.0.1:1234", "secret")
    assert Client.from_file(path).token == "secret"
    assert list(tmp_path.iterdir()) == [path]


def test_failed_endpoint_write_leaves_no_connection_file(tmp_path, monkeypatch):
    from openrua.sessions import client as module
    path = tmp_path / "endpoint.json"

    def failed_write(facts, stream):
        stream.write("{")
        stream.flush()
        raise OSError("disk write failed")

    monkeypatch.setattr(module.json, "dump", failed_write)
    with pytest.raises(OSError, match="disk write failed"):
        write_endpoint(path, "http://127.0.0.1:1234", "secret")
    assert list(tmp_path.iterdir()) == []

def test_http_timeout_does_not_cancel_accepted_command(tmp_path):
    async def run():
        store, session, native, execution, server = await running_server(tmp_path, request_timeout=0.05)
        command = execution.command
        async def delayed(operation, **params):
            await asyncio.sleep(0.1)
            return await command(operation, **params)
        execution.command = delayed
        try:
            client = Client(server.url, server.token)
            with pytest.raises(RuntimeError, match="not yet known"):
                await asyncio.to_thread(client.command, "enqueue", client_id="web", request_id="one", text="inspect")
            await until(lambda: bool(store.snapshot()["state"]["messages"]))
            assert len([f for f in native.writes if "submit" in f]) == 1
        finally:
            await stop(store, execution, server)
    asyncio.run(run())


def test_browser_assets_do_not_expose_session_files_or_bypass_api_auth(tmp_path):
    from openrua.web import assets
    async def run():
        store, session, native, execution, server = await running_server(tmp_path, assets=assets())
        def get(path, headers=None):
            connection = http.client.HTTPConnection(server.address)
            try:
                connection.request("GET", path, headers=headers or {})
                response = connection.getresponse()
                return response.status, dict(response.getheaders()), response.read()
            finally:
                connection.close()
        try:
            for path, (mime, expected) in assets().items():
                status, headers, body = await asyncio.to_thread(get, path)
                assert status == 200 and body == expected
                assert headers["Content-Type"] == mime
                assert "frame-ancestors 'none'" in headers["Content-Security-Policy"]
                assert server.token.encode() not in body
            assert (await asyncio.to_thread(get, "/api/session"))[0] == 401
            assert (await asyncio.to_thread(get, "/", {"Origin": "https://elsewhere.example"}))[0] == 403
            auth = {"Authorization": "Bearer " + server.token}
            for path in ("/../session.sqlite", "/endpoint.json", "/assets/workspace/README.md"):
                assert (await asyncio.to_thread(get, path, auth))[0] == 404
        finally:
            await stop(store, execution, server)
    asyncio.run(run())


def test_workspace_api_is_authenticated_read_only_and_scoped(tmp_path):
    from openrua.artifacts import WorkspaceFiles
    async def run():
        workspace = tmp_path / "workspace"
        workspace.mkdir()
        (workspace / "notes.txt").write_text("hello")
        (workspace / "escape").symlink_to(tmp_path)
        store, session, native, execution, server = await running_server(tmp_path, artifacts=WorkspaceFiles(workspace))
        client = Client(server.url, server.token)
        try:
            with pytest.raises(RuntimeError, match="token"):
                await asyncio.to_thread(Client(server.url, "wrong").request, "/api/workspace/read?path=notes.txt")
            listing = await asyncio.to_thread(client.request, "/api/workspace/list?path=")
            assert {e["name"] for e in listing["entries"]} == {"notes.txt", "escape"}
            value = await asyncio.to_thread(client.request, "/api/workspace/read?path=notes.txt")
            assert value["text"] == "hello"
            for path in ("../conversation.sqlite", "escape/conversation.sqlite", "/etc/passwd", "missing"):
                with pytest.raises(RuntimeError):
                    await asyncio.to_thread(client.request, "/api/workspace/read?path=" + path)
            with pytest.raises(RuntimeError, match="one relative"):
                await asyncio.to_thread(client.request, "/api/workspace/list?path=a&path=b")
            assert store.snapshot()["state"]["messages"] == []
            assert not any("submit" in f for f in native.writes)
        finally:
            await stop(store, execution, server)
    asyncio.run(run())
