"""A local HTTP boundary over the shared session contract.

The server owns neither agent protocols nor robot resources. Callers supply
an execution owner and an explicit end operation. Requests run on the owner's
asyncio loop, independently of the lifetime of each HTTP connection.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import hmac
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Awaitable, Callable
from urllib.parse import parse_qs, urlsplit

from openrua.artifacts import ArtifactReader
from openrua.sessions.execution import Execution

# Validate external input before invoking state transitions. These are public
# session operations, not agent or robot-specific commands.
_ARGUMENTS = {
    "enqueue": {"client_id": str, "request_id": str, "text": str},
    "edit": {"message_id": str, "text": str, "revision": int},
    "withdraw": {"message_id": str},
    "interrupt": {"message_id": str},
    "resume": {"pause_id": str},
    "resolve_unknown": {"message_id": str, "note": str},
    "respond": {"request_id": str, "answers": dict},
}


def validate_command(body: dict) -> tuple[str, dict]:
    if not isinstance(body, dict) or set(body) != {"operation", "params"}:
        raise ValueError("expected operation and params")
    operation, params = body["operation"], body["params"]
    if not isinstance(operation, str) or operation not in _ARGUMENTS:
        raise ValueError("unknown session operation")
    expected = _ARGUMENTS[operation]
    if not isinstance(params, dict) or set(params) != set(expected):
        raise ValueError(f"{operation} requires: {', '.join(expected)}")
    if any(type(params[key]) is not kind for key, kind in expected.items()):
        raise ValueError("incorrect command argument type")
    if operation == "respond" and any(
            not isinstance(k, str) or not isinstance(v, list)
            or any(not isinstance(answer, str) for answer in v)
            for k, v in params["answers"].items()):
        raise ValueError("answers must map question IDs to lists of strings")
    return operation, params


class LocalServer:
    def __init__(self, execution: Execution | None, end: Callable[[], Awaitable[None]],
                 token: str, port: int = 0, request_timeout: float = 120,
                 max_body_bytes: int = 1024 * 1024,
                 assets: dict[str, tuple[str, bytes]] | None = None,
                 artifacts: ArtifactReader | None = None,
                 snapshot: Callable[[], dict] | None = None,
                 read_only: bool = False):
        if not token or request_timeout <= 0 or max_body_bytes <= 0:
            raise ValueError("token and positive request limits are required")
        self.execution, self.end = execution, end
        self.read_only = read_only
        if execution is None and snapshot is None:
            raise ValueError("a session or explicit resource snapshot is required")
        self.snapshot = snapshot if snapshot is not None else execution.session.store.snapshot
        self.artifacts = artifacts
        self.token = token
        self.request_timeout, self.max_body_bytes = request_timeout, max_body_bytes
        self.loop = asyncio.get_running_loop()
        self.ended = asyncio.Event()
        self._pending: set[concurrent.futures.Future] = set()
        self._pending_lock = threading.Lock()
        owner = self
        public_assets = dict(assets or {})

        class Handler(BaseHTTPRequestHandler):
            # One response per connection. Clients reconnect with an event cursor.
            def setup(self):
                super().setup()
                self.connection.settimeout(owner.request_timeout)

            def log_message(self, *args):
                # Request paths, headers and conversations are private session data.
                pass

            def reply(self, status, value):
                self.reply_bytes(status, "application/json; charset=utf-8", json.dumps(value, ensure_ascii=False).encode())

            def reply_bytes(self, status, mime, raw):
                try:
                    self.send_response(status)
                    self.send_header("Content-Type", mime)
                    self.send_header("Content-Length", str(len(raw)))
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-Content-Type-Options", "nosniff")
                    self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; base-uri 'none'; frame-ancestors 'none'; form-action 'self'")
                    self.send_header("Referrer-Policy", "no-referrer")
                    self.send_header("Cross-Origin-Resource-Policy", "same-origin")
                    self.end_headers()
                    self.wfile.write(raw)
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    pass

            def same_origin(self):
                host = self.headers.get("Host", "")
                origin = self.headers.get("Origin")
                if host != owner.address or (origin is not None and origin != owner.url):
                    self.reply(403, {"error": "use the local service address and same origin"})
                    return False
                return True

            def authorized(self):
                if not self.same_origin():
                    return False
                supplied = self.headers.get("Authorization", "")
                if not hmac.compare_digest(supplied.encode(), ("Bearer " + owner.token).encode()):
                    self.reply(401, {"error": "session access token required"})
                    return False
                return True

            def do_GET(self):
                target = urlsplit(self.path)
                if target.path in public_assets:
                    if self.same_origin():
                        mime, content = public_assets[target.path]
                        self.reply_bytes(200, mime, content)
                    return
                if not self.authorized():
                    return
                target = urlsplit(self.path)
                try:
                    if target.path == "/api/session" and not target.query:
                        value = owner.snapshot()
                    elif target.path == "/api/events" and owner.execution is not None:
                        query = parse_qs(target.query, strict_parsing=True)
                        if set(query) - {"after", "limit"} or any(len(v) != 1 for v in query.values()):
                            raise ValueError("use one after cursor and one limit")
                        value = owner.execution.session.store.events(
                            int(query.get("after", ["0"])[0]), int(query.get("limit", ["1000"])[0]))
                    elif target.path in ("/api/workspace/list", "/api/workspace/read") and owner.artifacts is not None:
                        query = parse_qs(target.query, strict_parsing=True, keep_blank_values=True)
                        if set(query) - {"path"} or any(len(v) != 1 for v in query.values()):
                            raise ValueError("use one relative workspace path")
                        path = query.get("path", [""])[0]
                        value = (owner.artifacts.list(path) if target.path.endswith("/list")
                                 else owner.artifacts.read(path))
                    else:
                        self.reply(404, {"error": "unknown endpoint"})
                        return
                    self.reply(200, value)
                except ValueError as exc:
                    self.reply(400, {"error": str(exc)})
                except FileNotFoundError:
                    self.reply(404, {"error": "workspace path no longer exists; refresh the directory"})
                except OSError:
                    self.reply(400, {"error": "workspace path cannot be read; symlinks and special files are not supported"})
                except Exception as exc:
                    self.reply(500, {"error": str(exc)})

            def do_POST(self):
                if not self.authorized():
                    return
                if owner.read_only:
                    self.reply(409, {"error": "retained conversation is read-only; no execution has been restarted"})
                    return
                try:
                    if self.headers.get("Transfer-Encoding") or len(self.headers.get_all("Content-Length", [])) != 1:
                        raise ValueError("provide one Content-Length header")
                    size = int(self.headers["Content-Length"])
                    if not 0 < size <= owner.max_body_bytes:
                        self.reply(413, {"error": "request body exceeds the HTTP body limit"})
                        return
                    if self.headers.get_content_type() != "application/json":
                        raise ValueError("Content-Type must be application/json")
                    body = json.loads(self.rfile.read(size))
                    if self.path == "/api/commands":
                        if owner.execution is None:
                            raise RuntimeError("this endpoint controls resources only; use the native terminal for input")
                        operation, params = validate_command(body)
                        future = owner.submit(owner.execution.command(operation, **params))
                    elif self.path == "/api/end":
                        if body != {}:
                            raise ValueError("end takes an empty object")
                        future = owner.submit(owner.end())
                    else:
                        self.reply(404, {"error": "unknown endpoint"})
                        return
                    result = future.result(timeout=owner.request_timeout)
                    if self.path == "/api/end":
                        result = owner.snapshot()
                    self.reply(200, {"result": result})
                    if self.path == "/api/end":
                        owner.loop.call_soon_threadsafe(owner.ended.set)
                except concurrent.futures.TimeoutError:
                    # A timed-out HTTP request is not a canceled command. Its
                    # accepted intent remains visible through snapshot/events.
                    self.reply(504, {"error": "result not yet known; inspect session state before retrying"})
                except (ValueError, TypeError, KeyError) as exc:
                    self.reply(400, {"error": str(exc)})
                except RuntimeError as exc:
                    self.reply(409, {"error": str(exc)})
                except Exception as exc:
                    self.reply(500, {"error": str(exc)})

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.address = f"127.0.0.1:{self.server.server_port}"
        self.url = f"http://{self.address}"
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def submit(self, coroutine):
        future = asyncio.run_coroutine_threadsafe(coroutine, self.loop)
        with self._pending_lock:
            self._pending.add(future)
        def finished(done):
            with self._pending_lock:
                self._pending.discard(done)
        future.add_done_callback(finished)
        return future

    def start(self) -> None:
        self.thread.start()

    async def close(self) -> None:
        # Run outside the event loop: handlers may be waiting for that loop.
        if self.thread.is_alive():
            await asyncio.to_thread(self.server.shutdown)
        await asyncio.to_thread(self.server.server_close)
        with self._pending_lock:
            pending = list(self._pending)
        if pending:
            await asyncio.gather(*(asyncio.wrap_future(f) for f in pending), return_exceptions=True)
