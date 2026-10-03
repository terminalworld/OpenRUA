"""One execution owner drives a native conversation independently of clients.

Transport is injected. The local implementation runs the plugin's native
command with pipes; no model requests or robot-solving loop are built here.
"""

from __future__ import annotations

import asyncio
import fcntl
import json
import os
import signal
from pathlib import Path
from typing import Awaitable, Callable, Protocol

from openrua.agents.conversation import Conversation, Update
from openrua.sessions.core import Session


class Transport(Protocol):
    async def read(self) -> dict | None: ...
    async def write(self, frame: dict) -> None: ...
    async def close(self) -> None: ...


class StdioTransport:
    def __init__(self, process: asyncio.subprocess.Process):
        self.process = process

    @classmethod
    async def start(cls, argv: list[str], stderr_path: Path,
                    max_frame_bytes: int = 4 * 1024 * 1024) -> StdioTransport:
        stderr_path.parent.mkdir(parents=True, exist_ok=True)
        with stderr_path.open("ab") as errors:
            process = await asyncio.create_subprocess_exec(
                *argv, stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
                stderr=errors, start_new_session=True, limit=max_frame_bytes)
        return cls(process)

    async def read(self) -> dict | None:
        line = await self.process.stdout.readline()
        if not line:
            return None
        frame = json.loads(line)
        if not isinstance(frame, dict):
            raise ValueError("native output must be a JSON object; inspect the saved native diagnostics")
        return frame

    async def write(self, frame: dict) -> None:
        self.process.stdin.write((json.dumps(frame, ensure_ascii=False) + "\n").encode())
        await self.process.stdin.drain()

    async def close(self) -> None:
        if self.process.stdin:
            self.process.stdin.close()
        try:
            await asyncio.wait_for(self.process.wait(), 5)
        except asyncio.TimeoutError:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                await asyncio.wait_for(self.process.wait(), 5)
            except asyncio.TimeoutError:
                os.killpg(self.process.pid, signal.SIGKILL)
                await self.process.wait()


class Execution:
    def __init__(self, session: Session, conversation: Conversation,
                 transport: Callable[[], Awaitable[Transport]], owner_path: Path,
                 write_timeout: float = 30):
        if write_timeout <= 0:
            raise ValueError("write timeout must be positive")
        self._write_timeout = write_timeout
        self.session = session
        self.conversation = conversation
        self._factory = transport
        self._owner_path = owner_path
        self._owner = None
        self._transport: Transport | None = None
        self._reader: asyncio.Task | None = None
        self._gate = asyncio.Lock()
        self._closing = False
        self._usable = False

    async def start(self) -> None:
        if self._owner is not None:
            raise RuntimeError("execution owner already started")
        self._owner_path.parent.mkdir(parents=True, exist_ok=True)
        owner = self._owner_path.open("a")
        try:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            owner.close()
            raise RuntimeError("this session already has an execution owner; connect to it instead") from exc
        self._owner = owner
        try:
            if self.session.store.snapshot()["state"]["closed"]:
                raise ValueError("session is closed; start a new session to keep these records")
            self.session.recover()
            self._transport = await self._factory()
            self._usable = True
            await self._update(self.conversation.protocol.begin())
            self._reader = asyncio.create_task(self._read())
        except BaseException:
            await self.close()
            raise

    async def command(self, operation: str, **params):
        async with self._gate:
            if self._closing or self._owner is None:
                raise RuntimeError("execution owner is not accepting commands")
            if operation == "respond":
                if not self._usable:
                    raise ValueError("native connection is unavailable; reply was not sent")
                request_id = params["request_id"]
                pending = self.session.store.snapshot()["state"]["requests"].get(request_id)
                if not pending or pending["status"] != "pending":
                    raise ValueError("input request is no longer pending")
                # Validate in the plugin before committing a send intent.
                update = self.conversation.protocol.respond(request_id, params["answers"])
                self.session.claim_response(request_id)
                try:
                    await self._update(update)
                    self.session.response_sent(request_id)
                except Exception as exc:
                    self._lost(str(exc))
                    raise
                result = None
            else:
                operations = {"enqueue": self.session.enqueue, "edit": self.session.edit,
                              "withdraw": self.session.withdraw, "interrupt": self.session.interrupt,
                              "resume": self.session.resume, "resolve_unknown": self.session.resolve_unknown}
                if operation not in operations:
                    raise ValueError(f"unknown session operation: {operation}")
                result = operations[operation](**params)
            # Once accepted, a failed delivery is recorded as unknown. Return
            # the acceptance so a disconnected client does not infer rejection.
            try:
                await self._pump()
            except Exception as exc:
                self._lost(str(exc))
            return result

    async def _pump(self) -> None:
        if not self._usable:
            return
        target = self.session.take_interrupt()
        if target:
            await self._update(self.conversation.protocol.interrupt(target))
        if self.conversation.protocol.can_submit:
            message = self.session.take()
            if message:
                await self._update(self.conversation.protocol.submit(message["id"], message["text"]))

    async def _update(self, update: Update) -> None:
        for event in update.events:
            self.session.native(event)
            if event.kind == "protocol_error":
                self._usable = False
        for frame in update.outbound:
            await asyncio.wait_for(self._transport.write(frame), self._write_timeout)

    def _lost(self, reason: str) -> None:
        self._usable = False
        self.session.disconnected(reason)

    async def _read(self) -> None:
        try:
            while True:
                frame = await self._transport.read()
                async with self._gate:
                    if self._closing:
                        return
                    if frame is None:
                        self._lost("native process ended without a retained connection")
                        return
                    self.session.store.update(lambda state: (None, [{"kind": "native_frame", "data": frame}]))
                    await self._update(self.conversation.protocol.receive(frame))
                    await self._pump()
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            async with self._gate:
                if not self._closing:
                    self._lost(str(exc))

    async def close(self) -> None:
        """Close the native connection, not the robot or the retained session.

        The resource owner separately stops its robot/sandbox before recording
        session closure. A native connection ending is not a physical stop.
        """
        async with self._gate:
            self._closing = True
            self._usable = False
            try:
                try:
                    if self._reader:
                        self._reader.cancel()
                        try:
                            await self._reader
                        except asyncio.CancelledError:
                            pass
                finally:
                    if self._transport:
                        await self._transport.close()
                    if self._owner:
                        self.session.disconnected("native connection closed by its owner")
            finally:
                if self._owner:
                    self._owner.close()
                    self._owner = None
