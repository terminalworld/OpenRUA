"""One execution owner drives a native conversation independently of clients.

Transport is injected. The local implementation runs the plugin's native
command with pipes; no model requests or robot-solving loop are built here.
"""

from __future__ import annotations

import asyncio
import fcntl
import json
import os
import shutil
import signal
import tempfile
from pathlib import Path
from typing import Awaitable, Callable, Protocol

from openrua.agents.conversation import Conversation, Update
from openrua.sessions.core import Session


class Transport(Protocol):
    async def read(self) -> dict | None: ...
    async def write(self, frame: dict) -> None: ...
    async def close(self) -> None: ...


class StdioTransport:
    def __init__(self, process: asyncio.subprocess.Process, invalid_output: Path | None = None,
                 buffer_bytes: int = 64 * 1024):
        if buffer_bytes <= 0:
            raise ValueError('the native stream buffer must be positive')
        self.process = process
        self.invalid_output = invalid_output
        self.buffer_bytes = buffer_bytes

    @classmethod
    async def start(cls, argv: list[str], stderr_path: Path,
                    buffer_bytes: int = 64 * 1024,
                    cwd: Path | None = None) -> StdioTransport:
        if buffer_bytes <= 0:
            raise ValueError('the native stream buffer must be positive')
        stderr_path.parent.mkdir(parents=True, exist_ok=True)
        with stderr_path.open("ab") as errors:
            process = await asyncio.create_subprocess_exec(
                *argv, stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
                stderr=errors, start_new_session=True, limit=buffer_bytes, cwd=cwd)
        return cls(process, stderr_path.with_suffix('.invalid-output.log'), buffer_bytes)

    async def read(self) -> dict | None:
        while True:
            # The pipe limit bounds buffered chunks, not native message size.
            # Spill unfinished large messages to disk while waiting for the
            # delimiter. Decoding still needs one complete JSON object.
            with tempfile.SpooledTemporaryFile(max_size=self.buffer_bytes) as raw:
                nonblank = False
                while True:
                    complete = True
                    try:
                        chunk = await self.process.stdout.readuntil(b'\n')
                    except asyncio.LimitOverrunError as exc:
                        chunk = await self.process.stdout.readexactly(exc.consumed)
                        complete = False
                    except asyncio.IncompleteReadError as exc:
                        chunk = exc.partial
                        if not chunk and not raw.tell():
                            return None
                    raw.write(chunk)
                    nonblank = nonblank or bool(chunk.strip())
                    if complete:
                        break
                if not nonblank:
                    continue
                raw.seek(0)
                try:
                    frame = json.load(raw)
                    if isinstance(frame, dict):
                        return frame
                except (ValueError, UnicodeError):
                    pass
                where = 'the native process output'
                if self.invalid_output is not None:
                    fd = os.open(self.invalid_output, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
                    raw.seek(0)
                    with os.fdopen(fd, 'ab') as stream:
                        shutil.copyfileobj(raw, stream, self.buffer_bytes)
                    where = str(self.invalid_output)
                raise ValueError(f'The agent returned a response that is not a JSON object. OpenRUA cannot read it. Check {where}. '
                                 'The agent is disconnected. Use /resume in the terminal to reopen this conversation.')

    async def write(self, frame: dict) -> None:
        self.process.stdin.write((json.dumps(frame, ensure_ascii=False) + "\n").encode())
        await self.process.stdin.drain()

    async def close(self) -> None:
        # The owner has stopped reading frames. Drain the pipe in bounded
        # chunks so buffered stdout cannot prevent the child from exiting.
        async def drain():
            while await self.process.stdout.read(self.buffer_bytes):
                pass
        draining = asyncio.create_task(drain())
        async def finished():
            await self.process.wait()
            await asyncio.shield(draining)
        if self.process.stdin:
            self.process.stdin.close()
        try:
            try:
                await asyncio.wait_for(finished(), 5)
            except asyncio.TimeoutError:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    await asyncio.wait_for(finished(), 5)
                except asyncio.TimeoutError:
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    await asyncio.wait_for(finished(), 5)
        finally:
            draining.cancel()
            await asyncio.gather(draining, return_exceptions=True)


class Execution:
    def __init__(self, session: Session, conversation: Conversation,
                 transport: Callable[[], Awaitable[Transport]], owner_path: Path,
                 write_timeout: float = 30, diagnostics: Callable[[], str] | None = None):
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
        self._handover = None
        self._usable = False
        # What the native process last said on its own (its stderr), added
        # to the reason when it ends: the cause is usually there.
        self._diagnostics = diagnostics
        # What the resource owner also does when a turn is interrupted (a
        # robot's motions canceled), started without holding the queue.
        self.on_interrupt: Callable[[], Awaitable[None]] | None = None

    @property
    def usable(self) -> bool:
        """The native connection is open, including its startup handshake."""
        return self._usable and not self._closing

    async def start(self, *, reopen: bool = False, context_note: str | None = None) -> None:
        if self._owner is not None:
            raise RuntimeError("The agent for this session has already started.")
        self._owner_path.parent.mkdir(parents=True, exist_ok=True)
        owner = self._owner_path.open("a")
        try:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            owner.close()
            raise RuntimeError("Another agent process already controls this session. Connect to it instead.") from exc
        self._owner = owner
        try:
            if reopen:
                if context_note is None:
                    self.session.reopen()
                else:
                    self.session.reopen(context_note)
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
            coordination = {"enqueue": self.session.enqueue, "edit": self.session.edit,
                            "withdraw": self.session.withdraw, "release": self.session.release,
                            "resolve_unknown": self.session.resolve_unknown}
            if self._closing or self._owner is None:
                if self._handover and operation in coordination:
                    return coordination[operation](**params)
                raise RuntimeError("The agent for this session cannot accept commands.")
            if operation == "respond":
                if not self._usable:
                    raise ValueError("The agent is not connected. Your reply was not sent.")
                request_id = params["request_id"]
                pending = self.session.store.snapshot()["state"]["requests"].get(request_id)
                if not pending or pending["status"] != "pending":
                    raise ValueError("The agent is no longer waiting for this answer.")
                # Validate in the plugin before committing a send intent.
                if pending.get('reply_as_message'):
                    text = self.conversation.protocol.response_message(request_id, params['answers'])
                    result = self.session.enqueue_response(request_id, text, params['answers'])
                else:
                    update = self.conversation.protocol.respond(request_id, params["answers"])
                    self.session.claim_response(request_id)
                    try:
                        await self._update(update)
                        self.session.response_sent(request_id)
                    except Exception as exc:
                        self._lost(str(exc))
                        raise
                    result = None
            elif operation == "configure":
                if not self._usable:
                    raise ValueError("The agent is not connected. No settings were changed.")
                await self._update(self.conversation.protocol.configure(params["setting"],
                                                                        params["value"]))
                result = None
            elif operation == 'select_model':
                if not self._usable:
                    raise ValueError('The agent is not connected. No settings were changed.')
                await self._update(self.conversation.protocol.select_model(params['model'], params['effort']))
                result = None
            else:
                operations = {**coordination, "interrupt": self.session.interrupt,
                              "resume": self.session.resume}
                if operation not in operations:
                    raise ValueError(f"unknown session operation: {operation}")
                result = operations[operation](**params)
            # Once accepted, a failed delivery is recorded as unknown. Return
            # the acceptance so a disconnected client does not infer rejection.
            try:
                await self._pump()
            except Exception as exc:
                self._lost(str(exc))
            if operation == "interrupt" and self.on_interrupt is not None:
                asyncio.create_task(self.on_interrupt())
            return result

    async def _pump(self) -> None:
        if not self._usable:
            return
        target = self.session.take_interrupt()
        if target:
            await self._update(self.conversation.protocol.interrupt(target))
        protocol = self.conversation.protocol
        if protocol.can_submit:
            message = self.session.take()
            if message:
                text = message['text']
                if message.get('context_note'):
                    text = '[Runtime update]\n' + message['context_note'] + '\n\n[User message]\n' + text
                await self._update(protocol.submit(message["id"], text))
        # What is typed while a turn runs joins it, where the agent can take
        # it, the way a coding agent's own terminal passes it on.
        while protocol.can_steer:
            message = self.session.take_steer()
            if not message:
                break
            text = message['text']
            if message.get('context_note'):
                text = '[Runtime update]\n' + message['context_note'] + '\n\n[User message]\n' + text
            await self._update(protocol.steer(message["joined"], message["id"], text))

    async def robot_status(self, status: str, reason: str = '', *, hold: bool = False,
                           interrupt: bool = False) -> None:
        """Notify the conversation and send any requested interrupt under its gate."""
        async with self._gate:
            if self._closing:
                return
            self.session.robot_status(status, reason, hold=hold, interrupt=interrupt)
            try:
                await self._pump()
            except Exception as exc:
                self._lost(str(exc))

    async def wake(self) -> None:
        """Reconsider delivery after a resource owner's internal handover."""
        async with self._gate:
            if not self._closing:
                try:
                    await self._pump()
                except Exception as exc:
                    self._lost(str(exc))

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
                        said = self._diagnostics() if self._diagnostics else ""
                        self._lost("native process ended without a retained connection"
                                   + (f"; it said: {said}" if said else ""))
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

    async def close(self, handover: str | None = None) -> None:
        """Close the native connection, not the robot or the retained session.

        The resource owner separately stops its robot/sandbox before recording
        session closure. A native connection ending is not a physical stop.
        ``handover`` names a planned move to new resources between turns:
        recorded as such, with nothing paused.
        """
        async with self._gate:
            first_close = not self._closing
            if first_close and self._owner and handover:
                # Validate a planned boundary before closing anything. A late
                # native turn must not be killed as a supposedly idle handover.
                self.session.handed_over(handover)
                self._handover = handover
            self._closing = True
            self._usable = False
            try:
                # Record uncertainty even if stopping the native process fails.
                # Later clients must not see a still-running, connected session.
                if first_close and self._owner:
                    if not handover:
                        self.session.disconnected("native connection closing by its owner")
            finally:
                try:
                    if self._reader:
                        self._reader.cancel()
                        try:
                            await self._reader
                        except asyncio.CancelledError:
                            pass
                        self._reader = None
                finally:
                    if self._transport:
                        await self._transport.close()
                        self._transport = None
                    # Keep ownership on cleanup failure. A retry closes the same
                    # transport before another owner may start a native process.
                    if self._owner:
                        self._owner.close()
                        self._owner = None
