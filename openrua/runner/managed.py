"""Own a persistent conversation and its independently maintained resources."""

from __future__ import annotations

import asyncio
import fcntl
import time
from pathlib import Path
from typing import Callable

from openrua import agents
from openrua.artifacts import ArtifactReader, WorkspaceFiles
from openrua.config import paths
from openrua.errors import UnavailableError
from openrua.runner import live_state
from openrua.runner.live import LiveRobot, RobotRequest, open_robot
from openrua.sessions.core import Session, initial_state
from openrua.sessions.execution import Execution, StdioTransport
from openrua.sessions.store import SQLiteStore


def _last_words(path: Path, limit: int = 300) -> str:
    """The last nonempty line a native process wrote to its stderr log."""
    try:
        lines = [line.strip() for line in path.read_text(errors="replace").splitlines() if line.strip()]
    except OSError:
        return ""
    return lines[-1][-limit:] if lines else ""


class ManagedSession:
    def __init__(self, robot: LiveRobot, execution: Execution, store: SQLiteStore,
                 artifacts: ArtifactReader | None = None, name: str = "", home: Path | None = None):
        self.robot, self.execution, self.store = robot, execution, store
        self.artifacts = artifacts
        self.name, self.home = name, home
        self._end_lock = asyncio.Lock()
        self._ended = False
        self._native_stopped = False
        self._robot_stopped = False
        self._watch: asyncio.Task | None = None
        self.resource_lock = None
        self.maintenance = None
        self.on_execution = None

    def watch_robot(self, interval_s: float = 2.0) -> None:
        """Notice when the robot's resources end under the session (a
        driver that crashed after an emergency stop, say): say so in the
        service log, hold earlier work and leave repair dialogue usable."""
        async def watch():
            while True:
                await asyncio.sleep(interval_s)
                if self.maintenance and self.maintenance.task and not self.maintenance.task.done():
                    continue
                reason = await asyncio.to_thread(self.robot.stopped)
                if reason:
                    where = f"; read {self.robot.robot_log}" if self.robot.robot_log else ""
                    current = self.store.snapshot()['state'].get('robot') or {}
                    if current.get('status') != 'stopped' or current.get('reason') != reason:
                        print(f"[robot] {reason}{where}", flush=True)
                        self.robot.cameras = ({}, None)
                        await self.execution.robot_status('stopped', reason, hold=True, interrupt=True)
        self._watch = asyncio.create_task(watch())

    async def cancel_motion(self) -> None:
        """What an interrupt also does: the agent's turn stops through its
        own CLI, and the robot's motions stop here, whatever started them."""
        outcome = await asyncio.to_thread(self.robot.stop_motion)
        if outcome:
            self.execution.session.motion_canceled(outcome)

    async def move_to(self, open_resources: Callable[[], LiveRobot], reason: str) -> None:
        """Move the conversation to new resources between turns: the
        agent's process stops, the old resources stop, the new ones start
        (``open_resources``), and a new process goes on with the same
        native conversation. A failure leaves the old resources stopped
        and no agent process; moving again (back, say) recovers."""
        async with self._end_lock:
            if self._ended:
                raise RuntimeError("the session has ended")
            if self._watch is not None:
                self._watch.cancel()
                self._watch = None
            await self.execution.close(handover=reason)
            await asyncio.to_thread(self.robot.power_off)
            self.robot = await asyncio.to_thread(open_resources)
            facts = live_state.load(self.name, self.home)
            self.artifacts = WorkspaceFiles(Path(facts["workspace"]))
            self.execution = await _open_execution(self.name, self.home, self.store, resume=True)
            self.execution.on_interrupt = self.cancel_motion
            if self.on_execution:
                self.on_execution(self.execution)
            self.watch_robot()

    async def reconnect(self, body: dict) -> dict:
        """Reconnect the retained native conversation without changing robot resources."""
        if body != {}:
            raise ValueError('reconnect takes an empty object')
        async with self._end_lock:
            if (self._ended or self._native_stopped or self._robot_stopped
                    or (self.maintenance is not None and self.maintenance.closing)):
                raise UnavailableError('this session is stopping or has stopped',
                                       hint='finish stopping it before reopening the saved conversation')
            if self.execution.usable:
                return self.store.snapshot()
            if self.maintenance and self.maintenance.task and not self.maintenance.task.done():
                raise UnavailableError('the robot connection is being updated',
                                       hint='wait for the connection operation to finish, then reopen this conversation')
            current = self.store.snapshot()['state']
            sent = any(message['status'] not in {'queued', 'withdrawn'} for message in current['messages'])
            if sent and not current.get('native_session'):
                raise UnavailableError('the native conversation identity was not saved',
                                       hint='inspect the retained messages and agent log before starting a new conversation')
            # Close the former connection before taking its exclusive lock.
            # Disconnected work stays unknown or paused in Session.recover.
            await self.execution.close()
            self.execution = await _open_execution(self.name, self.home, self.store, resume=True)
            self.execution.on_interrupt = self.cancel_motion
            if self.on_execution:
                self.on_execution(self.execution)
            return self.store.snapshot()

    async def end(self) -> None:
        """Stop owned execution resources and retain all conversation materials."""
        if self.maintenance is not None:
            started = time.monotonic()
            await self.maintenance.close()
            print(f'[stop] Pending robot work took {time.monotonic() - started:.2f}s to finish.', flush=True)
        async with self._end_lock:
            if self._ended:
                return
            if self._watch is not None:
                self._watch.cancel()
            errors = []
            if not self._native_stopped:
                started = time.monotonic()
                try:
                    await self.execution.close()
                except Exception as exc:
                    errors.append(f"native process: {exc}")
                else:
                    self._native_stopped = True
                finally:
                    print(f'[stop] Agent shutdown took {time.monotonic() - started:.2f}s.', flush=True)
            if not self._robot_stopped:
                started = time.monotonic()
                try:
                    await asyncio.to_thread(self.robot.power_off)
                except Exception as exc:
                    errors.append(f"robot resources: {exc}")
                else:
                    self._robot_stopped = True
                finally:
                    print(f'[stop] Resource cleanup took {time.monotonic() - started:.2f}s.', flush=True)
            if errors:
                raise UnavailableError("; ".join(errors), hint="inspect the service log and retry ending the session")
            self.execution.session.close()
            self._ended = True
            if self.resource_lock is not None:
                self.resource_lock.close()
                self.resource_lock = None


async def _open_execution(name: str, home: Path, store: SQLiteStore,
                          resume: bool = False, reopen: bool = False) -> Execution:
    """The agent's native process for this session's resources, started.
    ``resume`` continues the session's native conversation (its settings
    too) instead of beginning one."""
    facts = live_state.load(name, home)
    adapter = agents.get(facts.get("agent_spec", facts["agent"]), home, version=facts.get("agent_version"))
    current = store.snapshot()["state"] if resume else {}
    native_session = current.get('native_session') if not (reopen and not current.get('messages')) else None
    settings = current.get("settings") or {}
    conversation = adapter.conversation(
        facts["sandbox"], settings.get("model") or facts["model"], facts["proxy"],
        options={**facts['options'], **({'effort': settings['effort']} if settings.get('effort') else {})}, limits=facts.get("limits"),
        permissions=agents.saved_permissions(adapter, facts, current),
        **({"session_id": native_session} if native_session else {}))
    if conversation is None:
        raise UnavailableError(
            f"{adapter.name} does not support structured conversations",
            hint="use this agent with openrua up and openrua agent instead")
    directory = paths.sandbox_dir(name, home)
    stderr = directory / "agent.stderr.log"
    async def transport():
        return await StdioTransport.start(conversation.argv, stderr)
    execution = Execution(Session(store), conversation, transport, directory / "conversation.lock",
                          diagnostics=lambda: _last_words(stderr))
    await execution.start(reopen=reopen,
                          context_note=('The robot connection has restarted. Read /robot/machine.yaml and acquire '
                                        'current observations before continuing; the physical scene may have changed. '
                                        'Past actions and queued instructions have not been replayed.') if reopen else None)
    return execution


async def start(request: RobotRequest, open_resources: Callable | None = None) -> ManagedSession:
    """Start through the existing robot/agent plugin path, with one owner.

    The agent environment and native reader become available before the
    robot connects in the background. No model task is submitted until a
    user sends a message. ``open_resources`` takes the request and
    returns the resources' handle (a robot by default; the connecting
    terminal for ``openrua connect``).
    """
    lock_path = paths.launch_dir(request.name, request.home) / 'resources.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    resource_lock = lock_path.open('a')
    try:
        fcntl.flock(resource_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        resource_lock.close()
        raise UnavailableError('this session already has a resource owner',
                               hint='connect to it using --resume') from exc
    continuing = getattr(request, 'continuing', False)
    if continuing:
        try:
            previous = live_state.load(request.name, request.home, require_running=False)
            if not request.project or previous.get('project') != str(request.project.resolve()) or previous.get('status') != 'stopped':
                raise ValueError('only a confirmed stopped conversation in this project can be reopened')
            database = SQLiteStore.open_readonly(paths.sandbox_dir(request.name, request.home) / 'conversation.sqlite')
            try:
                if not database.snapshot()['state']['closed']:
                    raise ValueError('conversation is not ended; inspect its existing owner')
            finally:
                database.close()
        except BaseException:
            resource_lock.close()
            raise
    try:
        robot = open_resources(request) if open_resources else open_robot(request, interactive=True)
    except BaseException:
        resource_lock.close()
        raise
    store = None
    execution = None
    try:
        facts = live_state.load(request.name, request.home)
        directory = paths.sandbox_dir(request.name, request.home)
        store = SQLiteStore(directory / "conversation.sqlite", initial_state())
        execution = await _open_execution(request.name, request.home, store,
                                           resume=continuing, reopen=continuing)
        managed = ManagedSession(robot, execution, store, WorkspaceFiles(Path(facts["workspace"])),
                                 name=request.name, home=request.home)
        execution.on_interrupt = managed.cancel_motion
        managed.resource_lock = resource_lock
        managed.watch_robot()
        from openrua.runner.maintenance import Maintenance
        managed.maintenance = Maintenance(managed)
        try:
            await managed.maintenance.begin()
        except BaseException:
            await managed.maintenance.close()
            raise
        return managed
    except BaseException:
        try:
            if execution is not None:
                await execution.close()
        finally:
            try:
                robot.power_off()
            finally:
                if store is not None:
                    if continuing:
                        Session(store).close()
                    store.close()
                resource_lock.close()
        raise
