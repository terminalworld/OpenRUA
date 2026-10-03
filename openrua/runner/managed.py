"""Combine interactive robot resources with a native conversation owner."""

from __future__ import annotations

import asyncio
from pathlib import Path

from openrua import agents
from openrua.artifacts import ArtifactReader, WorkspaceFiles
from openrua.config import paths
from openrua.errors import UnavailableError
from openrua.runner import live_state
from openrua.runner.live import LiveRobot, RobotRequest, open_robot
from openrua.sessions.core import Session, initial_state
from openrua.sessions.execution import Execution, StdioTransport
from openrua.sessions.store import SQLiteStore


class ManagedSession:
    def __init__(self, robot: LiveRobot, execution: Execution, store: SQLiteStore,
                 artifacts: ArtifactReader | None = None):
        self.robot, self.execution, self.store = robot, execution, store
        self.artifacts = artifacts
        self._end_lock = asyncio.Lock()
        self._ended = False
        self._native_stopped = False
        self._robot_stopped = False

    async def end(self) -> None:
        """Stop owned execution resources and retain all conversation materials."""
        async with self._end_lock:
            if self._ended:
                return
            errors = []
            if not self._native_stopped:
                try:
                    await self.execution.close()
                except Exception as exc:
                    errors.append(f"native process: {exc}")
                else:
                    self._native_stopped = True
            if not self._robot_stopped:
                try:
                    await asyncio.to_thread(self.robot.power_off)
                except Exception as exc:
                    errors.append(f"robot resources: {exc}")
                else:
                    self._robot_stopped = True
            if errors:
                raise UnavailableError("; ".join(errors), hint="inspect the service log and retry ending the session")
            self.execution.session.close()
            self._ended = True


async def start(request: RobotRequest) -> ManagedSession:
    """Start through the existing robot/agent plugin path, with one owner.

    Robot startup is synchronous just as in `up`. Once ready, the native
    reader and clients run on the event loop. No model task is submitted
    until a user sends a message.
    """
    robot = open_robot(request)
    store = None
    execution = None
    try:
        facts = live_state.load(request.name, request.home)
        adapter = agents.get(facts.get("agent_spec", facts["agent"]), request.home, version=facts.get("agent_version"))
        conversation = adapter.conversation(facts["sandbox"], facts["model"], facts["proxy"],
                                            options=facts["options"])
        if conversation is None:
            raise UnavailableError(
                f"{adapter.name} does not support structured conversations",
                hint="use this agent with openrua up and openrua agent instead")
        directory = paths.sandbox_dir(request.name, request.home)
        store = SQLiteStore(directory / "conversation.sqlite", initial_state())
        session = Session(store)
        async def transport():
            return await StdioTransport.start(conversation.argv, directory / "agent.stderr.log")
        execution = Execution(session, conversation, transport, directory / "conversation.lock")
        await execution.start()
        return ManagedSession(robot, execution, store, WorkspaceFiles(Path(facts["workspace"])))
    except BaseException:
        try:
            if execution is not None:
                await execution.close()
        finally:
            try:
                robot.power_off()
            finally:
                if store is not None:
                    store.close()
        raise
