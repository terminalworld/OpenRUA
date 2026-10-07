"""Optional native conversation contract, independent of clients and transport.

The owner serializes calls, writes outbound JSON frames in order, and feeds
received frames back to the protocol. It stores raw frames separately from
normalized events. This layer neither queues user tasks nor controls robots.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class Event:
    """A conversation event; turn_id is the caller's identity, never inferred by a UI.

    A finished turn describes agent execution, not robot task success.
    Native events remain available for diagnostics without vendor parsing in clients.
    input_required carries request_id and questions with id, text, choices and
    optional boolean multiple, allow_other and secret fields (default false).
    Replies contain choice labels or, when allowed, one free-text answer.
    Plugins map these fields to their native question and answer formats.
    """
    kind: str
    turn_id: str | None = None
    data: dict[str, Any] = field(default_factory=dict)


@dataclass
class Update:
    """Frames to write and events to record, in their respective order."""
    outbound: list[dict[str, Any]] = field(default_factory=list)
    events: list[Event] = field(default_factory=list)

    def extend(self, other: Update) -> None:
        self.outbound.extend(other.outbound)
        self.events.extend(other.events)


class ConversationProtocol(ABC):
    """One native connection, owned by a single serialized execution loop.

    submit accepts one outstanding turn. The owner, not the plugin, queues
    later user messages. interrupt targets that exact caller-supplied turn ID;
    an acknowledgement alone must never emit turn_finished. After transport
    loss the owner marks outstanding work unknown and never silently replays it.
    """

    @property
    @abstractmethod
    def can_submit(self) -> bool:
        """Whether the native connection can accept a new turn right now."""
        ...

    @abstractmethod
    def begin(self) -> Update: ...

    @abstractmethod
    def submit(self, turn_id: str, text: str) -> Update: ...

    @abstractmethod
    def interrupt(self, turn_id: str) -> Update: ...

    @abstractmethod
    def receive(self, frame: dict[str, Any]) -> Update: ...

    @abstractmethod
    def respond(self, request_id: str, answers: dict[str, list[str]]) -> Update:
        """Answer the questions supplied in an input_required event by their IDs."""
        ...


@dataclass(frozen=True)
class Conversation:
    """Native process command and its protocol. Constructing this starts nothing.

    argv uses piped stdin/stdout, not a terminal. The execution owner starts
    and owns the process; closing a client must not close this connection.
    """
    argv: list[str]
    protocol: ConversationProtocol
