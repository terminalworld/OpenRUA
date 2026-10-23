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
    steer_delivered / steer_rejected name the message_id a steer carried. A
    turn the agent starts by itself (a background task finished) arrives as
    turn_started with a turn ID the owner never submitted and
    data.origin "agent". setting {setting, value} reports a setting now in
    effect (see ConversationProtocol.configure), setting_rejected one the
    agent refused. A setting can include choices [{value, label, description}]
    for the modes the native connection offers. commands {commands: [{name, description}]} lists the
    agent's own commands, typed as a message starting with "/". A turn
    whose text is "/clear" starts a new native conversation: the plugin
    then reports session with the new session_id and cleared true.
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

    @property
    def can_steer(self) -> bool:
        """Whether a user message can join the running turn right now, the
        way a coding agent's own terminal passes on what is typed while it
        works. Default: never; the message waits for the turn to end."""
        return False

    def steer(self, turn_id: str, message_id: str, text: str) -> Update:
        """Add ``text`` to the running turn ``turn_id``; the plugin later
        emits steer_delivered, or steer_rejected so the owner queues it
        again, naming ``message_id``."""
        raise NotImplementedError("this agent cannot take a message during a turn")

    # Plugins declare native permission values and publish their labels as
    # setting choices. The default retains legacy ask and bypass modes.
    SETTINGS = {"permissions": ("ask", "auto"), "model": None}

    def configure(self, setting: str, value: str) -> Update:
        """Change ``setting`` (SETTINGS) for what the agent does next, the
        way a coding agent's own /permissions and /model do; the plugin
        emits setting once it is in effect. Default: unsupported."""
        raise NotImplementedError("this agent's settings cannot be changed from here")


@dataclass(frozen=True)
class Conversation:
    """Native process command and its protocol. Constructing this starts nothing.

    argv uses piped stdin/stdout, not a terminal. The execution owner starts
    and owns the process; closing a client must not close this connection.
    """
    argv: list[str]
    protocol: ConversationProtocol
