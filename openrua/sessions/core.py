"""Shared queue semantics. All state transitions commit before external effects.

The core knows no agent names, robot commands, client transports or SQL. One
execution owner delivers turns; any number of clients may enqueue through it.
"""

from __future__ import annotations

from dataclasses import asdict
from uuid import uuid4

from openrua.agents.conversation import Event
from openrua.sessions.store import Store


def initial_state() -> dict:
    return {"format": 1, "pause_id": None, "messages": [], "active": None, "paused": False, "connected": False,
            "closed": False, "native_session": None, "requests": {}}


def _message(state: dict, message_id: str) -> dict:
    for message in state["messages"]:
        if message["id"] == message_id:
            return message
    raise ValueError("unknown message")


def _open(state: dict) -> None:
    if state["closed"]:
        raise ValueError("session is closed; its records are retained")


def _pause(state: dict) -> None:
    state["paused"] = True
    state["pause_id"] = str(uuid4())


def _event(kind: str, **data) -> dict:
    return {"kind": kind, "data": data}


class Session:
    def __init__(self, store: Store):
        self.store = store
        if store.snapshot()["state"].get("format") != 1:
            raise ValueError("unsupported session format; use the OpenRUA version that wrote this record")

    def enqueue(self, client_id: str, request_id: str, text: str) -> dict:
        if not client_id or not request_id or not text.strip():
            raise ValueError("client ID, request ID and message text are required")
        def change(state):
            # Retries return the original acceptance even after edits or closure.
            for message in state["messages"]:
                if (message["client_id"], message["request_id"]) == (client_id, request_id):
                    if message["original_text"] != text:
                        raise ValueError("request ID was already used with different content")
                    return message, []
            _open(state)
            message = {"id": str(uuid4()), "client_id": client_id, "request_id": request_id,
                       "original_text": text, "text": text, "revision": 0, "status": "queued",
                       "cancel_requested": False, "cancel_sent": False}
            state["messages"].append(message)
            return message, [_event("message_accepted", message=message)]
        return self.store.update(change)

    def edit(self, message_id: str, text: str, revision: int) -> dict:
        if not text.strip():
            raise ValueError("message text is required")
        def change(state):
            _open(state)
            message = _message(state, message_id)
            if message["status"] != "queued" or message["revision"] != revision:
                raise ValueError("message changed or has already been dispatched; refresh before editing")
            message["text"] = text
            message["revision"] += 1
            return message, [_event("message_edited", message=message)]
        return self.store.update(change)

    def withdraw(self, message_id: str) -> None:
        def change(state):
            _open(state)
            message = _message(state, message_id)
            if message["status"] == "withdrawn":
                return None, []
            if message["status"] != "queued":
                raise ValueError("only queued messages can be withdrawn")
            message["status"] = "withdrawn"
            return None, [_event("message_withdrawn", message_id=message_id)]
        self.store.update(change)

    def take(self) -> dict | None:
        """Persist delivery intent before the owner submits to the native process."""
        def change(state):
            if state["closed"] or state["paused"] or state["active"] or not state["connected"]:
                return None, []
            for message in state["messages"]:
                if message["status"] == "queued":
                    message["status"] = "dispatching"
                    state["active"] = message["id"]
                    return message, [_event("dispatch_started", message_id=message["id"])]
            return None, []
        return self.store.update(change)

    def interrupt(self, message_id: str) -> None:
        """Pause first. A separate claim sends cancellation once native identity exists."""
        def change(state):
            _open(state)
            if state["active"] != message_id:
                raise ValueError("interrupt target is no longer the active turn")
            message = _message(state, message_id)
            if message["cancel_requested"]:
                return None, []
            message["cancel_requested"] = True
            _pause(state)
            return None, [_event("interrupt_requested", message_id=message_id)]
        self.store.update(change)

    def take_interrupt(self) -> str | None:
        def change(state):
            if not state["active"]:
                return None, []
            message = _message(state, state["active"])
            if message["status"] != "running" or not message["cancel_requested"] or message["cancel_sent"]:
                return None, []
            message["cancel_sent"] = True
            return message["id"], [_event("interrupt_sent", message_id=message["id"])]
        return self.store.update(change)

    def native(self, event: Event) -> None:
        def change(state):
            record = {"kind": "agent_event", "data": asdict(event)}
            if state["closed"]:
                return None, [record]
            if event.kind in {"ready", "session"}:
                identity = event.data.get("session_id")
                if not isinstance(identity, str) or not identity:
                    raise ValueError("native readiness requires a conversation identity")
                if state["native_session"] and identity != state["native_session"]:
                    raise ValueError("native conversation identity changed")
                state["native_session"] = identity
                if event.kind == "ready":
                    state["connected"] = True
            elif event.kind == "protocol_error":
                self._unknown(state, "native protocol error")
            elif event.turn_id and event.turn_id == state["active"]:
                message = _message(state, event.turn_id)
                if event.kind == "turn_started":
                    message["status"] = "running"
                elif event.kind == "turn_finished":
                    status = event.data["status"]
                    if status not in {"completed", "failed", "interrupted"}:
                        raise ValueError("unsupported terminal status")
                    message["status"] = status
                    state["active"] = None
                    state["requests"] = {}
                    if status != "completed" or message["cancel_requested"]:
                        if not state["paused"]:
                            _pause(state)
                elif event.kind == "input_required":
                    state["requests"][event.data["request_id"]] = {
                        "turn_id": event.turn_id, "status": "pending", **event.data}
                elif event.kind == "input_cancelled":
                    state["requests"].pop(event.data["request_id"], None)
                elif event.kind == "unsupported_request":
                    _pause(state)
            return None, [record]
        self.store.update(change)

    def claim_response(self, request_id: str) -> None:
        """Record input delivery before forwarding; never retry an uncertain reply."""
        def change(state):
            _open(state)
            request = state["requests"].get(request_id)
            if not request or request["status"] != "pending":
                raise ValueError("input request is no longer pending; refresh its status")
            request["status"] = "sending"
            # Do not persist answers to secret questions. The reply itself is
            # delivered through the live owner, separate from the event journal.
            return None, [_event("input_reply_started", request_id=request_id)]
        self.store.update(change)

    def response_sent(self, request_id: str) -> None:
        def change(state):
            state["requests"].pop(request_id, None)
            return None, [_event("input_reply_sent", request_id=request_id)]
        self.store.update(change)

    @staticmethod
    def _unknown(state: dict, reason: str) -> None:
        state["connected"] = False
        _pause(state)
        if state["active"]:
            message = _message(state, state["active"])
            message["status"] = "unknown"
            message["uncertainty"] = reason
            state["active"] = None
        state["requests"] = {}

    def disconnected(self, reason: str) -> None:
        def change(state):
            self._unknown(state, reason)
            return None, [_event("connection_lost", reason=reason)]
        self.store.update(change)

    def recover(self) -> None:
        """Called by a new owner; client reconnection never invokes this."""
        def change(state):
            if state["closed"]:
                return None, []
            if state["active"]:
                self._unknown(state, "previous execution owner ended without a terminal result")
            state["connected"] = False
            return None, [_event("owner_started")]
        self.store.update(change)

    def resolve_unknown(self, message_id: str, note: str) -> None:
        if not note.strip():
            raise ValueError("record how the uncertain execution was checked")
        def change(state):
            message = _message(state, message_id)
            if message["status"] != "unknown":
                raise ValueError("only uncertain executions need reconciliation")
            message["status"] = "reconciled"
            message["resolution"] = note
            return None, [_event("execution_reconciled", message_id=message_id, note=note)]
        self.store.update(change)

    def resume(self, pause_id: str) -> None:
        def change(state):
            _open(state)
            if state["active"] or not state["connected"] or any(
                    m["status"] == "unknown" for m in state["messages"]):
                raise ValueError("wait for the active turn and reconcile uncertain execution before resuming")
            if not state["paused"] and pause_id == state["pause_id"]:
                return None, []
            if not pause_id or pause_id != state["pause_id"]:
                raise ValueError("pause state changed; refresh before resuming")
            state["paused"] = False
            return None, [_event("queue_resumed")]
        self.store.update(change)

    def close(self) -> None:
        """Record closure after the owner stops resources, keeping uncertain outcomes."""
        def change(state):
            if state["closed"]:
                return None, []
            if state["active"]:
                self._unknown(state, "session ended during execution")
            state["closed"] = state["paused"] = True
            state["connected"] = False
            return None, [_event("session_closed")]
        self.store.update(change)
