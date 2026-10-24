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
    # pause_reason and robot came later; a record without them reads as no
    # reason given and no robot event seen.
    return {"format": 1, "pause_id": None, "messages": [], "active": None, "paused": False, "connected": False,
            "closed": False, "native_session": None, "requests": {}, "pause_reason": None,
            "robot": None}


def _message(state: dict, message_id: str) -> dict:
    for message in state["messages"]:
        if message["id"] == message_id:
            return message
    raise ValueError("unknown message")


def _open(state: dict) -> None:
    if state["closed"]:
        raise ValueError("session is closed; its records are retained")


def _pause(state: dict, reason: str) -> None:
    """Hold the queue, saying why, so a client can tell a person what
    stopped and what to do."""
    state["paused"] = True
    state["pause_id"] = str(uuid4())
    state["pause_reason"] = reason


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

    def release(self, message_id: str, revision: int) -> dict:
        """Explicitly reconsider one instruction held across a resource change."""
        def change(state):
            _open(state)
            message = _message(state, message_id)
            if message['status'] != 'queued' or message['revision'] != revision:
                raise ValueError('message changed or was dispatched; refresh before releasing it')
            if not message.get('held_reason'):
                raise ValueError('this instruction is not held')
            message.pop('held_reason')
            message['revision'] += 1
            return message, [_event('message_released', message=message)]
        return self.store.update(change)

    def take(self) -> dict | None:
        """Persist delivery intent before the owner submits to the native process."""
        def change(state):
            if (state["closed"] or state["paused"] or state["active"] or not state["connected"]
                    or state.get('handover_pending')):
                return None, []
            for message in state["messages"]:
                if message["status"] == "queued" and not message.get('held_reason'):
                    if not message['text'].lstrip().startswith('/'):
                        notes = [state.pop(key) for key in ('context_note', 'robot_note') if state.get(key)]
                        if notes:
                            message['context_note'] = '\n'.join(notes)
                    message["status"] = "dispatching"
                    state["active"] = message["id"]
                    return message, [_event("dispatch_started", message_id=message["id"])]
            return None, []
        return self.store.update(change)

    def interrupt(self, message_id: str) -> None:
        """Stop the running turn. Queued messages go on afterwards, as in a
        coding agent's own terminal; a separate claim sends cancellation
        once native identity exists."""
        def change(state):
            _open(state)
            if state["active"] != message_id:
                raise ValueError("interrupt target is no longer the active turn")
            message = _message(state, message_id)
            if message["cancel_requested"]:
                return None, []
            message["cancel_requested"] = True
            return None, [_event("interrupt_requested", message_id=message_id)]
        self.store.update(change)

    def take_steer(self) -> dict | None:
        """The next queued message, marked to join the running turn, when
        there is a running turn to join. Persisted before the owner sends."""
        def change(state):
            if (state["closed"] or state["paused"] or not state["active"] or not state["connected"]
                    or state.get('handover_pending')):
                return None, []
            active = _message(state, state["active"])
            if active["status"] != "running" or active["cancel_requested"]:
                return None, []
            for message in state["messages"]:
                if message["status"] == "queued" and not message.get('held_reason'):
                    notes = [state.pop(key) for key in ('context_note', 'robot_note') if state.get(key)]
                    if notes:
                        message['context_note'] = '\n'.join(notes)
                    message["status"] = "steering"
                    message["joined"] = state["active"]
                    return message, [_event("steer_started", message_id=message["id"],
                                            turn_id=state["active"])]
            return None, []
        return self.store.update(change)

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
                # /clear starts a new native conversation on purpose; any
                # other change of identity is an error.
                if (state["native_session"] and identity != state["native_session"]
                        and not event.data.get("cleared")):
                    raise ValueError("native conversation identity changed")
                state["native_session"] = identity
                if event.kind == "ready":
                    state["connected"] = True
            elif event.kind == "setting":
                state.setdefault("settings", {})[event.data["setting"]] = event.data["value"]
                if 'choices' in event.data:
                    state.setdefault('setting_choices', {})[event.data['setting']] = event.data['choices']
            elif event.kind == "commands":
                state["agent_commands"] = event.data["commands"]
            elif event.kind == "protocol_error":
                self._unknown(state, "native protocol error")
            elif event.kind in {"steer_delivered", "steer_rejected"}:
                message = _message(state, event.data["message_id"])
                if message["status"] == "steering":
                    if event.kind == "steer_delivered":
                        message["status"] = "steered"
                    else:
                        message["status"] = "queued"
                        message.pop("joined", None)
            elif event.kind == "turn_started" and event.turn_id and not state["active"]:
                found = [m for m in state["messages"] if m["id"] == event.turn_id]
                if found and found[0]["status"] == "steering":
                    # It reached the agent after the turn it joined had ended,
                    # and became a turn of its own.
                    found[0]["status"] = "running"
                    state["active"] = event.turn_id
                elif not found and event.data.get("origin") == "agent":
                    # The agent went on by itself (a background task finished).
                    message = {"id": event.turn_id, "client_id": "agent",
                               "request_id": event.turn_id, "original_text": "", "text": "",
                               "revision": 0, "status": "running", "cancel_requested": False,
                               "cancel_sent": False, "origin": "agent"}
                    state["messages"].append(message)
                    state["active"] = event.turn_id
                    return None, [record, _event("message_accepted", message=message)]
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
                    # An interrupt is the person's own stop: the queue goes
                    # on. A failure holds it until someone looks.
                    if status == "failed" and not message["cancel_requested"]:
                        if not state["paused"]:
                            _pause(state, "the turn failed")
                elif event.kind == "input_required":
                    state["requests"][event.data["request_id"]] = {
                        "turn_id": event.turn_id, "status": "pending", **event.data}
                elif event.kind == "input_cancelled":
                    state["requests"].pop(event.data["request_id"], None)
                elif event.kind == "unsupported_request":
                    _pause(state, "the agent asked for something this client cannot answer")
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
        _pause(state, f"agent disconnected: {reason}")
        for message in state["messages"]:
            if message["status"] == "steering":
                message["status"] = "unknown"
                message["uncertainty"] = (f"sent into a running turn, delivery unconfirmed "
                                          f"({reason})")
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

    def checked(self, step: str, status: str, summary: str = "", run: str = "") -> None:
        """A check on the host (connecting a robot) started or ended:
        ``status`` is running, passed or failed; ``run`` tells one run of
        a step from the next."""
        if status not in {"running", "passed", "failed"}:
            raise ValueError("a check is running, passed or failed")
        def change(state):
            state.setdefault("checks", {})[step] = {"status": status, "summary": summary,
                                                    "run": run}
            return None, [_event("check", step=step, status=status, summary=summary, run=run)]
        self.store.update(change)

    def motion_canceled(self, outcome: dict[str, bool]) -> None:
        """After an interrupt, the robot's motion actions were asked to
        cancel every goal: which did, which did not answer."""
        def change(state):
            return None, [_event("motion_canceled",
                                 canceled=[a for a, ok in outcome.items() if ok],
                                 failed=[a for a, ok in outcome.items() if not ok])]
        self.store.update(change)

    def handed_over(self, reason: str) -> None:
        """The agent's process is being moved to new resources on purpose
        (a robot just connected gets its own terminal): no turn is lost,
        so nothing is paused; the next owner reconnects the same
        conversation."""
        def change(state):
            if state["active"]:
                raise ValueError("a turn is running; hand over when it has finished")
            state["connected"] = False
            return None, [_event("handed_over", reason=reason)]
        self.store.update(change)

    def robot_stopped(self, reason: str) -> None:
        """Hold old instructions and interrupt the turn, keeping new dialogue usable."""
        self.robot_status('stopped', reason, hold=True, interrupt=True)

    def robot_status(self, status: str, reason: str = '', *, hold: bool = False,
                     interrupt: bool = False) -> None:
        """Resource availability is independent of the native conversation."""
        if status not in {'starting', 'ready', 'stopped', 'failed', 'updating', 'waiting'}:
            raise ValueError('unsupported robot connection status')
        def change(state):
            current = {'status': status, 'reason': reason}
            if state['closed'] or state.get('robot') == current:
                return None, []
            held = []
            if hold:
                for message in state['messages']:
                    if message['status'] == 'queued' and not message.get('held_reason'):
                        message['held_reason'] = reason or f'robot {status}'
                        message['revision'] += 1
                        held.append(message['id'])
            if interrupt and state['active']:
                _message(state, state['active'])['cancel_requested'] = True
            state['robot'] = current
            note = (f'Robot connection: {status}. {reason} '
                    'Use current observations before robot work. Instructions held across '
                    'the change remain held; do not replay prior actions or assume a restart '
                    'is a physical stop. To inspect or maintain this connection, run '
                    'python3 /openrua-runtime/robot.py --help.')
            state['robot_note'] = note
            return None, [_event('robot_status', **current, held=held)]
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
            if not state['connected']:
                raise ValueError('agent connection is unavailable; /continue only releases a queue pause, '
                                 'it cannot reconnect the agent. Inspect the session logs before restarting execution.')
            if state["active"] or any(
                    m["status"] == "unknown" for m in state["messages"]):
                raise ValueError("wait for the active turn and reconcile uncertain execution before resuming")
            if not state["paused"] and pause_id == state["pause_id"]:
                return None, []
            if not pause_id or pause_id != state["pause_id"]:
                raise ValueError("pause state changed; refresh before resuming")
            state["paused"] = False
            state["pause_reason"] = None
            return None, [_event("queue_resumed")]
        self.store.update(change)

    def reopen(self, note: str = 'Execution resources restarted. Check the current environment before continuing.') -> None:
        """Continue an ended conversation without replaying prior queued work."""
        def change(state):
            if not state['closed']:
                raise ValueError('conversation is not ended; connect to its owner instead')
            if state['active']:
                raise ValueError('conversation still has active execution; inspect it before reopening')
            withdrawn = []
            for message in state['messages']:
                if message['status'] == 'queued':
                    message['status'] = 'withdrawn'
                    withdrawn.append(message['id'])
            for request in state['requests'].values():
                if request['status'] == 'pending':
                    request['status'] = 'expired'
            state['closed'] = state['connected'] = False
            if not state['messages']:
                state['native_session'] = None
            state['robot'] = None
            state['context_note'] = note
            if any(m['status'] == 'unknown' for m in state['messages']):
                _pause(state, 'inspect the current robot and reconcile uncertain execution')
            else:
                state['paused'] = False
                state['pause_reason'] = None
                state['pause_id'] = None
            return None, [_event('session_reopened', withdrawn=withdrawn,
                                  note=note)]
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
