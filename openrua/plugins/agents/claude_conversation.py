"""Claude Code's native bidirectional JSON control protocol.

The CLI supplies no target turn on interrupt. Serialize turns and wait for
both its result and any outstanding interrupt acknowledgement before another
submit. The public owner must also pause its queue after interruption.
"""

from __future__ import annotations

from typing import Any

from openrua.agents.conversation import ConversationProtocol, Event, Update


class ClaudeConversation(ConversationProtocol):
    def __init__(self, session_id: str):
        self.session_id = session_id
        self.ready = False
        self._confirmed = False
        self._begun = False
        self._next_id = 0
        self._pending: dict[str, tuple[str, str | None]] = {}
        self._active: str | None = None
        self._accepted = False
        self._user_uuid: str | None = None
        self._seen: set[str] = set()
        self._results: set[str] = set()
        self._cancel_pending = False
        self._interrupted = False
        self._message_id: str | None = None
        self._requests: dict[str, tuple[dict, list[dict]]] = {}
        self._tool_calls: dict[str, tuple[str, str]] = {}
        self._finished_tools: set[str] = set()

    @property
    def can_submit(self) -> bool:
        return self.ready and self._active is None and not self._cancel_pending

    def _control(self, subtype: str, **fields) -> dict:
        self._next_id += 1
        key = f"openrua-{self._next_id}"
        self._pending[key] = (subtype, self._active)
        return {"type": "control_request", "request_id": key,
                "request": {"subtype": subtype, **fields}}

    def begin(self) -> Update:
        if self._begun:
            raise ValueError("connection initialization already started")
        self._begun = True
        return Update([self._control("initialize", hooks=None)])

    def submit(self, turn_id: str, text: str) -> Update:
        from uuid import uuid4
        if not self.can_submit:
            raise ValueError("conversation is not ready for a new turn")
        if not turn_id or turn_id in self._seen:
            raise ValueError("turn ID must be nonempty and new; replay requires explicit reconciliation")
        self._seen.add(turn_id)
        self._active = turn_id
        self._user_uuid = str(uuid4())
        self._accepted = self._interrupted = False
        self._message_id = None
        return Update([{"type": "user", "uuid": self._user_uuid,
                        "session_id": self.session_id, "parent_tool_use_id": None,
                        "message": {"role": "user", "content": text}}])

    def interrupt(self, turn_id: str) -> Update:
        if turn_id != self._active or not self._accepted:
            raise ValueError("interrupt requires the active turn's echoed user message")
        if self._cancel_pending:
            return Update()
        self._cancel_pending = self._interrupted = True
        return Update([self._control("interrupt")])

    def receive(self, frame: dict[str, Any]) -> Update:
        update = Update()
        native_session = frame.get("session_id")
        if native_session:
            if native_session != self.session_id:
                self.ready = False
                raise ValueError("native conversation identity differs from the requested session")
            if not self._confirmed:
                self._confirmed = True
                update.events.append(Event("session", data={"session_id": native_session,
                                                            "identity_confirmed": True}))
        update.extend(self._receive(frame))
        return update

    def _receive(self, frame: dict) -> Update:
        kind = frame.get("type")
        if kind == "control_response":
            response = frame["response"]
            pending = self._pending.pop(response.get("request_id"), None)
            if pending is None:
                return Update(events=[Event("native", data=frame)])
            operation, turn_id = pending
            if response.get("subtype") != "success":
                self.ready = False
                return Update(events=[Event("protocol_error", turn_id,
                                            {"operation": operation, "error": response.get("error")})])
            if operation == "initialize":
                self.ready = True
                # The ID was passed with --session-id/--resume. Confirmation
                # arrives in a native session event after the first message.
                return Update(events=[Event("ready", data={"session_id": self.session_id,
                                                          "identity_confirmed": self._confirmed})])
            self._cancel_pending = False
            return Update(events=[Event("interrupt_acknowledged", turn_id)])
        if kind == "control_cancel_request":
            key = frame["request_id"]
            self._requests.pop(key, None)
            return Update(events=[Event("input_cancelled", self._active, {"request_id": key})])
        if kind == "user":
            events = self._tool_results(frame)
            if events:
                return Update(events=events)
        if not self._active:
            return Update(events=[Event("native", data=frame)])
        if kind == "user" and frame.get("uuid") == self._user_uuid:
            if self._accepted:
                return Update(events=[Event("native", self._active, frame)])
            self._accepted = True
            return Update(events=[Event("turn_started", self._active,
                                        {"native_message_id": self._user_uuid})])
        if kind == "result":
            result_id = frame.get("uuid")
            if result_id and result_id in self._results:
                return Update(events=[Event("native", data=frame)])
            if not isinstance(frame.get("is_error"), bool):
                self.ready = False
                raise ValueError("native result lacks its execution status")
            if not self._accepted:
                self.ready = False
                return Update(events=[Event("protocol_error", self._active,
                                            {"error": "result arrived without the expected user-message echo"})])
            if result_id:
                self._results.add(result_id)
            event = Event("turn_finished", self._active, {
                "status": "failed" if frame.get("is_error") else "completed",
                "interruption_requested": self._interrupted,
                "native_subtype": frame.get("subtype"), "text": frame.get("result", "")})
            self._active = None
            self._requests.clear()
            return Update(events=[event])
        if kind == "control_request":
            return self._input(frame)
        if kind == "stream_event":
            event = frame["event"]
            if event.get("type") == "message_start":
                self._message_id = event["message"]["id"]
            if event.get("type") == "content_block_delta" and event.get("delta", {}).get("type") == "text_delta":
                return Update(events=[Event("text_delta", self._active,
                                            {"text": event["delta"]["text"], "item_id": self._message_id})])
        if kind == "assistant":
            message = frame["message"]
            events = []
            for block in message.get("content", []):
                if block.get("type") == "tool_use" and block["id"] not in self._tool_calls:
                    self._tool_calls[block["id"]] = (self._active, block["name"])
                    events.append(Event("item", self._active, {"phase": "started", "kind": "tool",
                        "item_id": block["id"], "text": block["name"], "details": block["input"]}))
            text = "".join(b["text"] for b in message.get("content", []) if b.get("type") == "text")
            if text:
                events.append(Event("item", self._active, {"phase": "completed", "kind": "message",
                    "item_id": message["id"], "text": text}))
            return Update(events=events)
        return Update(events=[Event("native", self._active, frame)])

    def _tool_results(self, frame: dict) -> list[Event]:
        content = frame.get("message", {}).get("content")
        if not isinstance(content, list):
            return []
        events = []
        for block in content:
            if not isinstance(block, dict) or block.get("type") != "tool_result":
                continue
            key = block.get("tool_use_id")
            if key not in self._tool_calls or key in self._finished_tools:
                continue
            turn_id, name = self._tool_calls[key]
            self._finished_tools.add(key)
            output = block.get("content", "")
            if isinstance(output, list):
                # Binary content remains in the separately stored native frame.
                output = "\n".join(
                    part.get("text", "") if part.get("type") == "text"
                    else f"[{part.get('type', 'structured')} result in native record]"
                    for part in output if isinstance(part, dict))
            elif not isinstance(output, str):
                output = "" if output is None else "[structured result in native record]"
            events.append(Event("item", turn_id, {
                "phase": "failed" if block.get("is_error") else "completed", "kind": "tool",
                "item_id": key, "text": name, "output": output}))
        return events

    def _input(self, frame: dict) -> Update:
        request, key = frame["request"], frame["request_id"]
        if request.get("subtype") != "can_use_tool":
            return Update(events=[Event("unsupported_request", self._active,
                                        {"request_id": key, "details": request})])
        if request["tool_name"] == "AskUserQuestion":
            questions = [{"id": str(i), "text": q["question"],
                          "choices": [o["label"] for o in q.get("options", [])],
                          "multiple": q.get("multiSelect", False), "secret": False}
                         for i, q in enumerate(request["input"]["questions"])]
        else:
            questions = [{"id": "decision", "text": request.get("description") or
                          f"Allow {request['tool_name']}?", "choices": ["allow", "deny"], "secret": False}]
        self._requests[key] = (request, questions)
        return Update(events=[Event("input_required", self._active,
                                    {"request_id": key, "questions": questions, "details": request})])

    def respond(self, request_id: str, answers: dict[str, list[str]]) -> Update:
        if request_id not in self._requests:
            raise ValueError("input request is not pending")
        request, questions = self._requests[request_id]
        if set(answers) != {q["id"] for q in questions} or not all(
                isinstance(v, list) and v and all(isinstance(x, str) for x in v)
                for v in answers.values()):
            raise ValueError("provide nonempty string answers for every requested question")
        if request["tool_name"] == "AskUserQuestion":
            value = {**request["input"], "answers": {
                q["text"]: ", ".join(answers[q["id"]]) for q in questions}}
            response = {"behavior": "allow", "updatedInput": value}
        else:
            decision = answers["decision"]
            if len(decision) != 1 or decision[0] not in {"allow", "deny"}:
                raise ValueError("choose one offered approval decision")
            response = ({"behavior": "allow", "updatedInput": request["input"]}
                        if decision[0] == "allow" else {"behavior": "deny", "message": "User declined"})
        del self._requests[request_id]
        return Update([{"type": "control_response", "response": {
            "subtype": "success", "request_id": request_id, "response": response}}])
