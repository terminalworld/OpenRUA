"""Codex app-server JSON protocol. Checked against CLI 0.159.1 schemas;
turn/steer, server-started turns and async questions against 0.161.0.

Native thread and turn IDs stay here. The execution owner supplies local turn
IDs and serializes all calls; it owns persistence and the user-message queue.
A message typed while a turn runs joins it through turn/steer, as Enter does
in Codex's own terminal. A turn the server starts by itself (a goal or a
wakeup) is a turn of the agent's own. A question the model asks with
request_user_input_async arrives as an agent message with delivery "async";
it is shown as a question, and the answer is the next user message.
"""

from __future__ import annotations

from typing import Any
from importlib.metadata import version

from openrua.agents.conversation import ConversationProtocol, Event, Update


class CodexConversation(ConversationProtocol):
    def __init__(self, model: str, session_id: str | None = None, effort: str = "high", workspace: str = "/workspace"):
        self.model = model
        self.workspace = workspace
        self.effort = effort
        self.thread_id = session_id
        self.ready = False
        self._begun = False
        self._next_id = 0
        self._pending: dict[int, tuple[str, str | None]] = {}
        self._active: str | None = None
        self._native_turn: str | None = None
        self._seen: set[str] = set()
        self._early: list[dict] = []
        self._requests: dict[str, tuple[Any, str, list[dict]]] = {}
        self._steers: dict[int, str] = {}   # request ID -> message ID it carried

    @property
    def can_submit(self) -> bool:
        return self.ready and self._active is None

    def _request(self, method: str, params: dict, turn_id: str | None = None) -> dict:
        self._next_id += 1
        self._pending[self._next_id] = (method, turn_id)
        return {"id": self._next_id, "method": method, "params": params}

    def begin(self) -> Update:
        if self._begun:
            raise ValueError("connection initialization already started")
        self._begun = True
        return Update([self._request("initialize", {"clientInfo": {
            "name": "openrua", "title": "OpenRUA", "version": version("openrua")}})])

    def submit(self, turn_id: str, text: str) -> Update:
        if not self.can_submit:
            raise ValueError("conversation is not ready for a new turn")
        if not turn_id or turn_id in self._seen:
            raise ValueError("turn ID must be nonempty and new; replay requires explicit reconciliation")
        self._seen.add(turn_id)
        self._active = turn_id
        self._native_turn = None
        return Update([self._request("turn/start", {
            "threadId": self.thread_id, "input": [{"type": "text", "text": text}],
            "effort": self.effort}, turn_id)])

    @property
    def can_steer(self) -> bool:
        return self.ready and self._active is not None and self._native_turn is not None

    def steer(self, turn_id: str, message_id: str, text: str) -> Update:
        if turn_id != self._active or not self.can_steer:
            raise ValueError("no running turn to join")
        frame = self._request("turn/steer", {
            "threadId": self.thread_id, "input": [{"type": "text", "text": text}],
            "expectedTurnId": self._native_turn}, turn_id)
        self._steers[frame["id"]] = message_id
        return Update([frame])

    def interrupt(self, turn_id: str) -> Update:
        if turn_id != self._active or not self._native_turn:
            raise ValueError("interrupt requires the active turn's acknowledged native ID")
        return Update([self._request("turn/interrupt", {
            "threadId": self.thread_id, "turnId": self._native_turn}, turn_id)])

    def receive(self, frame: dict[str, Any]) -> Update:
        # Server requests also have an ID, but include a method.
        if "id" in frame and "method" not in frame:
            return self._response(frame)
        params = frame.get("params", {})
        method = frame.get("method", "")
        if params.get("threadId") != self.thread_id:
            return Update(events=[Event("native", data=frame)])
        if self._active and not self._native_turn:
            self._early.append(frame)
            return Update()
        native_turn = params.get("turnId") or params.get("turn", {}).get("id")
        if not self._active and method == "turn/started" and native_turn:
            # The server started a turn by itself (a goal, a wakeup).
            turn_id = f"agent-{native_turn}"
            self._seen.add(turn_id)
            self._active, self._native_turn = turn_id, native_turn
            return Update(events=[Event("turn_started", turn_id,
                                        {"origin": "agent", "native_turn_id": native_turn})])
        if not self._active or native_turn != self._native_turn:
            return Update(events=[Event("native", data=frame)])
        if "id" in frame:
            return self._input(frame)
        if method == "turn/completed":
            status = params["turn"].get("status")
            if status not in {"completed", "failed", "interrupted"}:
                raise ValueError(f"unrecognized native terminal status: {status!r}")
            event = Event("turn_finished", self._active,
                          {"status": status, "native_turn_id": self._native_turn,
                           "error": params["turn"].get("error")})
            self._active = self._native_turn = None
            self._requests.clear()
            return Update(events=[event])
        if method == "item/agentMessage/delta":
            return Update(events=[Event("text_delta", self._active,
                                        {"text": params["delta"], "item_id": params["itemId"]})])
        if method in {"item/started", "item/completed"}:
            return self._item(method, params["item"])
        return Update(events=[Event("native", self._active, frame)])

    def _item(self, method: str, item: dict) -> Update:
        """One item as a person reads it: the user's own message echoed back
        and empty reasoning are not shown; a command shows its command once."""
        kind = item["type"]
        summary = "".join(item.get("summary") or []) if kind == "reasoning" else ""
        if kind == "userMessage" or (kind == "reasoning" and not summary):
            return Update(events=[Event("native", self._active, {"method": method, "item": item})])
        phase = method.split("/")[1]
        if kind == "agentMessage":
            question = item.get("delivery") == "async" and item.get("questions")
            return Update(events=[Event("item", self._active, {
                "phase": phase, "item_id": item["id"],
                "kind": "question" if question else "message",
                "text": item.get("text", ""), "output": "", "details": item})])
        if kind == "commandExecution":
            details = {k: item[k] for k in ("cwd", "exitCode", "durationMs") if item.get(k) is not None}
            return Update(events=[Event("item", self._active, {
                "phase": phase, "item_id": item["id"], "kind": "command",
                "text": item.get("command") or "command",
                "output": item.get("aggregatedOutput", ""), "details": details})])
        details = {k: v for k, v in item.items() if k not in ("id", "type") and v not in (None, [], "")}
        return Update(events=[Event("item", self._active, {
            "phase": phase, "item_id": item["id"],
            "kind": {"fileChange": "files", "reasoning": "reasoning"}.get(kind, "activity"),
            "text": summary or item.get("text") or kind,
            "output": item.get("aggregatedOutput", ""), "details": details})])

    def _response(self, frame: dict) -> Update:
        request_id = frame["id"]
        pending = self._pending.pop(request_id, None)
        if pending is None:
            return Update(events=[Event("native", data=frame)])
        method, turn_id = pending
        if method == "turn/steer":
            # A turn that ended before the steer arrived refuses it; the
            # message then waits for a turn of its own.
            message_id = self._steers.pop(request_id)
            kind = "steer_rejected" if "error" in frame else "steer_delivered"
            return Update(events=[Event(kind, turn_id, {"message_id": message_id,
                                                        "error": frame.get("error")})])
        if "error" in frame:
            # Do not infer that an errored request had no side effects.
            self.ready = False
            return Update(events=[Event("protocol_error", turn_id,
                                        {"operation": method, "error": frame["error"]})])
        result = frame.get("result", {})
        if method == "initialize":
            params = {"model": self.model, "cwd": self.workspace,
                      "approvalPolicy": "never", "sandbox": "danger-full-access"}
            verb = "thread/start"
            if self.thread_id:
                verb = "thread/resume"
                params["threadId"] = self.thread_id
            return Update([{"method": "initialized", "params": {}}, self._request(verb, params)])
        if method in {"thread/start", "thread/resume"}:
            native_id = result["thread"]["id"]
            if self.thread_id and native_id != self.thread_id:
                raise ValueError("resumed conversation identity differs from the requested thread")
            self.thread_id = native_id
            self.ready = True
            return Update(events=[Event("ready", data={"session_id": native_id})])
        if method == "turn/start":
            if turn_id != self._active:
                raise ValueError("start acknowledgement does not match the outstanding turn")
            self._native_turn = result["turn"]["id"]
            update = Update(events=[Event("turn_started", turn_id,
                                          {"native_turn_id": self._native_turn})])
            early, self._early = self._early, []
            for message in early:
                update.extend(self.receive(message))
            return update
        # Interrupt acceptance is not terminal; wait for turn/completed.
        return Update(events=[Event("interrupt_acknowledged", turn_id)])

    def _input(self, frame: dict) -> Update:
        method, params = frame["method"], frame["params"]
        key = f"request-{frame['id']}"
        if method == "item/tool/requestUserInput":
            questions = [{"id": q["id"], "text": q["question"],
                          "choices": [o["label"] for o in q.get("options") or []],
                          "allow_other": bool(q.get("isOther")) or not q.get("options"),
                          "secret": q.get("isSecret", False)} for q in params["questions"]]
        elif method in {"item/commandExecution/requestApproval", "item/fileChange/requestApproval"}:
            questions = [{"id": "decision", "text": params.get("reason") or "Allow this operation?",
                          "choices": ["accept", "decline", "cancel"], "secret": False}]
        else:
            return Update(events=[Event("unsupported_request", self._active,
                                        {"request_id": key, "method": method, "details": params})])
        self._requests[key] = (frame["id"], method, questions)
        return Update(events=[Event("input_required", self._active,
                                    {"request_id": key, "questions": questions, "details": params})])

    def respond(self, request_id: str, answers: dict[str, list[str]]) -> Update:
        if request_id not in self._requests:
            raise ValueError("input request is not pending")
        native_id, method, questions = self._requests[request_id]
        if set(answers) != {q["id"] for q in questions} or not all(
                isinstance(v, list) and v and all(isinstance(x, str) for x in v)
                for v in answers.values()):
            raise ValueError("provide nonempty string answers for every requested question")
        if method == "item/tool/requestUserInput":
            result = {"answers": {key: {"answers": value} for key, value in answers.items()}}
        else:
            if len(answers["decision"]) != 1 or answers["decision"][0] not in questions[0]["choices"]:
                raise ValueError("choose one of the offered approval decisions")
            result = {"decision": answers["decision"][0]}
        del self._requests[request_id]
        return Update([{"id": native_id, "result": result}])
