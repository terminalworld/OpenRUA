"""Codex app-server JSON protocol. Checked against CLI 0.159.1 schemas.

Native thread and turn IDs stay here. The execution owner supplies local turn
IDs and serializes all calls; it owns persistence and the user-message queue.
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
            item = params["item"]
            kind = {"agentMessage": "message", "commandExecution": "command",
                    "fileChange": "files", "reasoning": "reasoning"}.get(item["type"], "activity")
            return Update(events=[Event("item", self._active, {
                "phase": method.split("/")[1], "item_id": item["id"], "kind": kind,
                "text": item.get("text") or item.get("command") or item["type"],
                "output": item.get("aggregatedOutput", ""), "details": item})])
        return Update(events=[Event("native", self._active, frame)])

    def _response(self, frame: dict) -> Update:
        request_id = frame["id"]
        pending = self._pending.pop(request_id, None)
        if pending is None:
            return Update(events=[Event("native", data=frame)])
        method, turn_id = pending
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
