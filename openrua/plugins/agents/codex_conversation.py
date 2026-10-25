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
import re

from openrua.agents.conversation import ConversationProtocol, Event, Update


class CodexConversation(ConversationProtocol):
    DEFAULT_PERMISSIONS = 'auto_review'
    # What the person sets with /permissions, as Codex's approval policy.
    APPROVAL = {"untrusted": "untrusted", "never": "never", "auto_review": "on-request"}
    SETTINGS = {'permissions': tuple(APPROVAL), 'model': None, 'effort': None}
    PERMISSIONS = [
        {"value": "untrusted", "label": "Ask for approval", "description": "Use Codex's untrusted command policy in the agent container. Ask when its rules require approval"},
        {"value": "auto_review", "label": "Approve for me", "description": "Keep Codex's workspace restrictions and let it review requests for extra access automatically"},
        {"value": "never", "label": "Full access", "description": "Run commands inside the agent container without asking for approval"},
    ]

    def __init__(self, model: str, session_id: str | None = None, effort: str = "high",
                 workspace: str = "/workspace", permissions: str = DEFAULT_PERMISSIONS):
        if permissions not in self.APPROVAL:
            raise ValueError('permissions is untrusted, auto_review or never')
        self.model = model
        self.permissions = permissions
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
        self._server_turn = False           # /compact: the server starts the turn
        self._skill_text: str | None = None
        self._reviewer_supported = False
        self._models = {}
        self._model_pages = 0
        self.catalog_only = False

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
            "name": "openrua", "title": "OpenRUA", "version": version("openrua")},
            "capabilities": {"experimentalApi": True}})])

    def submit(self, turn_id: str, text: str) -> Update:
        if not self.can_submit:
            raise ValueError("conversation is not ready for a new turn")
        if not turn_id or turn_id in self._seen:
            raise ValueError("turn ID must be nonempty and new; replay requires explicit reconciliation")
        self._seen.add(turn_id)
        self._active = turn_id
        self._native_turn = None
        command = text.strip()
        if command == "/compact":
            # As in Codex's own terminal; the server runs it as a turn.
            self._server_turn = True
            return Update([self._request("thread/compact/start", {"threadId": self.thread_id},
                                         turn_id)])
        if command == "/clear":
            return Update([self._request("thread/start", self._thread_params(), turn_id)])
        if command == '/skills' or re.search(r'(?<![\w$])\$[a-zA-Z0-9][a-zA-Z0-9_-]*', text):
            self._skill_text = text
            return Update([self._request('skills/list', {'cwds': [self.workspace], 'forceReload': True}, turn_id)])
        return self._start_turn(turn_id, text)

    def _start_turn(self, turn_id: str, text: str, skills: tuple[dict, ...] | list[dict] = ()) -> Update:
        return Update([self._request("turn/start", {
            "threadId": self.thread_id, "input": [{"type": "text", "text": text}, *skills],
            "effort": self.effort, "model": self.model,
            **self._approval_params(), 'sandboxPolicy': self._sandbox_policy()}, turn_id)])

    def _sandbox_policy(self) -> dict:
        if self.permissions == 'auto_review':
            return {'type': 'workspaceWrite', 'writableRoots': [self.workspace], 'networkAccess': True}
        return {'type': 'dangerFullAccess'}

    def _approval_params(self) -> dict:
        return {"approvalPolicy": self.APPROVAL[self.permissions],
                'approvalsReviewer': 'auto_review' if self.permissions == 'auto_review' else 'user'}

    def _thread_params(self) -> dict:
        return {"model": self.model, "cwd": self.workspace,
                **self._approval_params(), "sandbox": 'workspace-write' if self.permissions == 'auto_review' else 'danger-full-access'}

    def configure(self, setting: str, value: str) -> Update:
        """Kept here and sent with every turn, as Codex's own terminal does:
        in effect from the next turn."""
        if setting == "permissions":
            if value not in self.APPROVAL:
                raise ValueError("permissions is untrusted, auto_review or never")
            if value == 'auto_review' and not self._reviewer_supported:
                raise ValueError('This native app-server did not advertise approval reviewer support; update the agent CLI.')
            self.permissions = value
        elif setting == "model":
            if not value:
                raise ValueError("name a model")
            preset = self._models.get(value, {})
            supported = [row['value'] for row in preset.get('efforts', [])]
            effort = self.effort
            if value != self.model or (supported and effort not in supported):
                default = preset.get('default_effort')
                effort = default if default and (not supported or default in supported) else (supported[0] if supported else effort)
            return self.select_model(value, effort)
        elif setting == 'effort':
            supported = {row['value'] for row in self._models.get(self.model, {}).get('efforts', [])}
            if not isinstance(value, str) or not value.strip() or (supported and value not in supported):
                raise ValueError('choose a native effort level' + (': ' + ', '.join(sorted(supported)) if supported else ''))
            self.effort = value
        else:
            raise ValueError(f"unknown setting: {setting}")
        return Update(events=[Event("setting", data={"setting": setting, "value": value,
                                                     "effective_from": "next_turn"})])

    def _effort_setting(self):
        return Event('setting', data={'setting': 'effort', 'value': self.effort, 'effective_from': 'next_turn',
                                      'choices': self._models.get(self.model, {}).get('efforts', [])})

    def select_model(self, model: str, effort: str) -> Update:
        if not isinstance(model, str) or not model.strip():
            raise ValueError('name a model')
        supported = {row['value'] for row in self._models.get(model, {}).get('efforts', [])}
        if not isinstance(effort, str) or not effort.strip() or (supported and effort not in supported):
            raise ValueError('choose a native effort level' + (': ' + ', '.join(sorted(supported)) if supported else ''))
        self.model, self.effort = model, effort
        return Update(events=[Event('setting', data={'setting': 'model', 'value': model,
                                                    'effective_from': 'next_turn'}), self._effort_setting()])

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
        if (self._server_turn and method == "turn/started" and self._active
                and not self._native_turn):
            # /compact's turn, started by the server: it is the running turn.
            self._server_turn = False
            self._native_turn = params["turn"]["id"]
            update = Update(events=[Event("turn_started", self._active,
                                          {"native_turn_id": self._native_turn})])
            early, self._early = self._early, []
            for message in early:
                update.extend(self.receive(message))
            return update
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
        if method in {'item/autoApprovalReview/started', 'item/autoApprovalReview/completed'}:
            review = params['review']
            finished = method.endswith('/completed')
            return Update(events=[Event('item', self._active, {
                'phase': ('completed' if review['status'] == 'approved' else 'failed') if finished else 'started',
                'item_id': 'approval-review-' + params['reviewId'], 'kind': 'activity',
                'text': 'Native approval review', 'details': params.get('action', {}),
                'output': '\n'.join(str(value) for value in (review['status'], review.get('riskLevel'), review.get('rationale')) if value)})])
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
        if method == 'model/list':
            if 'error' in frame:
                return Update(events=[Event('setting', data={'setting': 'model', 'value': self.model,
                    'choices': list(self._models.values()), 'error': frame['error'].get('message', str(frame['error']))})])
            result = frame.get('result', {})
            for row in result.get('data', []):
                value = row.get('model') or row.get('id')
                if isinstance(value, str) and value:
                    self._models[value] = dict(value=value, label=row.get('displayName') or value,
                                              description=row.get('description') or '',
                                              default_effort=row.get('defaultReasoningEffort'),
                                              efforts=[dict(value=level['reasoningEffort'], label=level['reasoningEffort'],
                                                            description=level.get('description', ''))
                                                       for level in row.get('supportedReasoningEfforts') or []
                                                       if isinstance(level.get('reasoningEffort'), str) and level['reasoningEffort']])
            self._model_pages += 1
            more = result.get('nextCursor')
            outbound = [self._request('model/list', {'limit': 100, 'cursor': more})] if more and self._model_pages < 5 else []
            return Update(outbound, [Event('setting', data={'setting': 'model', 'value': self.model,
                                                           'effective_from': 'next_turn',
                                                           'choices': list(self._models.values())}), self._effort_setting()])
        if method == 'skills/list':
            text, self._skill_text = self._skill_text or '', None
            data = frame.get('result', {}).get('data', [])
            skills = [skill for item in data for skill in item.get('skills', [])]
            errors = [error for item in data for error in item.get('errors', [])]
            if 'error' in frame or text.strip() == '/skills':
                lines = [f"${skill['name']} [{skill.get('scope', 'native')}]"
                         f"{' (disabled)' if not skill.get('enabled', True) else ''}: "
                         f"{skill.get('description', '')}\n  {skill['path']}" for skill in skills]
                lines.extend(str(error) for error in errors)
                if 'error' in frame:
                    lines.append(f"Native skill discovery failed: {frame['error']}. Use the agent's native terminal to inspect its supported skills.")
                self._active = None
                return Update(events=[
                    Event('item', turn_id, {'phase': 'completed', 'item_id': f'skills-{turn_id}', 'kind': 'message',
                                           'text': '\n'.join(lines) or 'No native skills are available.', 'output': ''}),
                    Event('turn_finished', turn_id, {'status': 'failed' if 'error' in frame else 'completed'})])
            requested = set(re.findall(r'(?<![\w$])\$([a-zA-Z0-9][a-zA-Z0-9_-]*)', text))
            selected = []
            for name in sorted(requested):
                matches = {skill['path']: skill for skill in skills
                           if skill['name'] == name and skill.get('enabled', True)}
                # Native selectors can expose multiple same-name skills. A
                # bare name must not silently select an arbitrary definition.
                # Without a structured item the native agent resolves the text.
                if len(matches) == 1:
                    skill = next(iter(matches.values()))
                    selected.append({'type': 'skill', 'name': name, 'path': skill['path']})
            return self._start_turn(turn_id, text, selected)
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
            if self.catalog_only:
                return Update([{'method': 'initialized', 'params': {}}, self._request('model/list', {'limit': 100})])
            params = self._thread_params()
            verb = "thread/start"
            if self.thread_id:
                verb = "thread/resume"
                params["threadId"] = self.thread_id
            return Update([{"method": "initialized", "params": {}}, self._request(verb, params)])
        if method == "thread/start" and turn_id is not None:
            # /clear: a new thread is the conversation from now on.
            self.thread_id = result["thread"]["id"]
            self._active = None
            return Update(events=[
                Event("session", data={"session_id": self.thread_id, "cleared": True}),
                Event("turn_finished", turn_id, {"status": "completed"})])
        if method in {"thread/start", "thread/resume"}:
            self._reviewer_supported = 'approvalsReviewer' in result
            if self.permissions == 'auto_review' and (
                    result.get('approvalsReviewer') != 'auto_review'
                    or result.get('approvalPolicy') != 'on-request'
                    or result.get('sandbox', {}).get('type') != 'workspaceWrite'):
                raise ValueError('Native automatic approval review was not confirmed; no turn was started.')
            native_id = result["thread"]["id"]
            if self.thread_id and native_id != self.thread_id:
                raise ValueError("resumed conversation identity differs from the requested thread")
            self.thread_id = native_id
            self.model = result.get('model') or self.model
            self.ready = True
            return Update([self._request('model/list', {'limit': 100})], events=[
                Event("ready", data={"session_id": native_id, 'settings': list(self.SETTINGS)}),
                Event("setting", data={"setting": "permissions", "value": self.permissions,
                                       "choices": [choice for choice in self.PERMISSIONS
                                                   if choice['value'] != 'auto_review' or self._reviewer_supported]}),
                Event("setting", data={"setting": "model", "value": self.model}),
                self._effort_setting(),
                Event("commands", data={"commands": [
                    {"name": "compact", "description": "summarize the conversation to free context"},
                    {"name": "clear", "description": "start a new conversation"},
                    {"name": "skills", "description": "list native skills; use $name in your message to select one"}]})])
        if method == "thread/compact/start":
            # The turn it runs arrives as the server's own turn/started.
            return Update()
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
            # Say what would run, as Codex's own terminal does.
            what = (f"Run `{params['command']}`?" if params.get("command")
                    else "Apply this file change?" if method.startswith("item/fileChange")
                    else "Allow this operation?")
            text = f"{what} ({params['reason']})" if params.get("reason") else what
            questions = [{"id": "decision", "text": text,
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
