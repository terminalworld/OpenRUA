"""ZCode CLI 0.16.9's native session protocol, for external plugin development.

The upstream compatibility methods are version-specific. This module does not
install an agent, select credentials, or register a selectable robot plugin.
"""
from __future__ import annotations

from openrua.agents.conversation import ConversationProtocol, Event, Update


class ZCodeConversation(ConversationProtocol):
    def __init__(self, cwd: str, session_id: str | None = None):
        self.cwd = cwd
        self.session_id = session_id
        self.ready = False
        self._begun = False
        self._seq = 0
        self._pending = {}
        self._active = None
        self._native_turn = None
        self._seen = set()
        self._events = set()
        self._requests = {}
        self._cancel_pending = False
        self._send_pending = False

    @property
    def can_submit(self):
        return self.ready and self._active is None and not self._cancel_pending and not self._send_pending

    def _request(self, method, params):
        self._seq += 1
        key = f'openrua-{self._seq}'
        self._pending[key] = (method, self._active)
        return {'id': key, 'method': method, 'params': params}

    def begin(self):
        if self._begun:
            raise ValueError('connection initialization already started')
        self._begun = True
        workspace = {'workspacePath': self.cwd, 'workspaceKey': self.cwd}
        if self.session_id:
            method, params = 'session/resume', {'sessionId': self.session_id, 'workspace': workspace}
        else:
            method, params = 'session/create', {'workspace': workspace, 'titleGenerationEnabled': False}
        return Update([self._request(method, params)])

    def submit(self, turn_id, text):
        if not self.can_submit:
            raise ValueError('conversation is not ready for a new turn')
        if not turn_id or turn_id in self._seen:
            raise ValueError('turn ID must be nonempty and new')
        self._seen.add(turn_id)
        self._active, self._native_turn = turn_id, None
        self._send_pending = True
        return Update([self._request('session/send', {
            'sessionId': self.session_id, 'inputId': turn_id, 'content': text})])

    def interrupt(self, turn_id):
        if turn_id != self._active or self._native_turn is None:
            raise ValueError('interrupt requires the active native turn')
        if self._cancel_pending:
            return Update()
        self._cancel_pending = True
        return Update([self._request('session/stop', {'sessionId': self.session_id})])

    def receive(self, frame):
        if 'method' in frame:
            if 'id' in frame:
                return self._input(frame)
            if frame['method'] == 'session/event':
                return self._event(frame['params'])
            return Update(events=[Event('native', data=frame)])
        pending = self._pending.pop(frame.get('id'), None)
        if pending is None:
            return Update(events=[Event('native', data=frame)])
        operation, target = pending
        if 'error' in frame:
            self.ready = False
            return Update(events=[Event('protocol_error', target, {
                'operation': operation, 'error': frame['error']})])
        result = frame['result']
        if operation in ('session/create', 'session/resume'):
            sid = result['session']['sessionId']
            if not sid or (self.session_id and sid != self.session_id):
                raise ValueError('native conversation identity differs from requested session')
            self.session_id = sid
            return Update([self._request('session/subscribe', {
                'sessionId': sid, 'deliveryKind': 'desktop-continuous', 'includeSnapshot': False})])
        if operation == 'session/subscribe':
            if result['sessionId'] != self.session_id:
                raise ValueError('subscription belongs to a different session')
            self.ready = True
            return Update(events=[Event('ready', data={'session_id': self.session_id, 'identity_confirmed': True})])
        if operation == 'session/send':
            if result.get('accepted') is not True or result.get('sessionId') != self.session_id:
                raise ValueError('native submission acknowledgement is invalid')
            self._send_pending = False
        if operation == 'session/stop':
            self._cancel_pending = False
            return Update(events=[Event('interrupt_acknowledged', target)])
        return Update()

    def _event(self, event):
        if event.get('sessionId') != self.session_id:
            return Update(events=[Event('native', data=event)])
        key = event.get('eventId')
        if not key or key in self._events:
            return Update()
        self._events.add(key)
        if not self._active:
            return Update(events=[Event('native', data=event)])
        kind, payload = event['type'], event.get('payload', {})
        if kind == 'turn.started':
            if payload.get('inputId') != self._active:
                return Update(events=[Event('native', data=event)])
            if self._native_turn:
                raise ValueError('native started another turn for the active input')
            self._native_turn = event['turnId']
            return Update(events=[Event('turn_started', self._active, {'native_turn_id': self._native_turn})])
        if not self._native_turn or event.get('turnId') != self._native_turn:
            return Update(events=[Event('native', data=event)])
        if kind in ('turn.completed', 'turn.failed'):
            if payload.get('inputId') != self._active:
                raise ValueError('native terminal event does not match the submitted input')
            if kind == 'turn.failed':
                status = 'failed'
            else:
                reason = payload['resultType']
                if reason not in ('success', 'cancelled', 'error_max_turns', 'error_max_budget',
                                  'error_during_execution', 'error_max_tool_calls'):
                    raise ValueError('unknown native completion status')
                status = {'success': 'completed', 'cancelled': 'interrupted'}.get(reason, 'failed')
            events = [Event('input_cancelled', self._active, {'request_id': k}) for k in self._requests]
            events.append(Event('turn_finished', self._active, {
                'status': status, 'text': payload.get('response', ''),
                'error': payload.get('error'), 'native_result': payload.get('resultType')}))
            self._active = self._native_turn = None
            self._requests.clear()
            return Update(events=events)
        if kind == 'permission.resolved':
            expired = [k for k, value in self._requests.items() if value['logical'] == payload.get('requestId')]
            for k in expired:
                del self._requests[k]
            return Update(events=[Event('input_cancelled', self._active, {'request_id': k}) for k in expired])
        if kind == 'model.streaming' and payload.get('kind') == 'text_delta':
            return Update(events=[Event('text_delta', self._active, {
                'text': payload['delta'], 'item_id': payload.get('assistantMessageId')})])
        if kind == 'tool.updated' and payload.get('toolCallId'):
            phase = 'completed' if payload['kind'] in ('result', 'error') else 'started'
            return Update(events=[Event('item', self._active, {'kind': 'tool', 'phase': phase,
                'item_id': payload['toolCallId'], 'text': payload.get('toolName', ''), 'details': payload})])
        return Update(events=[Event('native', self._active, event)])

    def _input(self, frame):
        params = frame.get('params', {})
        key = str(frame['id'])
        if frame['method'] == 'session/requestRuntimePreferences':
            # Native compatibility fallback retains its own runtime defaults.
            return Update([{'id': frame['id'], 'error': {'code': -32601, 'message': 'Use native runtime preferences'}}])
        if (not self._active or params.get('sessionId') != self.session_id or
                params.get('turnId') not in (None, self._native_turn)):
            return Update([{'id': frame['id'], 'error': {'code': -32602, 'message': 'No matching active turn'}}])
        if frame['method'] != 'interaction/requestPermission':
            # No credential callbacks, browser tools or user answers are fabricated.
            return Update([{'id': frame['id'], 'error': {'code': -32601, 'message': 'Unsupported client request'}}],
                          [Event('unsupported_request', self._active, {'request_id': key, 'details': params})])
        if key in self._requests:
            raise ValueError('duplicate pending native permission request')
        choices = {}
        for option in params.get('options', []):
            if option.get('kind') == 'allow_once' and option.get('response', {}).get('decision') == 'allow':
                choices['allow'] = option['response']
            elif option.get('kind') == 'deny' and option.get('response', {}).get('decision') == 'deny':
                choices['deny'] = option['response']
        if set(choices) != {'allow', 'deny'}:
            raise ValueError('native permission request lacks supported one-time decisions')
        logical = params['requestId']
        expired = [k for k, value in self._requests.items() if value['logical'] == logical]
        for k in expired:
            del self._requests[k]
        self._requests[key] = {'wire': frame['id'], 'logical': logical, 'choices': choices}
        events = [Event('input_cancelled', self._active, {'request_id': k}) for k in expired]
        return Update(events=events+[Event('input_required', self._active, {'request_id': key,
            'questions': [{'id': 'decision', 'text': params.get('reason') or f"Allow {params['toolName']}?",
                           'choices': ['allow', 'deny'], 'secret': False}], 'details': params})])

    def respond(self, request_id, answers):
        if request_id not in self._requests:
            raise ValueError('input request is not pending')
        if set(answers) != {'decision'} or answers['decision'] not in (['allow'], ['deny']):
            raise ValueError('choose exactly one permission decision: allow or deny')
        pending = self._requests.pop(request_id)
        return Update([{'id': pending['wire'], 'result': pending['choices'][answers['decision'][0]]}])
