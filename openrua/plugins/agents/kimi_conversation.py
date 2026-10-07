"""Kimi Code 2.1.1 transcript protocol over the example's local HTTP bridge.

Completion comes from a prompt-correlated native transcript, never ACP end_turn.
The shared execution owner retains responsibility for user queues and recovery.
"""
from __future__ import annotations

from openrua.agents.conversation import ConversationProtocol, Event, Update


class KimiConversation(ConversationProtocol):
    def __init__(self, cwd, model, session_id=None):
        self.cwd, self.model, self.session_id = cwd, model, session_id
        self.ready = False
        self._begun = False
        self._active = None
        self._native_turn = None
        self._seen = set()
        self._frames = {}
        self._requests = {}
        self._pending = {}
        self._seq = 0
        self._cancelling = False

    @property
    def can_submit(self):
        return self.ready and self._active is None and not self._pending

    def _request(self, method, params):
        self._seq += 1
        key = str(self._seq)
        self._pending[key] = (method, self._active)
        return {'id': key, 'method': method, 'params': params}

    def begin(self):
        if self._begun:
            raise ValueError('connection initialization already started')
        self._begun = True
        return Update([self._request('open', {'cwd': self.cwd, 'session_id': self.session_id})])

    def submit(self, turn_id, text):
        if not self.can_submit:
            raise ValueError('conversation is not ready for a new turn')
        if not turn_id or turn_id in self._seen:
            raise ValueError('turn ID must be nonempty and new')
        self._seen.add(turn_id)
        self._active = turn_id
        self._native_turn = None
        self._frames.clear()
        self._cancelling = False
        return Update([self._request('submit', {'prompt_id': turn_id, 'text': text, 'model': self.model})])

    def interrupt(self, turn_id):
        if turn_id != self._active:
            raise ValueError('interrupt requires the active turn')
        if self._cancelling:
            return Update()
        self._cancelling = True
        return Update([self._request('cancel', {'prompt_id': turn_id})])

    def receive(self, frame):
        if 'id' in frame:
            operation = self._pending.pop(frame['id'], None)
            if operation is None:
                return Update(events=[Event('native', data=frame)])
            method, target = operation
            if 'error' in frame:
                self.ready = False
                return Update(events=[Event('protocol_error', target, {'operation': method, 'error': frame['error']})])
            result = frame['result']
            if method == 'open':
                sid = result['id']
                if not sid or (self.session_id and sid != self.session_id):
                    raise ValueError('native conversation identity differs from requested session')
                self.session_id, self.ready = sid, True
                return Update(events=[Event('ready', data={'session_id': sid, 'identity_confirmed': True})])
            if method == 'submit' and result.get('prompt_id') != target:
                raise ValueError('native prompt acknowledgement differs from submitted input')
            if method == 'cancel':
                return Update(events=[Event('interrupt_acknowledged', target)])
            return Update()
        if frame.get('type') != 'snapshot':
            return Update(events=[Event('native', data=frame)])
        if frame.get('session_id') != self.session_id or frame.get('prompt_id') != self._active or not self._active:
            return Update()
        turns = [t for t in frame['transcript']['items'] if t.get('kind') == 'turn'
                 and t.get('triggerPromptId') == self._active]
        if not turns:
            return Update()
        if len(turns) != 1:
            raise ValueError('multiple native turns for one submitted input')
        turn = turns[0]
        events = []
        if self._native_turn is None:
            self._native_turn = turn['turnId']
            events.append(Event('turn_started', self._active, {'native_turn_id': self._native_turn}))
        if turn['turnId'] != self._native_turn:
            raise ValueError('native turn identity changed during execution')
        texts = []
        for step in turn.get('steps', []):
            for item in step.get('frames', []):
                key = item['frameId']
                prior = self._frames.get(key)
                if item['kind'] == 'text' and item.get('role') == 'assistant':
                    value = item['text']
                    texts.append(value)
                    previous = prior['text'] if prior else ''
                    if not value.startswith(previous):
                        raise ValueError('native text changed without an append')
                    if len(value) > len(previous):
                        events.append(Event('text_delta', self._active, {'text': value[len(previous):], 'item_id': key}))
                elif item['kind'] == 'tool' and prior != item:
                    events.append(Event('item', self._active, {'kind': 'tool', 'item_id': key,
                        'phase': 'started' if item['state'] == 'running' else 'completed',
                        'text': item['name'], 'details': item}))
                self._frames[key] = item
        status = turn['state']
        if status not in ('queued', 'running', 'completed', 'failed', 'cancelled'):
            raise ValueError('unknown native completion status')
        if status in ('completed', 'failed', 'cancelled'):
            events.extend(Event('input_cancelled', self._active, {'request_id': key}) for key in self._requests)
            events.append(Event('turn_finished', self._active, {
                'status': 'interrupted' if status == 'cancelled' else status,
                'text': '\n'.join(texts), 'error': turn.get('error')}))
            self._requests.clear()
            self._active = self._native_turn = None
        else:
            events.extend(self._inputs(frame, turn))
        return Update(events=events)

    def _inputs(self, frame, turn):
        events, current = [], {}
        for kind in ('approvals', 'questions'):
            for item in frame.get(kind, []):
                if item.get('session_id') != self.session_id or (item.get('agent_id', 'main') != 'main' or item.get('turn_id') not in (turn['turnId'], turn['ordinal'])):
                    continue
                native = item['approval_id' if kind == 'approvals' else 'question_id']
                key = kind + ':' + native
                current[key] = (kind, native, item)
                if key in self._requests:
                    continue
                if kind == 'approvals':
                    questions = [{'id': 'decision', 'text': f"Allow {item['tool_name']}?",
                                  'choices': ['allow', 'deny'], 'secret': False}]
                else:
                    questions = [{'id': q['id'], 'text': q['question'],
                                  'choices': [o['label'] for o in q['options']], 'secret': False,
                                  'multiple': q.get('multi_select', False),
                                  'allow_other': q.get('allow_other', False)} for q in item['questions']]
                events.append(Event('input_required', self._active, {'request_id': key,
                              'questions': questions, 'details': item}))
        for key in self._requests.keys() - current.keys():
            events.append(Event('input_cancelled', self._active, {'request_id': key}))
        self._requests = current
        return events

    def respond(self, request_id, answers):
        if request_id not in self._requests:
            raise ValueError('input request is not pending')
        kind, native, item = self._requests[request_id]
        if kind == 'approvals':
            if set(answers) != {'decision'} or answers['decision'] not in (['allow'], ['deny']):
                raise ValueError('choose exactly one permission decision: allow or deny')
            body = {'decision': 'approved' if answers['decision'] == ['allow'] else 'rejected'}
        else:
            if set(answers) != {q['id'] for q in item['questions']}:
                raise ValueError('answer each native question exactly once')
            encoded = {}
            for q in item['questions']:
                values = answers[q['id']]
                options = {o['label']: o['id'] for o in q['options']}
                if len(options) != len(q['options']):
                    raise ValueError('native question has ambiguous option labels')
                if not values or len(set(values)) != len(values) or (not q.get('multi_select') and len(values) != 1):
                    raise ValueError('invalid number of answers')
                selected = [options[v] for v in values if v in options]
                other = [v for v in values if v not in options]
                if other and (not q.get('allow_other') or len(other) != 1):
                    raise ValueError('question does not allow this free-text answer')
                if other:
                    encoded[q['id']] = ({'kind': 'multi_with_other', 'option_ids': selected, 'other_text': other[0]}
                                        if selected else {'kind': 'other', 'text': other[0]})
                else:
                    encoded[q['id']] = ({'kind': 'multi', 'option_ids': selected} if q.get('multi_select')
                                        else {'kind': 'single', 'option_id': selected[0]})
            body = {'answers': encoded}
        return Update([self._request('respond', {'kind': kind, 'request_id': native, 'body': body})])
