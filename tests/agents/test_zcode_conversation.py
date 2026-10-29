"""Native session correlation, failure and cancellation boundaries."""
import pytest

from openrua.plugins.agents.zcode_conversation import ZCodeConversation


def reply(protocol, request, result):
    return protocol.receive({'id': request['id'], 'result': result})


def ready(session_id=None):
    p = ZCodeConversation('/workspace', session_id)
    request = p.begin().outbound[0]
    assert request['method'] == ('session/resume' if session_id else 'session/create')
    subscribe = reply(p, request, {'session': {'sessionId': session_id or 'native'}}).outbound[0]
    reply(p, subscribe, {'sessionId': p.session_id, 'events': [], 'eventSeq': 0})
    assert p.can_submit
    return p


def event(p, kind, payload=None, *, turn='turn-native', key=None, session='native'):
    return p.receive({'method': 'session/event', 'params': {
        'eventId': key or kind, 'sessionId': session, 'turnId': turn, 'type': kind,
        'payload': payload or {}}})


def start(p):
    request = p.submit('input', 'inspect').outbound[0]
    reply(p, request, {'sessionId': 'native', 'accepted': True})
    update = event(p, 'turn.started', {'inputId': 'input'})
    assert update.events[0].kind == 'turn_started'
    return request


@pytest.mark.parametrize('reason,expected', [('success', 'completed'), ('cancelled', 'interrupted'),
    ('error_max_turns', 'failed'), ('error_max_budget', 'failed'),
    ('error_during_execution', 'failed'), ('error_max_tool_calls', 'failed')])
def test_terminal_status_is_not_inferred_from_event_name(reason, expected):
    p=ready(); start(p)
    result = event(p, 'turn.completed', {'inputId': 'input', 'resultType': reason})
    assert result.events[-1].data['status'] == expected
    assert p.can_submit


def test_error_details_are_preserved():
    p=ready(); start(p)
    result = event(p, 'turn.failed', {'inputId': 'input', 'error': {'message': 'quota exhausted'}})
    assert result.events[-1].data['status'] == 'failed'
    assert result.events[-1].data['error']['message'] == 'quota exhausted'


@pytest.mark.parametrize('ack_first', [True, False])
def test_cancel_ack_alone_never_releases_the_turn(ack_first):
    p=ready(); start(p)
    request = p.interrupt('input').outbound[0]
    with pytest.raises(ValueError): p.interrupt('foreign')
    assert not p.interrupt('input').outbound
    if ack_first:
        reply(p, request, {})
        assert not p.can_submit
    event(p, 'turn.completed', {'inputId': 'input', 'resultType': 'cancelled'})
    if not ack_first:
        assert not p.can_submit
        reply(p, request, {})
    assert p.can_submit


def test_terminal_before_send_ack_keeps_next_input_blocked():
    p=ready()
    request=p.submit('input','inspect').outbound[0]
    event(p, 'turn.started', {'inputId': 'input'})
    event(p, 'turn.completed', {'inputId': 'input', 'resultType': 'success'})
    assert not p.can_submit
    reply(p, request, {'sessionId': 'native', 'accepted': True})
    assert p.can_submit


def test_foreign_and_duplicate_events_do_not_end_current_turn():
    p=ready(); start(p)
    event(p,'turn.completed',{'inputId':'input','resultType':'success'},session='foreign')
    event(p,'turn.completed',{'inputId':'input','resultType':'success'},turn='old',key='old')
    assert not p.can_submit
    event(p,'turn.completed',{'inputId':'input','resultType':'success'},key='finish')
    p.submit('second', 'next')
    assert not event(p,'turn.completed',{'inputId':'input','resultType':'success'},key='finish').events
    assert not p.can_submit


def test_resume_must_preserve_exact_identity():
    p=ZCodeConversation('/workspace','saved')
    request=p.begin().outbound[0]
    assert request['params']['sessionId']=='saved'
    with pytest.raises(ValueError, match='identity'):
        reply(p,request,{'session':{'sessionId':'different'}})


def permission(p, key=7):
    return p.receive({'id':key,'method':'interaction/requestPermission','params':{
        'sessionId':'native','turnId':'turn-native','toolName':'Write','reason':'Write probe.txt', 'requestId':'permission',
        'options':[{'kind':'allow_once','response':{'decision':'allow'}}, {'kind':'deny','response':{'decision':'deny'}}]}})


def test_permission_reply_is_explicit_and_expires_at_completion():
    p=ready(); start(p)
    update=permission(p)
    assert update.events[0].kind=='input_required'
    with pytest.raises(ValueError): p.respond('7', {'decision':['allow','deny']})
    assert p.respond('7', {'decision':['deny']}).outbound == [{'id':7,'result':{'decision':'deny'}}]
    with pytest.raises(ValueError): p.respond('7', {'decision':['allow']})
    permission(p,8)
    event(p,'turn.completed',{'inputId':'input','resultType':'success'})
    with pytest.raises(ValueError): p.respond('8', {'decision':['allow']})


def test_unknown_client_request_is_not_implicitly_approved():
    p=ready(); start(p)
    result=p.receive({'id':9,'method':'interaction/requestProviderRuntimeHeaders','params':{'sessionId':'native'}})
    assert result.outbound[0]['error']['code']==-32601
    assert result.events[0].kind=='unsupported_request'


def test_invalid_completion_and_request_failure_never_resume():
    p=ready(); start(p)
    with pytest.raises(ValueError):
        event(p,'turn.completed',{'inputId':'input','resultType':'unknown-new-status'})
    stop=p.interrupt('input').outbound[0]
    result=p.receive({'id':stop['id'],'error':{'message':'stop unavailable'}})
    assert result.events[0].kind=='protocol_error' and not p.can_submit


def test_retried_permission_expires_old_rpc_request():
    p=ready(); start(p)
    permission(p,7)
    update=permission(p,8)
    assert update.events[0].kind=='input_cancelled'
    with pytest.raises(ValueError): p.respond('7', {'decision':['allow']})
    assert p.respond('8', {'decision':['allow']}).outbound[0]['id']==8


def test_optional_startup_preferences_use_native_fallback():
    p=ZCodeConversation('/workspace')
    p.begin()
    update=p.receive({'id':'server-1','method':'session/requestRuntimePreferences','params':{}})
    assert update.outbound[0]['error']['code']==-32601
    assert not update.events


def question(p, key=9, *, logical='question'):
    return p.receive({'id': key, 'method': 'interaction/requestUserInput', 'params': {
        'sessionId': 'native', 'turnId': 'turn-native', 'requestId': logical,
        'toolName': 'AskUserQuestion', 'prompt': 'Choose the object and where it should go.',
        'questions': [
            {'question': 'Which object?', 'header': 'Object', 'options': [
                {'label': 'Cup', 'value': 'cup-id', 'description': 'The cup beside the plate.'},
                {'label': 'Block', 'value': 'block-id'}]},
            {'question': 'Which locations?', 'header': 'Locations', 'multiSelect': True,
             'options': [{'label': 'Near', 'value': 'near-id'}, {'label': 'Far', 'value': 'far-id'}]},
        ]}})


def test_native_questions_use_shared_controls_and_return_native_option_values():
    p = ready(); start(p)
    update = question(p)
    assert not update.outbound
    event = update.events[-1]
    assert event.kind == 'input_required'
    first, second = event.data['questions']
    assert first['choices'] == ['Cup', 'Block']
    assert first['multiple'] is False and first['allow_other'] is True
    assert 'The cup beside the plate.' in first['text']
    assert second['multiple'] is True and second['allow_other'] is True
    response = p.respond('input:question', {'0': ['Cup'], '1': ['Near', 'Shelf']})
    assert response.outbound == [{'id': 9, 'result': {
        'action': 'accept', 'content': {'answers': {
            'Which object?': ['cup-id'], 'Which locations?': ['near-id', 'Shelf']}}}}]
    with pytest.raises(ValueError, match='not pending'):
        p.respond('input:question', {'0': ['Cup'], '1': ['Far']})


@pytest.mark.parametrize('answers', [
    {'0': ['Cup']}, {'0': ['Cup', 'Block'], '1': ['Near']},
    {'0': ['Cup'], '1': ['Near', 'Near']}, {'0': ['Cup'], '1': ['Shelf', 'Drawer']},
    {'0': [' '], '1': ['Near']}, {'0': 'Cup', '1': ['Near']},
])
def test_invalid_question_answers_leave_native_request_available(answers):
    p = ready(); start(p); question(p)
    with pytest.raises(ValueError):
        p.respond('input:question', answers)
    assert p.respond('input:question', {'0': ['Cup'], '1': ['Near']}).outbound[0]['id'] == 9


def test_reannounced_question_keeps_controls_and_replies_to_latest_native_request():
    p = ready(); start(p); question(p, 9)
    update = question(p, 10)
    assert not update.events and not update.outbound
    response = p.respond('input:question', {'0': ['Cup'], '1': ['Near']})
    assert response.outbound[0]['id'] == 10
    assert not question(p, 11).events


def test_plan_approval_keeps_the_plan_and_returns_the_native_decision_value():
    p = ready(); start(p)
    update = p.receive({'id': 12, 'method': 'interaction/requestUserInput', 'params': {
        'sessionId': 'native', 'turnId': 'turn-native', 'requestId': 'plan',
        'toolName': 'ExitPlanMode', 'schema': {'interaction': 'plan_approval'},
        'input': {'plan': 'Inspect the camera, then write the measurement program.'},
        'questions': [{'question': 'Review this implementation plan.', 'header': 'Plan',
                       'options': [{'label': 'Approve', 'value': 'approve'}]}]}})
    assert 'Inspect the camera' in update.events[0].data['questions'][0]['text']
    response = p.respond('input:plan', {'0': ['Approve']}).outbound[0]
    assert response['result'] == {'action': 'accept', 'content': {'answers': {
        'Review this implementation plan.': ['approve']}}}


def test_unstructured_user_input_is_explicitly_unsupported():
    p = ready(); start(p)
    update = p.receive({'id': 12, 'method': 'interaction/requestUserInput', 'params': {
        'sessionId': 'native', 'turnId': 'turn-native', 'requestId': 'unknown', 'schema': {}}})
    assert update.outbound[0]['error']['code'] == -32601
    assert update.events[0].kind == 'unsupported_request'


def test_resolved_question_expires_controls():
    p = ready(); start(p); question(p)
    event(p, 'permission.resolved', {'requestId': 'question'})
    with pytest.raises(ValueError, match='not pending'):
        p.respond('input:question', {'0': ['Cup'], '1': ['Near']})


def test_interrupting_native_question_clears_controls_only_on_terminal_event():
    p = ready(); start(p); question(p)
    cancel = p.interrupt('input').outbound[0]
    reply(p, cancel, {})
    assert 'input:question' in p._requests
    done = event(p, 'turn.completed', {'inputId': 'input', 'resultType': 'cancelled'})
    assert [e.kind for e in done.events] == ['input_cancelled', 'turn_finished']
    with pytest.raises(ValueError, match='not pending'):
        p.respond('input:question', {'0': ['Cup'], '1': ['Near']})
    assert p.can_submit
