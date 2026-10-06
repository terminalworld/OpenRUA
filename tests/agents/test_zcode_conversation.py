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
