"""Native failures, identity, streaming and interaction boundaries for Kimi."""
from copy import deepcopy

import pytest

from openrua.plugins.agents.kimi_conversation import KimiConversation


def reply(protocol, update, result):
    return protocol.receive({'id': update.outbound[0]['id'], 'result': result})


def connected(sid=None):
    p = KimiConversation('/workspace', 'model', sid)
    result = reply(p, p.begin(), {'id': sid or 'session-1'})
    assert result.events[0].kind == 'ready'
    return p


def submit(p):
    reply(p, p.submit('prompt-1', 'inspect'), {'prompt_id': 'prompt-1'})


def snapshot(p, state='running', text='Hello', **extra):
    return {'type': 'snapshot', 'session_id': p.session_id, 'prompt_id': 'prompt-1',
            'transcript': {'items': [{'kind': 'turn', 'turnId': 't0', 'ordinal': 0, 'triggerPromptId': 'prompt-1',
                'state': state, 'error': 'provider failure' if state == 'failed' else None,
                'steps': [{'frames': [{'kind': 'text', 'role': 'assistant', 'frameId': 'f0', 'text': text}]}]}]},
            **extra}


@pytest.mark.parametrize(('native', 'status'), [('completed', 'completed'), ('failed', 'failed'), ('cancelled', 'interrupted')])
def test_terminal_status_uses_correlated_native_transcript(native, status):
    p = connected(); submit(p)
    update = p.receive(snapshot(p, native))
    assert update.events[-1].kind == 'turn_finished'
    assert update.events[-1].data['status'] == status
    assert p.can_submit
    assert p.receive(snapshot(p, native)).events == []


def test_duplicate_and_foreign_snapshots_do_not_advance_turn_or_repeat_text():
    p = connected(); submit(p)
    frame = snapshot(p)
    assert [e.kind for e in p.receive(frame).events] == ['turn_started', 'text_delta']
    assert p.receive(frame).events == []
    assert p.receive(snapshot(p, text='Hello there')).events[0].data['text'] == ' there'
    for field in ('session_id', 'prompt_id'):
        other = {**snapshot(p, 'completed'), field: 'foreign'}
        assert not p.receive(other).events and not p.can_submit
    other = snapshot(p, 'completed')
    other['transcript']['items'][0]['triggerPromptId'] = 'old'
    assert not p.receive(other).events and not p.can_submit


def test_cancel_ack_is_not_completion_and_late_ack_blocks_next_submission():
    p = connected(); submit(p)
    cancel = p.interrupt('prompt-1')
    assert not p.interrupt('prompt-1').outbound
    p.receive(snapshot(p, 'cancelled'))
    assert not p.can_submit
    assert reply(p, cancel, {'aborted': True}).events[0].kind == 'interrupt_acknowledged'
    assert p.can_submit


def test_protocol_errors_and_unknown_terminal_states_fail_closed():
    p = connected()
    update = p.submit('prompt-1', 'inspect')
    result = p.receive({'id': update.outbound[0]['id'], 'error': 'model unavailable'})
    assert result.events[0].kind == 'protocol_error' and not p.can_submit
    p = connected(); submit(p)
    with pytest.raises(ValueError, match='unknown native'):
        p.receive(snapshot(p, 'end_turn'))


def test_identity_is_checked_on_resume_and_submit():
    p = KimiConversation('/workspace', 'model', 'saved')
    with pytest.raises(ValueError, match='identity differs'):
        reply(p, p.begin(), {'id': 'other'})
    p = connected()
    with pytest.raises(ValueError, match='acknowledgement differs'):
        reply(p, p.submit('prompt-1', 'inspect'), {'prompt_id': 'other'})


def approval(p):
    return {'approval_id': 'a', 'session_id': p.session_id, 'turn_id': 0,
            'tool_name': 'Write', 'tool_input_display': {'path': 'probe.txt'}}


def test_permission_is_explicit_one_time_and_cancelled_when_native_request_expires():
    p = connected(); submit(p)
    frame = snapshot(p, approvals=[approval(p)])
    event = p.receive(frame).events[-1]
    assert event.kind == 'input_required'
    assert event.data['details']['tool_name'] == 'Write'
    with pytest.raises(ValueError, match='exactly one'):
        p.respond('approvals:a', {'decision': ['allow', 'deny']})
    request = p.respond('approvals:a', {'decision': ['allow']}).outbound[0]
    assert request['params']['body'] == {'decision': 'approved'}  # no remembered session-wide permission
    assert not p.receive(frame).events
    update = p.receive(snapshot(p))
    assert update.events[-1].kind == 'input_cancelled'
    with pytest.raises(ValueError, match='not pending'):
        p.respond('approvals:a', {'decision': ['allow']})


def test_foreign_permission_does_not_reach_user():
    p = connected(); submit(p)
    item = {**approval(p), 'turn_id': 'foreign'}
    assert all(e.kind != 'input_required' for e in p.receive(snapshot(p, approvals=[item])).events)


def test_question_labels_map_to_native_option_ids_and_free_text_stays_explicit():
    p = connected(); submit(p)
    question = {'question_id': 'q', 'session_id': p.session_id, 'turn_id': 0, 'questions': [
        {'id': 'part', 'question': 'Which object?', 'options': [{'id': 'o1', 'label': 'Block'}]}]}
    p.receive(snapshot(p, questions=[question]))
    params = p.respond('questions:q', {'part': ['Block']}).outbound[0]['params']
    assert params['body'] == {'answers': {'part': {'kind': 'single', 'option_id': 'o1'}}}
    with pytest.raises(ValueError, match='free-text'):
        p.respond('questions:q', {'part': ['Cup']})
    question = deepcopy(question)
    question['questions'][0].update(allow_other=True, multi_select=True)
    p.receive(snapshot(p, questions=[question]))
    body = p.respond('questions:q', {'part': ['Block', 'Cup']}).outbound[0]['params']['body']
    assert body['answers']['part'] == {'kind': 'multi_with_other', 'option_ids': ['o1'], 'other_text': 'Cup'}


def test_tool_details_and_changes_are_preserved():
    p = connected(); submit(p)
    frame = snapshot(p)
    tool = {'kind': 'tool', 'frameId': 'tool1', 'name': 'Write', 'state': 'running', 'input': {'path': 'probe'}}
    frame['transcript']['items'][0]['steps'][0]['frames'].append(tool)
    assert p.receive(frame).events[-1].data['details'] == tool
    frame = deepcopy(frame)
    frame['transcript']['items'][0]['steps'][0]['frames'][-1]['state'] = 'done'
    event = p.receive(frame).events[-1]
    assert event.kind == 'item' and event.data['phase'] == 'completed'


def test_native_question_capabilities_use_the_shared_frontend_contract():
    p = connected(); submit(p)
    question = {'question_id': 'q', 'session_id': p.session_id, 'turn_id': 0, 'questions': [
        {'id': 'locations', 'question': 'Which locations?', 'multi_select': True, 'allow_other': True,
         'options': [{'id': 'near', 'label': 'Near'}, {'id': 'far', 'label': 'Far'}]}]}
    event = p.receive(snapshot(p, questions=[question])).events[-1]
    assert event.data['questions'] == [{'id': 'locations', 'text': 'Which locations?',
        'choices': ['Near', 'Far'], 'secret': False, 'multiple': True, 'allow_other': True}]
    body = p.respond(event.data['request_id'], {'locations': ['Near', 'Far', 'Shelf']}).outbound[0]['params']['body']
    assert body == {'answers': {'locations': {'kind': 'multi_with_other',
                                            'option_ids': ['near', 'far'], 'other_text': 'Shelf'}}}
