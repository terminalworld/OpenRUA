"""Claude native JSON ordering, including its untargeted interrupt protocol."""

import pytest

from openrua import agents
from openrua.plugins.agents.claude_conversation import ClaudeConversation


def ready():
    protocol = ClaudeConversation("session-1")
    request = protocol.begin().outbound[0]
    protocol.receive(ack(request))
    return protocol


def ack(request, success=True):
    return {"type": "control_response", "response": {
        "subtype": "success" if success else "error", "request_id": request["request_id"],
        "response": {}, "error": None if success else "rejected"}}


def started(protocol, turn_id="turn-1"):
    request = protocol.submit(turn_id, "inspect the scene").outbound[0]
    update = protocol.receive(request)
    assert update.events[-1].kind == "turn_started"
    return request


def result(protocol, is_error=False, uuid="result-1"):
    return protocol.receive({"type": "result", "session_id": "session-1", "uuid": uuid,
                             "is_error": is_error, "subtype": "success" if not is_error else "error_during_execution"})


def test_factory_preserves_native_login_and_workspace_customization():
    plugin = agents.get("claude-code")
    connection = plugin.conversation("box", "model", "http://proxy")
    assert connection.argv[:3] == ["docker", "exec", "-i"]
    assert "CLAUDE_CONFIG_DIR=/claude-config" in connection.argv
    assert "--session-id" in connection.argv and "--resume" not in connection.argv
    assert "--replay-user-messages" in connection.argv
    assert not set(connection.argv) & {"--bare", "--no-session-persistence", "--setting-sources"}
    resumed = plugin.conversation("box", "model", "http://proxy", session_id="original")
    assert resumed.argv[resumed.argv.index("--resume") + 1] == "original"
    assert "--session-id" not in resumed.argv


def test_ready_identity_is_confirmed_by_the_native_session():
    protocol = ready()
    request = protocol.submit("turn-1", "text").outbound[0]
    update = protocol.receive(request)
    assert [e.kind for e in update.events] == ["session", "turn_started"]
    assert update.events[0].data["identity_confirmed"]
    with pytest.raises(ValueError, match="identity"):
        protocol.receive({"type": "system", "session_id": "other"})
    assert not protocol.can_submit


@pytest.mark.parametrize("ack_first", [True, False])
def test_interrupt_waits_for_ack_and_result_before_next_turn(ack_first):
    protocol = ready()
    started(protocol)
    cancel = protocol.interrupt("turn-1").outbound[0]
    assert cancel["request"] == {"subtype": "interrupt"}
    if ack_first:
        update = protocol.receive(ack(cancel))
        assert update.events[0].kind == "interrupt_acknowledged"
    else:
        update = result(protocol)
        assert update.events[0].kind == "turn_finished"
    assert not protocol.can_submit
    with pytest.raises(ValueError):
        protocol.submit("turn-2", "text")
    if ack_first:
        update = result(protocol)
        assert update.events[0].data["interruption_requested"]
    else:
        protocol.receive(ack(cancel))
    assert protocol.can_submit
    started(protocol, "turn-2")
    with pytest.raises(ValueError):
        protocol.interrupt("turn-1")


def test_duplicate_interrupt_sends_no_second_untargeted_request():
    protocol = ready()
    started(protocol)
    protocol.interrupt("turn-1")
    assert not protocol.interrupt("turn-1").outbound


def test_result_without_echo_is_not_assigned_to_unacknowledged_input():
    protocol = ready()
    protocol.submit("turn-1", "text")
    update = result(protocol)
    assert update.events[-1].kind == "protocol_error"
    assert not any(e.kind == "turn_finished" for e in update.events)
    assert not protocol.can_submit


def test_failed_interrupt_does_not_release_next_turn():
    protocol = ready()
    started(protocol)
    cancel = protocol.interrupt("turn-1").outbound[0]
    update = protocol.receive(ack(cancel, success=False))
    assert update.events[0].kind == "protocol_error"
    result(protocol)
    assert not protocol.can_submit


def test_completed_and_failed_results_keep_the_local_turn_id():
    for error in [False, True]:
        protocol = ready()
        started(protocol)
        event = result(protocol, is_error=error).events[0]
        assert event.kind == "turn_finished" and event.turn_id == "turn-1"
        assert event.data["status"] == ("failed" if error else "completed")


def test_duplicates_cannot_complete_later_turn():
    protocol = ready()
    started(protocol)
    result(protocol)
    started(protocol, "turn-2")
    assert result(protocol).events[0].kind == "native"
    assert not protocol.can_submit
    assert result(protocol, uuid="result-2").events[0].turn_id == "turn-2"


def control_request(protocol, request):
    return protocol.receive({"type": "control_request", "request_id": "q1", "request": request})


def test_question_response_uses_original_question_text_and_tool_input():
    protocol = ready()
    started(protocol)
    update = control_request(protocol, {"subtype": "can_use_tool", "tool_name": "AskUserQuestion",
        "input": {"questions": [{"question": "Which bowl?", "options": [{"label": "green"}]}]}})
    assert not update.outbound
    assert update.events[0].data["questions"][0]["id"] == "0"
    response = protocol.respond("q1", {"0": ["green"]}).outbound[0]
    assert response["response"]["request_id"] == "q1"
    assert response["response"]["response"]["updatedInput"]["answers"] == {"Which bowl?": "green"}
    with pytest.raises(ValueError):
        protocol.respond("q1", {"0": ["green"]})


def test_permission_is_explicit_and_cancelled_requests_cannot_be_answered():
    protocol = ready()
    started(protocol)
    request = {"subtype": "can_use_tool", "tool_name": "Bash", "input": {"command": "ls"}}
    assert not control_request(protocol, request).outbound
    response = protocol.respond("q1", {"decision": ["deny"]}).outbound[0]
    assert response["response"]["response"]["behavior"] == "deny"
    control_request(protocol, request)
    protocol.receive({"type": "control_cancel_request", "request_id": "q1"})
    with pytest.raises(ValueError):
        protocol.respond("q1", {"decision": ["allow"]})


def test_message_and_tool_events_have_common_display_fields():
    protocol = ready()
    started(protocol)
    update = protocol.receive({"type": "assistant", "message": {"id": "m1", "content": [
        {"type": "text", "text": "Inspecting"},
        {"type": "tool_use", "id": "tool1", "name": "Bash", "input": {"command": "ls"}}]}})
    assert {e.data["kind"] for e in update.events} == {"message", "tool"}
    assert all(e.turn_id == "turn-1" and "item_id" in e.data for e in update.events)


def tool_call(protocol, tool_id='tool1'):
    return protocol.receive({'type': 'assistant', 'message': {'id': 'm1', 'content': [
        {'type': 'tool_use', 'id': tool_id, 'name': 'Bash', 'input': {'command': 'ls'}}]}})


def tool_result(protocol, content, *, tool_id='tool1', is_error=False):
    return protocol.receive({'type': 'user', 'message': {'role': 'user', 'content': [
        {'type': 'tool_result', 'tool_use_id': tool_id, 'content': content, 'is_error': is_error}]}})


@pytest.mark.parametrize('is_error', [False, True])
def test_native_tool_result_updates_original_item_without_finishing_turn(is_error):
    protocol = ready()
    started(protocol)
    tool_call(protocol)
    update = tool_result(protocol, 'native command output', is_error=is_error)
    event = next(e for e in update.events if e.kind == 'item')
    assert event.turn_id == 'turn-1'
    assert event.data == {'phase': 'failed' if is_error else 'completed', 'kind': 'tool',
                         'item_id': 'tool1', 'text': 'Bash', 'output': 'native command output'}
    assert not any(e.kind == 'turn_finished' for e in update.events)
    assert not protocol.can_submit


def test_tool_content_blocks_keep_text_without_dumping_image_payload():
    protocol = ready()
    started(protocol)
    tool_call(protocol)
    update = tool_result(protocol, [
        {'type': 'text', 'text': 'camera saved'},
        {'type': 'image', 'source': {'type': 'base64', 'data': 'IMAGE_BYTES'}},
        {'type': 'text', 'text': '640 x 480'}])
    event = next(e for e in update.events if e.kind == 'item')
    assert event.data['output'] == 'camera saved\n[image result in native record]\n640 x 480'
    assert 'IMAGE_BYTES' not in str(event.data)


def test_late_tool_result_keeps_its_original_turn_and_duplicates_do_not_regress_it():
    protocol = ready()
    started(protocol)
    tool_call(protocol)
    result(protocol)
    started(protocol, 'turn-2')
    event = next(e for e in tool_result(protocol, 'done').events if e.kind == 'item')
    assert event.turn_id == 'turn-1'
    assert not any(e.kind == 'item' for e in tool_result(protocol, 'done').events)
    assert not any(e.kind == 'item' for e in tool_call(protocol).events)
    assert not protocol.can_submit


def test_unknown_tool_result_is_retained_without_assigning_it_to_current_turn():
    protocol = ready()
    started(protocol)
    assert not any(e.kind == 'item' for e in tool_result(protocol, 'unknown result').events)
    assert not protocol.can_submit


def test_a_message_typed_during_a_turn_joins_it():
    # Checked against the CLI: the message is handed to the model after the
    # running tool calls, echoed in place, and the turn ends with one result.
    protocol = ready()
    started(protocol)
    assert protocol.can_steer
    frame = protocol.steer("turn-1", "message-2", "the ball moved left").outbound[0]
    assert frame["type"] == "user" and frame["message"]["content"] == "the ball moved left"
    echo = protocol.receive({**frame, "session_id": "session-1"})
    assert [(e.kind, e.turn_id, e.data) for e in echo.events] == [
        ("steer_delivered", "turn-1", {"message_id": "message-2"})]
    assert result(protocol).events[-1].kind == "turn_finished"


def test_a_steer_that_arrives_after_the_turn_becomes_its_own_turn():
    protocol = ready()
    started(protocol)
    frame = protocol.steer("turn-1", "message-2", "and then place it").outbound[0]
    result(protocol)
    update = protocol.receive({**frame, "session_id": "session-1"})
    assert [(e.kind, e.turn_id) for e in update.events] == [("turn_started", "message-2")]
    assert result(protocol, uuid="result-2").events[-1].turn_id == "message-2"


def test_output_with_no_turn_open_is_a_turn_of_the_agents_own():
    # A background task finished after the turn ended and the CLI went on;
    # its output used to be dropped, then counted under the next message.
    protocol = ready()
    started(protocol)
    result(protocol)
    update = protocol.receive({"type": "assistant", "session_id": "session-1", "message": {
        "id": "msg-9", "content": [{"type": "text", "text": "The trajectory finished."}]}})
    first = update.events[0]
    assert first.kind == "turn_started" and first.data == {"origin": "agent"}
    assert update.events[1].turn_id == first.turn_id
    assert result(protocol, uuid="result-9").events[-1].turn_id == first.turn_id
    assert protocol.can_submit


def test_a_question_takes_an_answer_of_ones_own():
    protocol = ready()
    started(protocol)
    asked = protocol.receive({"type": "control_request", "request_id": "q-1", "request": {
        "subtype": "can_use_tool", "tool_name": "AskUserQuestion", "input": {"questions": [{
            "question": "How should I proceed?", "header": "Next",
            "options": [{"label": "Look first"}, {"label": "Retry"}], "multiSelect": False}]}}})
    assert asked.events[0].data["questions"][0]["allow_other"] is True
    reply = protocol.respond("q-1", {"0": ["use position control, then try other joints"]})
    answers = reply.outbound[0]["response"]["response"]["updatedInput"]["answers"]
    assert answers == {"How should I proceed?": "use position control, then try other joints"}


def test_initialize_reports_the_agents_commands_and_its_permissions():
    protocol = ClaudeConversation("session-1", permissions="default")
    request = protocol.begin().outbound[0]
    response = ack(request)
    response["response"]["response"] = {"commands": [
        {"name": "compact", "description": "Clear history, keep a summary"}, {"bad": True}]}
    events = protocol.receive(response).events
    assert [e.kind for e in events] == ["ready", "setting", "setting", "commands"]
    assert events[1].data['setting'] == 'permissions' and events[1].data['value'] == 'default'
    assert any(choice['value'] == 'auto' for choice in events[1].data['choices'])
    assert events[3].data["commands"] == [{"name": "compact",
                                           "description": "Clear history, keep a summary"}]


@pytest.mark.parametrize("setting,value,subtype,field", [
    ("permissions", "default", "set_permission_mode", ("mode", "default")),
    ("permissions", "bypassPermissions", "set_permission_mode", ("mode", "bypassPermissions")),
    ("permissions", "auto", "set_permission_mode", ("mode", "auto")),
    ("permissions", "acceptEdits", "set_permission_mode", ("mode", "acceptEdits")),
    ("permissions", "plan", "set_permission_mode", ("mode", "plan")),
    ("permissions", "dontAsk", "set_permission_mode", ("mode", "dontAsk")),
    ("model", "claude-haiku-4-5", "set_model", ("model", "claude-haiku-4-5")),
])
def test_settings_go_through_the_clis_own_controls(setting, value, subtype, field):
    protocol = ready()
    request = protocol.configure(setting, value).outbound[0]
    assert request["request"]["subtype"] == subtype
    assert request["request"][field[0]] == field[1]
    events = protocol.receive(ack(request)).events
    assert [(e.kind, e.data["setting"], e.data["value"]) for e in events] == [("setting", setting, value)]
    assert protocol.can_submit


def test_a_refused_setting_is_reported_and_the_connection_stays_usable():
    protocol = ready()
    request = protocol.configure("model", "no-such-model").outbound[0]
    assert protocol.receive(ack(request, success=False)).events[0].kind == "setting_rejected"
    assert protocol.can_submit
    with pytest.raises(ValueError):
        protocol.configure("permissions", "sometimes")


def test_refused_native_auto_mode_keeps_the_last_acknowledged_permissions():
    protocol = ready()
    request = protocol.configure('permissions', 'bypassPermissions').outbound[0]
    assert protocol.permissions == 'auto'
    event = protocol.receive(ack(request, success=False)).events[0]
    assert event.kind == 'setting_rejected' and event.data['reason'] == 'rejected'
    assert protocol.permissions == 'auto' and protocol.can_submit


def test_the_model_each_request_runs_on_is_reported():
    protocol = ready()
    started(protocol)
    events = protocol.receive({"type": "system", "subtype": "init", "session_id": "session-1",
                               "model": "claude-sonnet-5-5"}).events
    assert ("setting", "model") in [(e.kind, e.data.get("setting")) for e in events]


def test_clear_begins_a_new_native_conversation():
    protocol = ready()
    request = protocol.submit("turn-1", "/clear").outbound[0]
    protocol.receive(request)
    events = protocol.receive({"type": "system", "subtype": "init", "session_id": "session-2"}).events
    assert events[0].kind == "session" and events[0].data == {
        "session_id": "session-2", "identity_confirmed": True, "cleared": True}
    assert protocol.session_id == "session-2"
    with pytest.raises(ValueError, match="identity"):
        protocol.receive({"type": "system", "session_id": "session-3"})


def test_a_session_that_asks_starts_in_the_default_mode_and_may_switch():
    plugin = agents.get("claude-code")
    asking = plugin.conversation("box", "model", "", permissions="default").argv
    assert asking[asking.index("--permission-mode") + 1] == "default"
    assert "--allow-dangerously-skip-permissions" in asking
    assert "--dangerously-skip-permissions" not in asking
    running = plugin.conversation("box", "model", "").argv
    assert running[running.index('--permission-mode') + 1] == 'auto'
