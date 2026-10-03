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
