"""Native protocol ordering and identity, without model or robot calls."""

import pytest

from openrua import agents
from openrua.agents.base import Agent
from openrua.agents.conversation import Conversation, ConversationProtocol
from openrua.plugins.agents.codex_conversation import CodexConversation


def ready(session_id=None):
    protocol = CodexConversation("model", session_id=session_id)
    start = protocol.begin().outbound[0]
    update = protocol.receive({"id": start["id"], "result": {}})
    assert update.outbound[0] == {"method": "initialized", "params": {}}
    request = update.outbound[1]
    assert request["method"] == ("thread/resume" if session_id else "thread/start")
    protocol.receive({"id": request["id"], "result": {"thread": {"id": session_id or "thread-1"}}})
    return protocol


def started(protocol, local="message-1", native="turn-1"):
    request = protocol.submit(local, "inspect the table").outbound[0]
    update = protocol.receive({"id": request["id"], "result": {"turn": {"id": native}}})
    assert update.events[0].kind == "turn_started"
    assert update.events[0].turn_id == local


def completed(protocol, native="turn-1", status="completed"):
    return protocol.receive({"method": "turn/completed", "params": {
        "threadId": protocol.thread_id, "turn": {"id": native, "status": status}}})


def test_optional_factory_keeps_batch_only_plugins_compatible():
    class BatchOnly(Agent):
        name, default_model = "batch", "model"
        def launch_argv(self, *a, **kw):
            return ["docker", "exec", "box", "agent"]
    plugin = BatchOnly()
    assert plugin.conversation("box", "model", "http://proxy") is None
    assert "conversation" not in plugin.capabilities


def test_codex_factory_uses_native_process_and_existing_profile():
    agent = agents.get("codex")
    connection = agent.conversation("box", "model", "http://proxy", session_id="original")
    assert isinstance(connection, Conversation)
    assert isinstance(connection.protocol, ConversationProtocol)
    assert connection.argv[:3] == ["docker", "exec", "-i"]
    assert "-it" not in connection.argv
    assert "CODEX_HOME=/codex-home" in connection.argv
    assert "app-server" in connection.argv and "stdio://" in connection.argv
    assert "--last" not in connection.argv
    assert "conversation" in agent.capabilities


def test_resume_requires_exact_native_identity():
    protocol = CodexConversation("model", session_id="original")
    request = protocol.begin().outbound[0]
    request = protocol.receive({"id": request["id"], "result": {}}).outbound[-1]
    assert request["params"]["threadId"] == "original"
    with pytest.raises(ValueError, match="identity"):
        protocol.receive({"id": request["id"], "result": {"thread": {"id": "other"}}})
    assert not protocol.ready


def test_start_rejects_unready_busy_and_duplicate_turns():
    protocol = CodexConversation("model")
    with pytest.raises(ValueError):
        protocol.submit("message-1", "text")
    protocol = ready()
    started(protocol)
    with pytest.raises(ValueError):
        protocol.submit("message-2", "text")
    assert completed(protocol).events[0].data["status"] == "completed"
    with pytest.raises(ValueError, match="replay"):
        protocol.submit("message-1", "text")
    started(protocol, "message-2", "turn-2")


def test_early_events_are_correlated_only_after_start_acknowledgement():
    protocol = ready()
    request = protocol.submit("message-1", "text").outbound[0]
    early = protocol.receive({"method": "item/agentMessage/delta", "params": {
        "threadId": "thread-1", "turnId": "turn-1", "itemId": "item-1", "delta": "Hello"}})
    assert not early.events
    assert not completed(protocol).events
    update = protocol.receive({"id": request["id"], "result": {"turn": {"id": "turn-1"}}})
    assert [e.kind for e in update.events] == ["turn_started", "text_delta", "turn_finished"]
    assert all(e.turn_id == "message-1" for e in update.events)
    started(protocol, "message-2", "turn-2")


def test_cancel_acknowledgement_does_not_finish_turn():
    protocol = ready()
    started(protocol)
    request = protocol.interrupt("message-1").outbound[0]
    assert request["params"] == {"threadId": "thread-1", "turnId": "turn-1"}
    update = protocol.receive({"id": request["id"], "result": {}})
    assert [e.kind for e in update.events] == ["interrupt_acknowledged"]
    with pytest.raises(ValueError):
        protocol.submit("message-2", "text")
    assert completed(protocol, status="interrupted").events[0].data["status"] == "interrupted"


def test_late_cancel_response_cannot_finish_the_next_turn():
    protocol = ready()
    started(protocol)
    request = protocol.interrupt("message-1").outbound[0]
    completed(protocol)
    started(protocol, "message-2", "turn-2")
    update = protocol.receive({"id": request["id"], "result": {}})
    assert update.events[0].turn_id == "message-1"
    with pytest.raises(ValueError):
        protocol.interrupt("message-1")
    assert protocol.interrupt("message-2").outbound[0]["params"]["turnId"] == "turn-2"


def test_old_or_other_thread_terminal_event_does_not_clear_active_turn():
    protocol = ready()
    started(protocol)
    assert completed(protocol, native="old").events[0].kind == "native"
    update = protocol.receive({"method": "turn/completed", "params": {
        "threadId": "other", "turn": {"id": "turn-1", "status": "completed"}}})
    assert update.events[0].kind == "native"
    with pytest.raises(ValueError):
        protocol.submit("message-2", "text")
    assert completed(protocol).events[0].kind == "turn_finished"


def test_failed_rpc_does_not_silently_replay_or_claim_terminal_state():
    protocol = ready()
    request = protocol.submit("message-1", "text").outbound[0]
    update = protocol.receive({"id": request["id"], "error": {"message": "connection lost"}})
    assert update.events[0].kind == "protocol_error"
    assert update.events[0].turn_id == "message-1"
    assert not update.outbound
    with pytest.raises(ValueError):
        protocol.submit("message-2", "text")


def input_request(protocol, method, **params):
    return protocol.receive({"id": 42, "method": method, "params": {
        "threadId": "thread-1", "turnId": "turn-1", **params}})


def test_user_input_answers_stay_with_the_request_and_expire_with_turn():
    protocol = ready()
    started(protocol)
    update = input_request(protocol, "item/tool/requestUserInput", questions=[
        {"id": "target", "question": "Which bowl?", "options": [{"label": "green"}]}])
    event = update.events[0]
    assert event.kind == "input_required" and event.turn_id == "message-1"
    key = event.data["request_id"]
    with pytest.raises(ValueError):
        protocol.respond(key, {"wrong": ["green"]})
    response = protocol.respond(key, {"target": ["green"]}).outbound[0]
    assert response == {"id": 42, "result": {"answers": {"target": {"answers": ["green"]}}}}
    with pytest.raises(ValueError):
        protocol.respond(key, {"target": ["green"]})
    input_request(protocol, "item/tool/requestUserInput", questions=[])
    completed(protocol)
    with pytest.raises(ValueError):
        protocol.respond(key, {})


def test_approval_requires_explicit_allowed_answer_and_never_auto_accepts():
    protocol = ready()
    started(protocol)
    update = input_request(protocol, "item/commandExecution/requestApproval", reason="Run command?")
    assert not update.outbound
    key = update.events[0].data["request_id"]
    with pytest.raises(ValueError):
        protocol.respond(key, {"decision": ["acceptForSession"]})
    assert protocol.respond(key, {"decision": ["decline"]}).outbound == [
        {"id": 42, "result": {"decision": "decline"}}]


def test_unsupported_requests_are_visible_without_fabricated_reply():
    protocol = ready()
    started(protocol)
    update = input_request(protocol, "future/request")
    assert update.events[0].kind == "unsupported_request"
    assert not update.outbound


def test_native_workspace_is_an_explicit_protocol_parameter():
    protocol = CodexConversation("model", workspace="/tmp/isolated-workspace")
    request = protocol.begin().outbound[0]
    request = protocol.receive({"id": request["id"], "result": {}}).outbound[-1]
    assert request["params"]["cwd"] == "/tmp/isolated-workspace"


def test_codex_takes_a_message_during_a_turn_through_steer():
    protocol = ready()
    started(protocol)
    request = protocol.steer("message-1", "message-2", "the cup is on the left").outbound[0]
    assert request["method"] == "turn/steer"
    assert request["params"]["expectedTurnId"] == "turn-1"
    done = protocol.receive({"id": request["id"], "result": {"turnId": "turn-1"}})
    assert [(e.kind, e.data["message_id"]) for e in done.events] == [("steer_delivered", "message-2")]
    late = protocol.steer("message-1", "message-3", "too late").outbound[0]
    refused = protocol.receive({"id": late["id"], "error": {"message": "no active turn to steer"}})
    assert refused.events[0].kind == "steer_rejected" and protocol.ready


def test_codex_turn_the_server_starts_is_the_agents_own():
    protocol = ready()
    update = protocol.receive({"method": "turn/started", "params": {
        "threadId": "thread-1", "turn": {"id": "turn-9"}}})
    assert update.events[0].kind == "turn_started" and update.events[0].data["origin"] == "agent"
    assert completed(protocol, "turn-9").events[0].kind == "turn_finished"


def test_codex_items_read_as_a_person_reads_them():
    protocol = ready()
    started(protocol)
    def item(body):
        return protocol.receive({"method": "item/completed", "params": {
            "threadId": "thread-1", "turnId": "turn-1", "item": body}}).events[0]
    assert item({"type": "userMessage", "id": "u", "content": []}).kind == "native"
    assert item({"type": "reasoning", "id": "r", "summary": [], "content": []}).kind == "native"
    command = item({"type": "commandExecution", "id": "c", "command": "/bin/bash -lc 'ls'",
                    "cwd": "/workspace", "aggregatedOutput": "a\n"})
    assert command.data["text"] == "/bin/bash -lc 'ls'" and "command" not in command.data["details"]
    asked = item({"type": "agentMessage", "id": "q", "text": "Is the e-stop released?",
                  "delivery": "async", "questions": [{"title": "Is the e-stop released?"}]})
    assert asked.data["kind"] == "question"


def test_codex_reports_settings_and_commands_when_ready():
    protocol = CodexConversation("model", permissions="ask")
    start = protocol.begin().outbound[0]
    request = protocol.receive({"id": start["id"], "result": {}}).outbound[1]
    assert request["params"]["approvalPolicy"] == "untrusted"
    events = protocol.receive({"id": request["id"], "result": {"thread": {"id": "thread-1"}}}).events
    assert [e.kind for e in events] == ["ready", "setting", "setting", "commands"]
    assert {c["name"] for c in events[-1].data["commands"]} == {"compact", "clear"}


def test_codex_settings_apply_from_the_next_turn():
    protocol = ready()
    assert protocol.configure("permissions", "ask").events[0].data == {
        "setting": "permissions", "value": "ask"}
    protocol.configure("model", "gpt-6-astra")
    params = protocol.submit("message-1", "text").outbound[0]["params"]
    assert params["approvalPolicy"] == "untrusted" and params["model"] == "gpt-6-astra"
    with pytest.raises(ValueError):
        protocol.configure("permissions", "sometimes")


def test_codex_compact_runs_as_the_servers_turn():
    protocol = ready()
    request = protocol.submit("message-1", "/compact").outbound[0]
    assert request["method"] == "thread/compact/start"
    assert not protocol.receive({"id": request["id"], "result": {}}).events
    update = protocol.receive({"method": "turn/started", "params": {
        "threadId": "thread-1", "turn": {"id": "turn-9"}}})
    assert [(e.kind, e.turn_id) for e in update.events] == [("turn_started", "message-1")]
    assert completed(protocol, native="turn-9").events[0].turn_id == "message-1"


def test_codex_clear_starts_a_new_thread():
    protocol = ready()
    request = protocol.submit("message-1", "/clear").outbound[0]
    assert request["method"] == "thread/start"
    events = protocol.receive({"id": request["id"], "result": {"thread": {"id": "thread-2"}}}).events
    assert [e.kind for e in events] == ["session", "turn_finished"]
    assert events[0].data == {"session_id": "thread-2", "cleared": True}
    assert protocol.thread_id == "thread-2" and protocol.can_submit
