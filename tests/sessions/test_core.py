"""Shared-queue user scenarios against the actual persistent store."""

from concurrent.futures import ThreadPoolExecutor

import pytest

from openrua.agents.conversation import Event
from openrua.sessions.core import Session, initial_state
from openrua.sessions.store import SQLiteStore


@pytest.fixture
def session(tmp_path):
    store = SQLiteStore(tmp_path / "session.sqlite", initial_state())
    session = Session(store)
    session.native(Event("ready", data={"session_id": "native-1"}))
    yield session
    store.close()


def state(session):
    return session.store.snapshot()["state"]


def add(session, text="inspect", client="web", request="1"):
    return session.enqueue(client, request, text)["id"]


def finish(session, message_id, status="completed"):
    session.native(Event("turn_finished", message_id, {"status": status}))


def test_multiple_clients_submit_and_one_turn_is_delivered_at_a_time(session):
    first = add(session)
    second = add(session, "grasp", client="cli")
    assert session.take()["id"] == first
    assert session.take() is None
    session.native(Event("turn_started", first))
    finish(session, first)
    assert session.take()["id"] == second


def test_parallel_acceptance_is_ordered_and_retries_do_not_duplicate(session):
    with ThreadPoolExecutor(max_workers=8) as workers:
        ids = list(workers.map(lambda _: add(session), range(32)))
    assert len(set(ids)) == 1
    assert len(state(session)["messages"]) == 1
    accepted = [e for e in session.store.events() if e["kind"] == "message_accepted"]
    assert len(accepted) == 1
    with pytest.raises(ValueError, match="different content"):
        add(session, "different")


def test_edit_survives_original_send_retry_and_rejects_stale_revision(session):
    message = add(session)
    session.edit(message, "new instruction", 0)
    assert add(session) == message
    assert state(session)["messages"][0]["text"] == "new instruction"
    with pytest.raises(ValueError, match="refresh"):
        session.edit(message, "stale edit", 0)
    assert session.take()["text"] == "new instruction"
    with pytest.raises(ValueError):
        session.edit(message, "too late", 1)


def test_interrupt_before_native_ack_waits_then_sends_once(session):
    message = add(session)
    session.take()
    session.interrupt(message)
    assert session.take_interrupt() is None
    session.native(Event("turn_started", message))
    assert session.take_interrupt() == message
    assert session.take_interrupt() is None
    assert state(session)["paused"]
    session.native(Event("interrupt_acknowledged", message))
    assert state(session)["active"] == message


def test_interrupt_retains_queue_and_needs_explicit_resume_even_if_completed(session):
    first = add(session)
    second = add(session, "place", request="2")
    session.take()
    session.interrupt(first)
    finish(session, first)  # natural completion won the cancellation race
    assert session.take() is None
    assert state(session)["messages"][1]["status"] == "queued"
    session.resume(state(session)["pause_id"])
    assert session.take()["id"] == second
    with pytest.raises(ValueError):
        session.interrupt(first)
    assert state(session)["active"] == second


@pytest.mark.parametrize("outcome", ["failed", "interrupted"])
def test_abnormal_terminal_pauses_pending_messages(session, outcome):
    first = add(session)
    add(session, "next", request="2")
    session.take()
    finish(session, first, outcome)
    assert state(session)["paused"] and session.take() is None
    session.resume(state(session)["pause_id"])
    assert session.take()


def test_stale_resume_cannot_release_a_later_pause(session):
    first = add(session)
    session.take()
    finish(session, first, "failed")
    old_pause = state(session)["pause_id"]
    session.resume(old_pause)
    second = add(session, "next", request="2")
    session.take()
    finish(session, second, "failed")
    with pytest.raises(ValueError, match="pause state changed"):
        session.resume(old_pause)
    assert state(session)["paused"]


def test_reconnection_reads_journal_without_resubmitting(session):
    message = add(session)
    cursor = session.store.snapshot()["cursor"]
    session.take()
    session.native(Event("turn_started", message))
    session.native(Event("text_delta", message, {"text": "Working"}))
    first_read = session.store.events(after=cursor)
    assert first_read == session.store.events(after=cursor)
    assert len(state(session)["messages"]) == 1
    assert state(session)["active"] == message


def test_crash_after_delivery_intent_is_unknown_not_replayed(tmp_path):
    path = tmp_path / "session.sqlite"
    store = SQLiteStore(path, initial_state())
    original = Session(store)
    original.native(Event("ready", data={"session_id": "native-1"}))
    first = add(original)
    second = add(original, "next", request="2")
    original.take()  # could have crashed before OR after the native write
    store.close()
    restored_store = SQLiteStore(path, initial_state())
    restored = Session(restored_store)
    restored.recover()
    assert state(restored)["messages"][0]["status"] == "unknown"
    restored.native(Event("ready", data={"session_id": "native-1"}))
    with pytest.raises(ValueError, match="reconcile"):
        restored.resume(state(restored)["pause_id"])
    restored.resolve_unknown(first, "User checked the robot and will not repeat the grasp")
    restored.resume(state(restored)["pause_id"])
    assert restored.take()["id"] == second
    restored_store.close()


def test_protocol_error_pauses_without_turn_success(session):
    message = add(session)
    session.take()
    session.native(Event("protocol_error", message, {"error": "connection lost"}))
    assert state(session)["messages"][0]["status"] == "unknown"
    assert not state(session)["connected"]
    assert session.take() is None


def test_late_terminal_cannot_finish_new_turn(session):
    first = add(session)
    session.take()
    finish(session, first)
    second = add(session, "next", request="2")
    session.take()
    finish(session, first)
    assert state(session)["active"] == second


def test_question_reply_does_not_enter_regular_message_queue(session):
    message = add(session)
    session.take()
    session.native(Event("turn_started", message))
    session.native(Event("input_required", message, {"request_id": "question", "questions": []}))
    add(session, "next", request="2")
    session.claim_response("question")
    with pytest.raises(ValueError):
        session.claim_response("question")
    session.response_sent("question")
    assert not state(session)["requests"] and len(state(session)["messages"]) == 2
    assert session.take() is None


def test_close_retains_unfinished_records_and_stops_further_acceptance(session):
    message = add(session)
    session.take()
    session.close()
    assert state(session)["messages"][0]["status"] == "unknown"
    assert state(session)["closed"] and session.take() is None
    assert add(session) == message  # original send retry remains idempotent
    with pytest.raises(ValueError, match="closed"):
        add(session, "new", request="2")


def test_transaction_error_rolls_back_state_and_journal(session):
    before = session.store.snapshot()
    def fail(snapshot):
        snapshot["closed"] = True
        raise RuntimeError("storage operation rejected")
    with pytest.raises(RuntimeError):
        session.store.update(fail)
    assert session.store.snapshot() == before


def test_multiple_store_connections_share_acceptance_order(tmp_path):
    path = tmp_path / "session.sqlite"
    stores = [SQLiteStore(path, initial_state()) for _ in range(4)]
    sessions = [Session(store) for store in stores]
    try:
        with ThreadPoolExecutor(max_workers=4) as workers:
            list(workers.map(lambda i: add(sessions[i], client=f"client-{i}"), range(4)))
        snapshots = [state(s) for s in sessions]
        assert all(s == snapshots[0] for s in snapshots)
        assert len(snapshots[0]["messages"]) == 4
        assert len(stores[0].events()) == 4
    finally:
        for store in stores:
            store.close()
