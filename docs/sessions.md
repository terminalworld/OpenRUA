# Shared session coordination

The `openrua.sessions` package coordinates user messages and native agent
connections. It does not plan robot actions or replace the coding agent's own
workflow. It is currently an internal foundation: the shared CLI/web service
and robot resource lifecycle still need to be connected to it.

## Boundaries

- `Session` defines accepted messages, dispatch intent, interruption, input
  requests, pause/resume and retained outcomes. It depends on a storage
  contract and normalized agent events.
- `Store` provides atomic state/event updates, snapshots and cursor-based
  event reads. `SQLiteStore` is the local implementation. A store is passed
  to the session explicitly; clients never need to know its SQL.
- `Execution` owns one native connection, serializes protocol calls and
  consumes the existing agent plugin's optional `Conversation` capability.
  Its transport is supplied through a factory. `StdioTransport` starts the
  plugin's command using pipes and saves native stderr to the specified file.
- Robot creation and shutdown remain outside this package. A resource owner
  reuses OpenRUA's existing bring-up path, supplies the conversation, and stops
  its resources before recording session closure. Closing a native connection
  is not proof that a robot stopped moving.

Import contracts enforce that coordination does not import vendor plugins,
robot backends, CLI code or benchmark execution. Agent plugins cannot import
session coordination. Storage and transport implementations can be replaced
through the supplied contracts without adding a second agent registry.

## User-visible semantics

Every submitted message has a client ID and request ID. Retrying the same
request returns the same accepted message; changing its content under the same
ID is rejected. Different clients share acceptance order. Editing a queued
message requires its current revision, and an old send retry cannot overwrite
that edit. A message already dispatched cannot be edited or withdrawn.

The owner records `dispatch_started` before writing to the native process.
One turn is outstanding at a time. Normal completion releases the next message.
Agent failure, interrupted execution, or uncertain delivery pauses the queue.
An explicit interrupt also pauses it immediately, even if normal completion
wins the cancellation race. Cancellation waits for the native turn identity,
and its acknowledgement alone does not complete a turn.

Resuming requires the current `pause_id`, so a delayed resume request cannot
release a newer pause. Unknown execution must first be reconciled with an
explicit note. Reconciliation retains the uncertainty and the note; it does
not silently turn the old operation into a success or replay it.

Tool questions and approval replies are associated with a pending request,
separate from queued new tasks. No automatic permission response is invented.
Unsupported native requests remain visible for diagnosis and pause scheduling.

## Persistence and reconnecting

A snapshot contains state and a matching event cursor. Clients reconnect by
reading events after that cursor. Reconnection does not submit a message,
restart the agent, or recover the execution owner.

A new execution owner takes an exclusive local lock and calls `recover`.
Any old dispatch without a terminal record becomes `unknown`; it stays paused
until checked. The lock prevents a second owner from starting another native
process for the same session. This is local process coordination, not a promise
of distributed failover or recovery of an already-running robot program.

State and events commit in one SQLite transaction. Native frames and normalized
events are retained for diagnosis; treat the journal and native logs as private
session data. Closing a session retains its messages, pending queue and unknown
outcomes. Explicit deletion remains separate from stopping resources.

## Validation scope

Tests exercise the real SQLite implementation, concurrent client acceptance,
separate store connections, retries after edits, cancellation ordering, crash
recovery after dispatch intent, cursor replay, stale events and exclusive
ownership. Execution tests use a fake native transport to inject failures.
They do not yet establish a working network client, live model cancellation,
or robot task execution. See [agents.md](agents.md#structured-conversations-experimental)
for the native plugin protocol and handshake checks.
