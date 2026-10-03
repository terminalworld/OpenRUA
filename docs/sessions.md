# Shared session coordination

The `openrua.sessions` package coordinates user messages and native agent
connections. It does not plan robot actions or replace the coding agent's own
workflow. The experimental local service connects the existing robot startup path to
this coordinator. CLI clients use its HTTP API; a browser UI is not yet included.

## Start and connect

Use the same robot, simulator, benchmark and agent selections as `up`. The
sandbox images and native agent login must already be configured; see
[install.md](install.md). For example, in the execution terminal:

```sh
openrua serve panda --sim robosuite --name shared --agent claude-code
```

`serve` stays in the foreground and owns the robot, sandbox and native agent
process. Keep it running when leaving a client. In another terminal:

```sh
openrua chat --name shared
openrua chat --name shared "Inspect the scene before moving."
openrua session --name shared send "Describe what you observed."
openrua session --name shared status
```

Each client may submit to the same queue. `chat` displays native text and tool
activity and waits for its message; `session send` returns after acceptance,
so another task can be queued while the current one runs. Ctrl-C in **chat**
only detaches. To reconnect without submitting a message:

```sh
openrua chat --name shared --follow
openrua chat --name shared --follow --after 42
openrua session --name shared events --after 42
```

The cursor is an event sequence number. Raw and normalized events are both
retained; reconnecting never reruns old commands. `send` prints its client and
request IDs before sending. After a lost acknowledgement, inspect status or
retry the same text using both `--client-id` and `--request-id`.

## Pause, input and end

`status` includes message IDs, queued message revisions, `pause_id`, and pending
questions. Use those identities explicitly:

```sh
openrua session --name shared interrupt MESSAGE_ID
openrua session --name shared edit MESSAGE_ID "Revised instruction" --revision 0
openrua session --name shared withdraw MESSAGE_ID
openrua session --name shared resume PAUSE_ID
openrua session --name shared respond REQUEST_ID '{"question-id":["answer"]}'
```

Interrupting the agent is not an emergency stop for the robot. After interruption
or failure the queue remains paused until explicitly resumed. If execution is
unknown, inspect the robot and files before recording a reconciliation note:

```sh
openrua session --name shared resolve_unknown MESSAGE_ID "What was checked and observed"
```

A lost native process is not automatically restarted. Reconciliation alone does
not reconnect it or replay the task. The current CLI supports reconnecting clients
to a live service; recovery of a crashed execution host is not implemented.

To end the session and retain its workspace, native profile and event journal:

```sh
openrua session --name shared end
# Equivalent for a managed session:
openrua down --name shared
# Read retained records even after the service exits:
openrua session --name shared status
openrua session --name shared events
# Explicit deletion, after ending:
openrua clean --name shared
```

Ctrl-C in **serve** also requests resource shutdown. A shutdown failure leaves
the service available for inspection and retry. `clean` refuses a directory
with a service endpoint, including an endpoint left by a crashed host. Inspect
remaining resources before removing such a stale endpoint manually.

`up` / `agent` / `run` retain their original native-terminal mode. Opening a
second raw agent inside a managed session is refused because it would bypass
the shared queue. A native terminal attached to the *same managed conversation*
is not yet supported; the structured chat is the shared-input path today.

## Local API and access

The service binds only to `127.0.0.1`, on a chosen free port by default. Its
address and random bearer token are stored in the session's `endpoint.json`
with mode `0600`. The client reads that file; tokens are not placed in URLs or
printed in startup messages. Native records and the SQLite journal are private
session data. Do not publish them as application assets.

All requests require `Authorization: Bearer TOKEN`. The server checks the Host
and browser Origin, sends no CORS allowance, and rejects oversized bodies.
There is no remote/public hosting option or built-in TLS in this initial local
transport. The API is independent of agent vendors:

| Request | Result |
|---|---|
| `GET /api/session` | State snapshot and matching event cursor |
| `GET /api/events?after=N&limit=1000` | Ordered retained events after N |
| `POST /api/commands` | `{ "operation": "enqueue", "params": { ... } }` and other session operations |
| `POST /api/end` with `{}` | Stop owned resources before acknowledging closure |

A response timeout does not cancel an accepted command. Clients inspect the
journal or use the original request identity to resolve uncertain acceptance.
Storage, native transport and the HTTP front end each consume narrow contracts;
none adds a task-specific robot action API.

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
Local HTTP tests cover concurrent clients, a connection dropped before its
response, request timeouts, cursor replay, request validation and explicit end.
A CLI integration test exercises serve/chat/down through actual local HTTP with
a controlled native transport. The network tests use a controlled native transport. Separate live Codex
checks verified consecutive turns, a file task, native-thread resume and active
interruption followed by explicit queue resume. Robot execution through the new
service and successful Claude model turns remain unverified. See [agents.md](agents.md#structured-conversations-experimental)
for the native plugin protocol and handshake checks.
