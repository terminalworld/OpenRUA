---
summary: Try the chat-first terminal interface for a shared robot session
read_when:
  - You want to chat while the agent works and manage queued instructions
  - You are evaluating the experimental terminal interface
---

# Terminal chat (experimental)

The terminal interface attaches to the same session as the browser and plain
CLI. It uses Textual's editor, layout, dialogs, and collapsible widgets;
OpenRUA's existing service still owns the robot, native agent, and message queue.
It does not implement a new agent loop.

## Try it

Install OpenRUA 0.1.0 or later with the optional terminal interface:

```sh
pip install -U 'openrua[tui]>=0.1.0'
```

Prepare the robot images and native agent login as described in
[Your first shared robot session](../examples/shared-session.md), then start the
service in an execution terminal:

```sh
openrua serve panda --sim robosuite --name shared --agent codex
```

In a client terminal, open the TUI:

```sh
openrua chat --tui --name shared
```

This interface attaches to an existing service. It does not yet make bare
`openrua` launch a session or start a background service automatically.
`openrua run` retains the native agent terminal, and `openrua chat` without
`--tui` retains the plain text interface.

## Work in the conversation

- Type an instruction and press **Ctrl+S**, or select **Send**. Enter inserts a
  new line. You can keep typing while the agent works; subsequent instructions
  join the shared queue.
- Tool output starts collapsed. Select its title to inspect it.
- **Ctrl+P** or **Queue** toggles a panel that stays open until you hide it.
  It appears beside the conversation in wide terminals and below it in narrow
  terminals. Edit or withdraw queued instructions there.
- **Interrupt** requests interruption of the current turn and pauses the queue.
  Review retained instructions before choosing **Resume** and confirming.
- **Answer** opens pending questions from the agent. Answers go to that question,
  not into the task queue. Secret inputs are masked and their answers are not
  added to the event journal.
- **Ctrl+Q** detaches. The session continues in its execution terminal.
  **End session** asks for confirmation before stopping resources, retaining
  records and the workspace.

After a connection failure, the interface reconnects and replays missing events.
If sending an instruction has an unknown result, its draft is locked until the
acceptance is observed or **Retry send** succeeds. Retries reuse the same request
identity to prevent duplicate execution. If you close the TUI before this is
resolved, inspect `openrua session --name shared status` before sending it again.

An unknown execution result keeps the queue paused. Reconcile it through the
[session CLI or browser](sessions.md); the TUI displays this state but does not
yet provide its reconciliation editor. A disconnected native agent is different
from a disconnected client: reopening the TUI does not restart the agent.

## Current scope

Agent and model selection still happens when starting `serve`. Changing them
inside a running conversation, a workspace file panel, terminal image previews,
and saved layout preferences are not implemented in this release. The browser
can display saved workspace images. Use a terminal at least 60 columns wide and
28 rows high for the tested narrow layout.

Automated headless Textual tests use the real HTTP service and SQLite journal
with a controlled agent transport. They exercise shared queues, stale edits,
streamed output, interrupted turns, question replies, unknown outcomes,
reconnection, lost acknowledgements, and confirmed shutdown. They do not invoke
a paid model or a physical robot. Multiline Unicode content is tested; physical
terminal IME behavior remains to be checked on users' terminal applications.

## Development

```sh
pip install -e '.[dev,tui]'
pytest -q tests/tui
lint-imports
```

`openrua/tui/` depends on `openrua.sessions.client` only within OpenRUA. It has no
imports from robot backends, agent plugins, or the session execution internals.
CLI code selects and launches this presentation module; replacing its UI toolkit
does not require changing agent plugins or the shared session protocol.
