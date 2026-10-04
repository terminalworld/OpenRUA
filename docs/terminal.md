---
summary: Configure, start, and use the default terminal interface for shared robot sessions
read_when:
  - You want to chat while the agent works and manage queued instructions
  - You are evaluating the experimental terminal interface
---

# Terminal chat (experimental)

The terminal interface attaches to the same session as the browser and plain
CLI. It uses Textual's editor, layout, dialogs, and collapsible widgets;
OpenRUA's existing service still owns the robot, native agent, and message queue.
It does not implement a new agent loop.

## Start and configure

Install OpenRUA 0.3.0 or later, then run it:

```sh
pip install -U 'openrua>=0.3.0'
openrua
```

Without an explicit name, each launch prepares a new session with a unique ID.
A setup form collects the robot, simulator, optional benchmark, coding agent, model, and
an automatically generated session ID. You may supply a name, but do not need to. Use Tab to complete available names or enter a profile path.
**Save & check** saves shared defaults and reports missing images or login;
**Save & start** also starts the session once the checks pass. Neither button
automatically builds images or logs in. Check/start operations disable editing
until their result is known.

Run `openrua --setup` to reopen the form later. You can also use
`openrua config set` or edit `config.yaml` directly. The TUI stores no separate
copy of these defaults. Changing the selected agent resets the model field to
that agent's saved model, rather than carrying the previous agent's model over.

The background service continues after leaving the TUI. Use `/resume` in the
startup command field (Enter) or chat editor (Ctrl+S) to search history by its
automatic title or fixed ID. Titles come from the first user instruction without
a model call. `openrua --resume` opens the same selector directly;
`openrua --resume ID` selects a conversation explicitly. Running sessions
reconnect with their existing queue and native agent conversation. Ended or
unavailable sessions open read-only; the interface does not restart resources
or replay instructions. Switching conversations detaches from the current one.

Existing named sessions also appear in history. `openrua --name NAME` retains its
explicit start-or-connect behavior; only unnamed launches always create new IDs.
Use `openrua --gui --resume ID` or `openrua --cli --resume ID` for an existing live
conversation in another interface. Read-only history is currently in the TUI;
the web interface remains scoped to the selected live session. Startup failures appear in the form and are retained in
`~/.openrua/launches/NAME/service.log`. A startup timeout may leave an owner still
initializing; inspect that log and use the displayed ID with `--name` when ready
rather than starting another session or deleting its files.

The existing `openrua chat --tui --name NAME` attaches to a service explicitly.
For a foreground service with visible execution logs, use `openrua serve`.
`openrua run` continues to open the native agent terminal.

For a complete simulated example, follow
[Your first shared robot session](../examples/shared-session.md).

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
- **Ctrl+Q** detaches. The session continues in its service process.
  **End session** asks for confirmation before stopping resources, retaining
  records and the workspace.

After a connection failure, the interface reconnects and replays missing events.
If sending an instruction has an unknown result, its draft is locked until the
acceptance is observed or **Retry send** succeeds. Retries reuse the same request
identity to prevent duplicate execution. If you close the TUI before this is
resolved, inspect `openrua session --name chat-demo status` before sending it again.

An unknown execution result keeps the queue paused. Reconcile it through the
[session CLI or browser](sessions.md); the TUI displays this state but does not
yet provide its reconciliation editor. A disconnected native agent is different
from a disconnected client: reopening the TUI does not restart the agent.

## Current scope

Agent and model selection still happens when starting a session. Changing them
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

`openrua/tui/` depends on `openrua.sessions.client` only within OpenRUA.
The setup form receives explicit save, check, and launch callbacks from the
entry point; it does not import configuration or robot ownership modules. It has no
imports from robot backends, agent plugins, or the session execution internals.
CLI code selects and launches this presentation module; replacing its UI toolkit
does not require changing agent plugins or the shared session protocol.

## Pi client prototype (source checkout)

An opt-in [Pi terminal prototype](../ui/terminal/README.md) explores a
keyboard-first coding-agent interface on the same session service. It uses Pi's
editor, completion, Markdown, and keyboard selectors. See its README for setup,
commands, tests, and current limits. It does not replace the installed default.
