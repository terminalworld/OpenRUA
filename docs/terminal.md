---
summary: Keyboard-only terminal chat, configuration, history, and saved robot observations
read_when:
  - You want to chat while the agent works and manage queued instructions
  - You want keyboard controls for the default terminal
---

# Terminal chat

OpenRUA uses Pi's main-screen renderer, editor, Markdown and keyboard
selectors. There are no mouse-driven forms or buttons. The terminal consumes
the same session API as the browser and plain CLI; the existing service owns
the native coding agent, robot, message queue and history.

## Start and configure

```sh
pip install -U openrua
openrua
```

The runtime is included. No separate Node/npm install or frontend flag is needed.
Use arrows and Enter to choose robot, simulator, benchmark, agent and model;
Escape returns to the previous menu. Only environment combinations with recorded
startup checks are offered, with dependent fields updated together. Custom
profiles remain available through explicit CLI options and config files.

For example, `openrua --robot ./my-robot.yaml --sim '' --bench ''` uses a real
robot profile with no inherited simulator or benchmark. A custom environment
selected by a file path, including a saved default, is validated and prepared
before opening chat directly; it is not replaced by a guided-menu selection.
Use the file or `openrua config set` to edit custom defaults. `--setup` remains
the selector for tested bundled environments. Custom benchmark paths are kept
as paths throughout startup, so their settings are not replaced by an internal
benchmark name.

**Authentication** defaults to **Native CLI login**, using the account already
selected in your coding agent. To use API billing explicitly, choose **API key
file** and enter the path to a private file containing the raw key. Do not paste
the key into the menu. API mode is offered only when the selected plugin supports
it. Each agent keeps its own saved choice; switching agents never borrows another
agent's key path. Switching back to native login clears the selected path without
deleting the key file. These defaults apply to new sessions, not running ones.
For an external plugin, choose **Coding agent → Enter a name or profile path**.
OpenRUA loads the manifest's shared-chat and authentication capabilities before
showing its options. An invalid path returns to setup with an error; selecting
an agent alone does not save defaults, start resources, or call a model. Unsaved
model and authentication edits are retained separately when switching agents.
Experimental plugins still require the installation and validation steps in
their own guides and are not added to the default agent list.
See [API configuration](install.md#explicit-api-authentication) for CLI commands
and file permissions. Local checks do not call a model or verify account quota.

**Prepare and start** saves a valid selection, builds missing images with logs,
checks readiness and opens chat. Existing images are reused. **Save and check**
only checks preparation. Logs remain in `~/.openrua/preparation/` for retry.
See [setup validation](setup-validation.md) and [native login discovery](install.md#logging-an-agent-in).

Use `openrua --setup`, `openrua config set`, or `config.yaml` to change the same
shared defaults. Each ordinary launch creates a new ID. `/resume` in chat,
the setup menu's resume entry, or `openrua --resume` searches history. A running
session reconnects to its owner; ended or unavailable sessions open read-only
without restarting resources or replaying actions. `--resume ID` selects one
directly. Explicit `--name NAME` still starts or connects to that named session.

`openrua chat --tui --name NAME` also opens this keyboard interface for an
existing service. `--gui` and `--cli` select the browser and plain text client;
`openrua run` retains the native coding agent terminal. There is no Textual
fallback. Startup failures keep their logs in `~/.openrua/launches/NAME/`.

## Work in the conversation

| Key or command | Action |
| --- | --- |
| Enter / Ctrl+J | Send / insert newline |
| `/tools` | Expand or collapse a tool result |
| `/files` | Browse saved workspace files |
| `/queue` | Inspect, edit or withdraw queued instructions |
| `/questions` | Answer ordinary agent questions |
| Escape or `/interrupt` | Confirm interruption and pause the queue |
| `/continue` | Confirm continuing the paused queue |
| `/retry` | Retry an unconfirmed send with its original request ID |
| `/resume` | Search retained conversations |
| `/end` | Confirm ending execution, retaining history and workspace |
| Ctrl+D on empty input or `/quit` | Detach while execution continues |

Menus use arrows, Enter and Escape. Confirmation starts on Cancel. Typing
while the agent works does not interrupt it; submitted instructions queue.
A failed or unknown execution outcome pauses the queue for review. Client
reconnection replays events without automatically resending instructions.

For `/questions`, single choices use Enter. Multiple-choice requests use
Space or Enter to toggle selections, followed by **Continue**. When the native
agent allows a custom answer, **Write another answer** opens a text editor.
Escape cancels without sending; a final confirmation sends the collected
answers. Replies answer the pending question rather than enqueueing a new task.

## Current scope

`/files` uses the existing read-only workspace API. Text previews scroll;
images use the terminal's supported image protocol with a metadata fallback.
Escape returns, `r` refreshes, and the chat draft is preserved. Saved images
are observations, not a live video stream. Ended or unavailable conversations also support `/files` from history, using
the recorded workspace on disk. This starts only a temporary local read-only
viewer, never the robot or agent. Missing files are reported without losing
access to the transcript. The browser also provides image viewing and downloads.

Secret-input questions and reconciliation of unknown execution outcomes use
the [session CLI](sessions.md), so these operations also require no mouse.
In-session agent/model switching and persistent multi-panel layouts remain
unimplemented. Linux x86_64 installation and real PTY startup are tested;
terminal-specific image support and physical IMEs depend on the terminal.

## Development

See the [Pi frontend guide](../ui/terminal/README.md) for component boundaries,
keyboard tests over the real HTTP service, asset preparation and distribution
checks. The launcher receives configuration and history operations as callbacks;
it does not import execution internals. Tests use a controlled agent transport,
not paid models or physical robots.
