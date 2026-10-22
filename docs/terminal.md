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

The runtime is included. No separate Node/npm install or frontend flag is needed
(`--tui pi` remains an explicit alias for this terminal).
Use arrows and Enter to choose robot, simulator, benchmark, agent and model;
Escape returns to the previous menu. Only environment combinations with recorded
startup checks are offered, with dependent fields updated together. Custom
profiles remain available through explicit CLI options and config files.
**Connect a real robot** opens `openrua connect` for the robot folder
`~/.openrua/robots/my-robot/` ([connect.md](connect.md)).

Before a session starts, `openrua` offers a newer agent CLI at most once a
day: update now (rebuilds the sandbox image), skip, or skip until the next
version. A CLI older than OpenRUA can run must be updated to start
([update or remove](install.md#update-or-remove)).

For example, `openrua --robot ./my-robot.yaml` uses a real robot profile; a
real robot takes no saved simulator or benchmark default, only one named on the
command line. A custom environment
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

**Prepare and start** saves a valid selection, sets up Docker when it is
missing or not usable (the commands are shown and run only on a yes; see
[Docker](install.md#docker)), builds missing images with logs, offers the
agent's sign-in when it is not signed in, checks readiness and opens chat.
**Connect a real robot** sets up the same two first. Existing images are
reused. **Save and check preparation** only checks preparation. Logs remain in
`~/.openrua/preparation/` for retry. See [setup validation](setup-validation.md)
and [your agent's login](install.md#your-agents-login).

Use `openrua --setup`, `openrua config set`, or `config.yaml` to change the same
shared defaults. Each ordinary launch creates a new ID in the current project
directory; the first use asks whether you trust it. `--workspace DIR` selects
another project. `/resume` in chat, the setup menu's resume entry, or
`openrua --resume` searches this project's history, sessions still running
first. `/resume --all`, `--resume --all`, or Tab in the picker shows every
project. A running session reconnects to its owner; a confirmed ended project
conversation reconnects to fresh robot resources without replaying old actions
or queued instructions. Older or uncertain sessions open read-only.
See [robot projects](projects.md). `--resume ID` selects one
directly. Explicit `--name NAME` still starts or connects to that named session.

`openrua chat --tui --name NAME` also opens this keyboard interface for an
existing service. `--gui` and `--cli` select the browser and plain text client;
`openrua up` and `openrua agent` open the native coding agent terminal. There is no Textual
fallback. Startup failures keep their logs in `~/.openrua/launches/NAME/`.

## Work in the conversation

A conversation opens with a welcome: which robot you are on, simulated or
real, which agent is at the controls, and a few things to try. It is
OpenRUA's own greeting; the agent has not looked at anything yet.

| Key or command | Action |
| --- | --- |
| Enter / Ctrl+J | Send / insert newline |
| `/tools` | Expand or collapse a tool call, with its details and output |
| `/files` | Browse saved workspace files |
| `/watch` | See the robot's cameras live: the session's browser address and key to open it with (a terminal shows no video) |
| `/queue` | Inspect, edit or withdraw queued instructions |
| `/questions` | Choose answers to agent questions (or type the answer) |
| Escape or `/interrupt` | Stop the current turn and cancel the robot's motions; queued instructions go next |
| `/continue` | Confirm continuing a paused queue |
| `/permissions` | Whether the agent asks before each command that changes something, or runs without asking (`--permissions ask\|auto` sets how a session starts: `auto` by default, `ask` under `openrua connect`) |
| `/model` | Show the model, or change it: `/model <name>` |
| The agent's own commands | `/compact`, `/clear`, and what else the agent lists (Claude Code's skills and custom commands) go to the agent, as in its own terminal; `/help` lists them |
| `/retry` | Retry an unconfirmed send with its original request ID |
| `/resume` | Resume this project's conversations; `--all` searches all projects |
| `/end` | Confirm ending execution, retaining history and workspace |
| Ctrl+D on empty input or `/quit` | Leave, then choose: Enter keeps the session (and its robot) running in the background, `e` ends it, keeping its files |

Menus use arrows, Enter and Escape. Confirmation starts on Cancel. The terminal
behaves the way a coding agent's own does: what you send while the agent works
joins the running turn after its current tool calls (Claude Code and Codex
take it; with an agent that cannot, it waits for the turn to end), Escape
stops the turn at once and anything still queued goes next, and while the
agent waits on a question, what you type is the answer. A turn the agent
starts by itself (a background task finished) is shown as its own. The status
line says how long the agent has been quiet once it passes a minute.
A failed turn, a lost agent, a request this client cannot answer or a stopped
robot pauses the queue for review, and the status line says which. Client
reconnection replays events without automatically resending instructions.
If a running client's connection drops, the status shows **Disconnected · retrying**
until polling succeeds again. The recovered status reflects the shared session;
your unsent draft and queued instructions remain intact. Connection recovery does
not retry a send whose result is unconfirmed; use `/retry` with its original ID.

For `/questions`, single choices use Enter. Multiple-choice requests use
Space or Enter to toggle selections, followed by **Continue**. When the native
agent allows a custom answer, **Write another answer** opens a text editor.
Escape cancels without sending; a final confirmation sends the collected
answers. Replies answer the pending question rather than enqueueing a new task.

## Current scope

`/files` uses the existing read-only workspace API. Text previews scroll;
images use the terminal's supported image protocol (kitty, Ghostty, WezTerm,
iTerm2; over SSH too for iTerm2 and kitty), and every file also shows its
path on this computer: in Cursor or VS Code, which draw no pictures in their
terminal, Ctrl+click opens it. A file the agent names under `/workspace` in a
reply gets the same path under its message.
Escape returns, `r` refreshes, and the chat draft is preserved. Saved images
are observations, not a live video stream; `/watch` gives the cameras live, in
the browser. Ended or unavailable conversations also support `/files` from history, using
the recorded workspace on disk. This starts only a temporary local read-only
viewer, never the robot or agent. Missing files are reported without losing
access to the transcript. The browser also provides image viewing and downloads.

Secret-input questions and reconciliation of unknown execution outcomes use
the browser or the [session CLI](sessions.md), which needs no mouse.
Switching the agent inside a session and persistent multi-panel layouts remain
unimplemented; `/model` changes the model. Linux x86_64 installation and real PTY startup are tested;
terminal-specific image support and physical IMEs depend on the terminal.

## Development

See the [Pi frontend guide](../ui/terminal/README.md) for component boundaries,
keyboard tests over the real HTTP service, asset preparation and distribution
checks. The launcher receives configuration and history operations as callbacks;
it does not import execution internals. Tests use a controlled agent transport,
not paid models or physical robots.
