---
summary: Keyboard-only terminal chat, configuration, history, and saved robot observations
read_when:
  - You want to chat while the agent works and manage queued instructions
  - You want keyboard controls for the default terminal
---

# Terminal chat

OpenRUA uses Pi's fullscreen renderer, editor, Markdown and keyboard
selectors. A project header, scrollable conversation and fixed input area share
one interface from setup through preparation and chat. Leaving restores the
previous terminal screen. The terminal consumes
the same session API as the browser; the existing service owns
the native coding agent, robot, message queue and history.

OpenRUA keeps your terminal's background and uses green accents for its welcome
screen and selected items. Where the terminal reports its appearance, local
surfaces and text adapt to light or dark colors. Other terminals use their own
color palette. Set `NO_COLOR=1` to keep the interface without color; labels and
symbols still distinguish status and selection.

## Start and configure

```sh
pip install -U openrua
openrua
```

The runtime is included. No separate Node/npm install or frontend flag is needed
(`--tui pi` remains an explicit alias for this terminal).
First choose **Real robot** or **Simulation**. Real robots can use a project
configuration or **Connect a new robot**. Simulation offers **Simulation scene**
and **Benchmark environment**, followed by a supported starting environment
or a custom configuration path. Selecting a benchmark here opens an interactive
task, not batch evaluation. Next, **Choose your coding agent** lists Claude Code,
Codex, Kimi Code and ZCode, plus **Other agent or configuration file…**. Select an agent,
review its model and authentication, then choose **Continue to review**.
**Advanced options** holds the session name and preparation check.
Arrows and Enter select, number keys choose compatible menu entries, and Escape
cancels the current view without applying a selection. Confirmations require Enter.
Maintained files remain editable in the project.
**Connect a new robot** opens `openrua connect` inside
the current coding project ([connect.md](connect.md)).

Before a session starts, `openrua` offers available software updates in the same
fullscreen interface. Optional updates default to **Continue with current
version**; **Skip this version** remembers that release. A CLI below the plugin's
supported minimum requires an update or exit. During an update, the target,
stage and elapsed time stay visible; **L** shows or hides live installer logs.
The result offers a retained log, continuation or explicit retry. Updating an
agent retains compatible runtime dependencies and other image contents, then
verifies the selected CLI before new sessions use it. Updating OpenRUA itself
uses its installer and asks you to restart the command after success
([update or remove](install.md#update-or-remove)).

For example, `openrua --robot ./my-robot.yaml` uses a real robot profile; a
real robot takes no saved simulator or benchmark default, only one named on the
command line. A custom environment
selected by a file path, including a saved default, is validated and prepared
before opening chat directly; it is not replaced by a guided-menu selection.
Use the file or `openrua config set` to edit custom defaults. `--setup` edits the current project or selects another starting environment. Custom benchmark paths are kept
as paths throughout startup, so their settings are not replaced by an internal
benchmark name.

**Authentication** defaults to the selected agent's **account**, using the account already
selected in your coding agent. To use API billing explicitly, choose **API key
file** and enter the path to a private file containing the raw key. Do not paste
the key into the menu. API mode is offered only when the selected plugin supports
it. Each agent keeps its own saved choice; switching agents never borrows another
agent's key path. Switching back to native login clears the selected path without
deleting the key file. These defaults apply to new sessions, not running ones.
For an external plugin, choose **Other agent or configuration file…** in the agent list.
OpenRUA loads the manifest's shared-chat and authentication capabilities before
showing its options. An invalid path returns to setup with an error; selecting
an agent alone does not save defaults, start resources, or call a model. Unsaved
model and authentication edits are retained separately when switching agents.
Experimental plugins still require the installation and validation steps in
their own guides and are not added to the default agent list.
See [API configuration](install.md#explicit-api-authentication) for CLI commands
and file permissions. Local checks do not call a model or verify account quota.

**Save project and start** saves a valid selection, sets up Docker when it is
missing or not usable (the commands are shown and run only on a yes; see
[Docker](install.md#docker)), builds missing images with logs, offers the
agent's sign-in when it is not signed in, checks readiness and opens chat.
**Connect robot** sets up the same two first. Existing images are
reused. **Save project settings and check setup** only checks preparation. Logs remain in
`~/.openrua/preparation/` for retry. See [setup validation](setup-validation.md)
and [your agent's login](install.md#your-agents-login).
Preparation logs appear within the same interface; failures return to setup for
editing and retry. Native sign-in and sudo prompts temporarily use the ordinary
terminal, then return to OpenRUA.

Use `/setup` in chat to review project settings and start a fresh conversation
within the same terminal. Before switching, choose whether to keep the previous
conversation's execution resources. No is selected by default and stops owned
resources; Yes leaves them running in the background. Files and history remain
available through `/resume` either way. Cancelling setup before confirming
departure returns to the current conversation without stopping it. The new conversation uses the selected
environment and agent with its own context. You can also use `openrua --setup`,
`openrua config set`, or `.openrua/settings.json`
to change project settings. Each ordinary launch creates a new ID in the current project
directory; the first use asks whether you trust it. `--workspace DIR` selects
another project. `/resume` in chat, the setup menu's resume entry, or
`openrua --resume` searches this project's history, sessions still running
first. `/resume --all`, `--resume --all`, or Tab in the picker shows every
project. A running session reconnects to its owner; a confirmed ended project
conversation reconnects to fresh robot resources without replaying old actions
or queued instructions. Older or uncertain sessions open read-only.
See [robot projects](projects.md). `--resume ID` selects one
directly. Explicit `--name NAME` still starts or connects to that named session.

`openrua --name NAME` also opens this keyboard interface for an
existing service. `--gui` selects the browser;
`openrua up` and `openrua agent` open the native coding agent terminal. There is no Textual
fallback. Startup failures keep their logs in `~/.openrua/launches/NAME/`.

## Work in the conversation

A conversation opens with an OpenRUA welcome card showing the project,
environment and coding agent. It stays visible before your first message,
then collapses to a compact project heading when you send. Connection status
appears separately below; the welcome does not imply the agent has inspected
the robot.

| Key or command | Action |
| --- | --- |
| Enter / Ctrl+J | Send / insert newline |
| `/tools` | Expand or collapse a tool call, with its details and output |
| `/files` | Browse saved workspace files |
| `/project` | Browse skills and reports, start or cancel a review, manage periodic plans, and inspect candidates |
| `/watch` | See the robot's cameras live: the session's browser address and key to open it with (a terminal shows no video) |
| `/robot` | Inspect connection status/logs, build images, or update the connection in this conversation |
| `/queue` | Inspect, edit, withdraw or explicitly send held instructions |
| `/questions` | Choose answers to agent questions (or type the answer) |
| Escape or `/interrupt` | Stop the current turn and cancel the robot's motions; queued instructions go next |
| `/continue` | Confirm continuing a paused queue |
| `/permissions` | Choose the current agent's native permission mode; `--permissions MODE` selects a startup mode (the selected plugin supplies its default) |
| `/model` | Open native model choices or enter a model ID; `/model <name>` changes the next-turn setting directly |
| The agent's own commands | `/compact`, `/clear`, and what else the agent lists (Claude Code's skills and custom commands) go to the agent, as in its own terminal; `/help` lists them |
| `/retry` | Retry an unconfirmed send with its original request ID |
| `/setup` | Configure and open a new conversation, retaining the current session in history; cancel to return |
| `/resume` | Resume this project's conversations; `--all` searches all projects |
| `/end` | Confirm ending execution, retaining history and workspace |
| Ctrl+D twice on empty input or `/quit` | Choose whether to keep execution resources; No is selected by default, Yes keeps them running, Back or Escape cancels leaving |
| Ctrl+C | Interrupt a running response and keep the draft; when idle, clear input and show a hint, then press again to open the leave choice |

The keyboard follows Claude Code's interrupt and input priorities. A running
response receives Ctrl+C before the input is cleared. The existing No/Yes leave
choice follows the double press, so you can still return without stopping anything.
Press the same shortcut again within 800 ms; typing or opening a menu cancels
the shortcut hint. Back and Escape on the leave page return without stopping
the session. `/quit` opens that page directly.
Key releases and held-key repeats do not count as a second exit press.
Ctrl+C and Ctrl+D leave the resource choice open; use Back or Escape to return.
With text in the editor, Ctrl+D deletes the next character instead of leaving.

No stops the agent and frees the memory used by the session. Any simulator or
drivers OpenRUA started for it also stop. Your project files and chat history
are saved. Drivers you started outside OpenRUA keep running.

In `/files`, Up/Down selects an entry, Left returns to the parent directory,
and Right or Enter opens the selected directory or file. Returning to a parent
selects the directory you just left. Escape closes the directory browser;
in a file preview it returns to the same directory and selection. Reads keep
the browser focused while loading, and closing a view discards its late result.

Initial model, effort and permission synchronization updates the status without
a change notification. Your own setting changes receive one confirmation after
the agent accepts them. While a turn runs, the status distinguishes its settings
from a changed next-turn setting when the agent supports that distinction.
Escape in an idle conversation leaves the draft and feedback in place.

Ending shows progress until the owner confirms resource shutdown. A successful
`/end` does not ask you to end the session again. On ordinary exit the terminal
checks the current session state before asking whether to keep execution resources.
The same choice appears before switching to another conversation through history
or setup. Merely opening or cancelling either screen does not stop resources.
Selecting the current conversation or leaving read-only history needs no choice. If
the service cannot be reached, it reports an unconfirmed status and gives a
`session status` command instead of claiming the robot is still running or stopped.
Stopping owned resources does not guarantee physical motor power is off; that
depends on the real robot's driver. See [ending and reconnecting](sessions.md).

The status line shows the configured model even when the conversation scrolls.
`/model` uses the native agent's model choices when available and always offers
an explicit model-ID editor. For Codex, select a model and its supported
reasoning level, then confirm to apply both. Escape returns to model selection
without changing either setting. A new model starts with its native default
level. Codex changes apply to the next turn; other agents report their own
setting behavior. An accepted request remains pending until the native agent
confirms it; rejections retain the native reason. Viewing the menu calls no model.

OpenRUA selects the model independently of a host coding agent's default. Its
bundled Codex profile defaults to `gpt-5.6-sol`; an explicit model, project
selection or agent configuration can override it. For example,
`/model gpt-6-astra` sets that ID for the next turn. The model setting and native
responses are the evidence to inspect, rather than asking the model to guess its
identity. Native account availability is determined by the agent service.

For Codex, `/model effort` opens the reasoning effort choices advertised for
the selected model. `/model effort medium` sets a level directly. The setting
is visible alongside the model and retained when the conversation is resumed;
an active turn keeps its existing parameters. A direct model change also chooses
a supported native default level rather than retaining an incompatible one. Agents
that have not reported live effort control do not expose this picker.

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

If the native agent connection has ended, `/continue` cannot reconnect it. Read
the session's `agent.stderr.log` and service log first. Malformed native stdout
is retained in `agent.stderr.invalid-output.log`; this private file may contain
agent output, so inspect it locally before sharing. Blank lines alone do not
break a native JSON connection. Queued instructions are not automatically
replayed after a disconnection.

Pending questions and command approvals open automatically below the transcript,
with a highlighted **Your response is needed** heading. The status changes to
**Waiting for your response**, rather than continuing to show Working. Existing
draft text is kept. Escape hides the question; `/questions` reopens it, and the
waiting status remains until the request is answered. Another client's answer
closes an obsolete question dialog automatically.

For question dialogs, single choices use Enter. Multiple-choice requests use
Space or Enter to toggle selections, followed by **Continue**. When the native
agent allows a custom answer, **Write another answer** opens a text editor.
Escape cancels without sending; a final confirmation sends the collected
answers. Replies answer the pending question rather than enqueueing a new task.

Long questions and commands use a bounded text window. PgUp/PgDn reads all
details while the response heading and choices stay visible; scrolling never
sends an answer. The final response review shows the selected answer first and
uses the same scrollable details.

Follow-up messages use a compact pending area: **Queued** means waiting for a
turn, **Sending to agent** means dispatch is in progress, and **Adding to current
turn** means native delivery is awaiting confirmation. After the agent confirms
receipt, the message leaves this area and appears in conversation history, at
the turn it joined. Receipt does not mean the requested work has completed.

## Native permission modes

`/permissions` lists only the modes supplied by the selected native agent.
Claude Code uses its own identifiers: `default`, `auto`, `acceptEdits`, `plan`,
`dontAsk` and `bypassPermissions`. Its default is `auto`, which uses the native
action classifier. `bypassPermissions` explicitly skips permission checks.

Codex offers Ask for approval (`untrusted`), Approve for me (`auto_review`) and
Full access (`never`). It defaults to Approve for me, with `on-request` approvals,
the native automatic reviewer and a workspace-write sandbox. Full access uses
`never` approvals and danger-full-access. These are Codex policies; Claude's Plan
and Accept edits modes are not applied to Codex.

```sh
openrua --agent claude-code --permissions auto
openrua --agent codex --permissions auto_review
```

Kimi/ZCode previews use native approval questions and do not currently expose
switchable permission modes. Their menus do not invent another agent's choices.
An unsupported explicit mode is refused. Existing saved conversations retain
the meaning of their recorded modes, including older bypass identifiers.

Claude mode changes require a native acknowledgement. Codex changes apply to
the next turn; native automatic review must be confirmed by its app-server.
Native errors remain visible. No rejected mode is silently replaced by full
access. OpenRUA supplies the shared selection, confirmation and feedback UI;
the selected plugin owns the options and native behavior.

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

## Native model selection

Setup's Model page loads the selected CLI's own model catalog using its chosen
account or explicit API profile. Claude Code supplies models from initialization;
Codex supplies its paginated model list and reasoning levels. A temporary
metadata terminal submits no prompt and connects to no robot. If agent tools are
missing, their preparation stays in the compact fullscreen progress view.

Shared chat uses the native metadata on the running connection. Discovery errors
are shown with the current model, reload/custom entry remain explicit, and the
configured default is never presented as the complete native catalog. Model IDs
and choices remain scoped to the selected agent. The Kimi/ZCode previews keep the
model configured in their manifest and explain that limitation.

Setup preparation shows stages and elapsed time. Press L to inspect live details;
full build logs remain available at the reported path. Every project conversation
opens with a welcome card that collapses after you send a message. Saved projects reuse their
selection on ordinary startup; use `/setup` or `openrua --setup` to review it, or `--resume`
to resume a retained conversation.
