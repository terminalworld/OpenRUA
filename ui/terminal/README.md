# Pi terminal client

A keyboard-first OpenRUA client built with
[`@earendil-works/pi-tui`](https://github.com/earendil-works/pi/tree/main/packages/tui).
It reuses Pi's fullscreen renderer, multiline editor, slash-command completion,
Markdown, and keyboard selectors. The existing OpenRUA service owns execution,
the native coding agent, robot resources, queue, and durable history. No Pi
agent runtime or model provider is imported.

The host owns its color theme. Components use semantic color roles, and the
renderer resolves them using Pi's color capabilities and terminal appearance
reports. This also recolors cached text when the terminal changes appearance.
The overall terminal background is inherited; welcome, editor and selected
items have local surfaces. Unknown appearance uses the terminal palette, and
`NO_COLOR` removes color without changing keyboard behavior or cursor control.

## Start

The terminal comes with the package; all interaction uses the keyboard:

```sh
pip install -U openrua
openrua
```

The wheel includes the frontend, its dependency resources and a packaged Node
runtime, so users need neither a separate Node/npm install nor a second
terminal. This distribution path has been tested on Linux x86_64. Other runtime
platforms and terminal IMEs need further trials.

Setup first separates **Real robot** from **Simulation**. Real robots use a
project configuration or start a new connection; simulation separates native
scenes and benchmark environments. The next step explicitly chooses Claude Code,
Codex, Kimi Code, ZCode or another manifest before the final review. Authentication belongs under Agent settings; session naming
and preparation checks belong under Advanced options. The bundled starting
combinations have [startup evidence](../../docs/setup-validation.md).

**Save project and start** saves the selection, prepares missing images, checks
readiness and starts the existing background service. Preparation and failures
stay in the same interface. Native login and sudo borrow the terminal only when
needed. Project configuration lives in `.openrua/settings.json`.

```sh
openrua --setup
openrua --resume
openrua --resume ID
```

New sessions receive automatic IDs in the current project directory, whose
first use asks for trust. `openrua --resume` lists this project's retained
conversations, running ones first; `--all` or Tab in the picker shows every
project. Live conversations reconnect to their owner. Confirmed ended project
conversations reconnect to fresh robot resources with their native identity;
past actions and queued instructions are not replayed. Older or uncertain
records remain read-only. See [robot projects](../../docs/projects.md).

## Keyboard interaction

| Input | Result |
| --- | --- |
| Enter | Send; while the agent works, the message joins its running turn |
| Ctrl+J | Insert a newline |
| `/` | Complete OpenRUA's commands and the agent's own |
| Esc or `/interrupt` | Stop the current turn and cancel the robot's motions; queued messages go next |
| Ctrl+D twice on an empty input, or `/quit` | Choose whether to keep execution resources; No is selected by default, Yes keeps them running, Back or Esc returns |
| Ctrl+C | Interrupt a response; when idle, clear input and press again to leave |

`/help` lists every other command (`/files`, `/watch`, `/tools`, `/queue`,
`/questions`, `/permissions`, `/model`, `/continue`, `/retry`, `/resume`,
`/end`) and the agent's own; the [terminal guide](../../docs/terminal.md)
describes them. The terminal and the browser take their commands from one
list, `openrua/web/session.mjs`.

Selection lists use arrows, Enter, and Escape. Queue continuation, withdrawal,
answering questions, and ending execution ask for confirmation. Edits carry the
message revision, so another client's change is not overwritten. A disconnected
client never automatically resends a robot instruction. For multiple-choice
questions and optional text replies, see the
[question controls](../../docs/terminal.md#work-in-the-conversation).

Requests containing secret inputs and resolution of unknown execution results
use the browser or the session CLI. Switching the agent inside a conversation
and a persistent multi-panel layout are not implemented. Automated tests do not
establish real-terminal IME, SSH, clipboard, or long-transcript usability.

## Saved workspace files

Use `/files` to open a keyboard directory selector. Enter opens a directory or
file; Escape returns to the directory and then to chat, preserving the editor
draft. Select Refresh directory or press `r` in a preview to fetch the latest
saved content. Text previews scroll with arrows, Page Up/Down, Home and End.
Pi displays images using the terminal's supported image protocol; unsupported
formats or terminals show metadata instead. The existing browser can display
and download saved images. These are saved observations; `/watch` gives the
cameras live, in the browser.

This is read-only access through the same workspace API as the browser. It does
not send an agent message or execute a file. Symlinks, paths outside the workspace,
and files exceeding the server limit are rejected; changed or deleted files
report an error and can be refreshed. Text previews show at most 131,072 characters.
For ended or unavailable sessions, the launcher opens a temporary read-only
workspace viewer using the recorded directory. This does not restart the robot
or agent; missing files leave the transcript available. See the
[terminal guide](../../docs/terminal.md#current-scope) for supported preview formats.

## Develop and verify

From the repository root, prepare the existing Python development environment:

```sh
. .venv/bin/activate
pip install -e '.[dev]' build
npm ci --prefix ui/terminal --ignore-scripts
npm test --prefix ui/terminal
python scripts/terminal_assets.py prepare
openrua
```

Frontend developers need Node 22.19 or newer for npm. Re-run asset preparation
after changing frontend source, including before building simulator images from the checkout. No npm command or download runs when a user
starts OpenRUA. A source-only client can still attach to an already running
service with `npm start --prefix ui/terminal -- --endpoint PATH`.

```sh
python -m build
python scripts/check_dist.py --dist dist --version "$(python -c 'from openrua import __version__; print(__version__)')"
```

Distribution builds check the asset manifest, source hashes, and locked
versions. The source archive carries prepared dependencies and can build its
wheel without npm. Upstream resources and license files remain intact. The
clean-install check exercises the terminal, including pseudo-terminal
configuration startup and leaving it, with no Node on PATH.

Session tests use the actual local HTTP service and SQLite store, with
controlled native-agent events. They cover shared input, event replay, tool
expansion, keyboard configuration and history, queue edits, agent questions,
interruption, detachment, and idempotent retries. They use no model credentials,
paid calls, simulator, or robot.

## Boundaries and upstream

- `src/client.mjs`: existing authenticated local HTTP API only.
- `src/workspace.mjs`: read-only saved-file browsing with upstream Pi components.
- `src/controller.mjs`: event cursor and unconfirmed request identity.
- `src/view.mjs`: event presentation using Pi components.
- `src/app.mjs`, `src/screens.mjs`: keyboard chat, configuration, and history.
- `openrua/terminal/launcher.py`: private input/output handoff and injected callbacks.
- `openrua/cli/commands/start.py`: connects these callbacks to existing product operations.

The launcher passes temporary private files to the child UI, keeping stdin and
stdout available for the terminal. Credentials are not placed in command-line
arguments. It uses a Python-packaged Node runtime, not a guessed system path.
History and setup return explicit choices; the CLI performs the existing
operations. No extra HTTP server or replacement execution loop is introduced.

Pi is pinned to 1.0.2 in `package-lock.json`. Its
[chat-simple example](https://github.com/earendil-works/pi/blob/main/packages/tui/examples/chat-simple.ts)
and interactive UI informed the component composition. Pi and its dependencies
are MIT-licensed; their license files are included in the prepared resources.
The Node runtime is supplied by
[nodejs-wheel-binaries](https://github.com/njzjz/nodejs-wheel), under its own notices.

The Python application owns preparation and lifecycle operations. One Node host
renders the successive pages over a private inherited control socket. Screen
specifications use private temporary files; credentials never appear in command
arguments. The host restores the prior screen without replaying the transcript,
then prints a brief session-status message.

Startup software updates use the same host as setup and chat. Runtime dependency
checks and target CLI installation stay in plugins; the update page consumes
plain offer/result data. Compact progress retains raw output privately, with L
toggling live details and result-page log inspection. Retry is an explicit action.
