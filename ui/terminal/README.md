# Pi terminal client

A keyboard-first OpenRUA client built with
[`@earendil-works/pi-tui`](https://github.com/earendil-works/pi/tree/main/packages/tui).
It reuses Pi's main-screen renderer, multiline editor, slash-command completion,
Markdown, and keyboard selectors. The existing OpenRUA service owns execution,
the native coding agent, robot resources, queue, and durable history. No Pi
agent runtime or model provider is imported.

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

The setup screen offers the robot, simulator, benchmark, agent and model
combinations with recorded startup evidence
([setup validation](../../docs/setup-validation.md)), and three actions:
**Save and check preparation** reports what is missing and builds nothing;
**Prepare and start** sets up Docker and the agent's sign-in when they are
missing, builds missing images with progress and retained logs, then starts the
background service once the checks pass; **Connect a real robot** opens
`openrua connect`. CLI configuration and direct edits to `config.yaml` remain
supported.

```sh
openrua --setup
openrua --resume
openrua --resume ID
```

New sessions receive automatic IDs; `openrua --resume` lists retained
conversations, running ones first. History selection reconnects to the original
execution owner; ended or unavailable conversations open read-only, without
restarting a robot or replaying instructions.

## Keyboard interaction

| Input | Result |
| --- | --- |
| Enter | Send; while the agent works, the message joins its running turn |
| Ctrl+J | Insert a newline |
| `/` | Complete OpenRUA's commands and the agent's own |
| Esc or `/interrupt` | Stop the current turn and cancel the robot's motions; queued messages go next |
| Ctrl+D on an empty input, or `/quit` | Leave: Enter keeps the session running, `e` ends it |
| Ctrl+C | Clear a draft; leave as Ctrl+D does if the input is empty |

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
