# Pi terminal client

A keyboard-first OpenRUA client built with
[`@earendil-works/pi-tui`](https://github.com/earendil-works/pi/tree/main/packages/tui).
It reuses Pi's main-screen renderer, multiline editor, slash-command completion,
Markdown, and keyboard selectors. The existing OpenRUA service owns execution,
the native coding agent, robot resources, queue, and durable history. No Pi
agent runtime or model provider is imported.

## Start

Pi is an opt-in frontend in OpenRUA 0.5.0; the default remains Textual:

```sh
pip install -U 'openrua[pi]>=0.5.0'
openrua --tui pi
```

The wheel includes the frontend and its dependency resources. The `pi` extra
installs a packaged Node runtime through pip, so users need neither a separate
Node/npm install nor a second terminal. This distribution path has been tested
on Linux x86_64. Other runtime platforms and terminal IMEs need further trials.

The keyboard setup shows the same robot, simulator, benchmark, agent, and model
configuration as the existing clients. Use arrows and Enter to edit fields or
choose available profiles. **Save and check** reports missing images or login;
**Prepare and start** builds missing images with progress and retained logs,
then starts the existing background service after checks pass. Environment
selectors are linked and offer only combinations with recorded startup evidence;
see [setup validation](../../docs/setup-validation.md).
Save and check does not build. Agent login still uses the native CLI. CLI configuration
and direct edits to `config.yaml` remain supported.

```sh
openrua --tui pi --setup
openrua --tui pi --resume
openrua --tui pi --resume ID
```

You can also reconnect to a named running session using `--name NAME`. New
sessions receive automatic IDs. History selection reconnects to the original
execution owner; ended or unavailable conversations open read-only, without
restarting a robot or replaying instructions.

## Keyboard interaction

| Input | Result |
| --- | --- |
| Enter | Send through the shared queue |
| Ctrl+J | Insert a newline |
| `/` | Complete available commands |
| `/files` | Browse saved workspace files and images |
| `/tools` | Select a tool result to expand or collapse |
| `/queue` | Select a queued instruction to inspect, edit, or withdraw |
| `/questions` | Answer pending agent questions, then confirm submission |
| `/resume` | Find another retained conversation |
| Esc or `/interrupt` | Confirm interruption of the current turn |
| `/continue` | Confirm continuation of the paused queue |
| `/retry` | Retry an unconfirmed send using its original request ID |
| `/end` | Confirm ending execution while keeping history and workspace |
| Ctrl+D on an empty input, or `/quit` | Detach without ending the session |
| Ctrl+C | Clear a draft; detach if the input is empty |

Selection lists use arrows, Enter, and Escape. Interruption, queue continuation,
withdrawal, answering questions, and ending execution require explicit choices.
Edits carry the message revision, so another client's change is not overwritten.
A disconnected client never automatically resends a robot instruction.

Requests containing secret inputs and resolution of unknown execution results
still use the existing clients. In-session agent/model switching and a persistent
multi-panel layout are not implemented. Automated tests do not establish
real-terminal IME, SSH, clipboard, or long-transcript usability.

## Saved workspace files

Use `/files` to open a keyboard directory selector. Enter opens a directory or
file; Escape returns to the directory and then to chat, preserving the editor
draft. Select Refresh directory or press `r` in a preview to fetch the latest
saved content. Text previews scroll with arrows, Page Up/Down, Home and End.
Pi displays images using the terminal's supported image protocol; unsupported
formats or terminals show metadata instead. The existing browser can display
and download saved images. These are saved observations, not a live video feed.

This is read-only access through the same workspace API as the browser. It does
not send an agent message or execute a file. Symlinks, paths outside the workspace,
and files exceeding the server limit are rejected; changed or deleted files
report an error and can be refreshed. Text previews show at most 131,072 characters. Retained offline history has no workspace API; its transcript remains
available without restarting execution.

## Develop and verify

From the repository root, prepare the existing Python development environment:

```sh
. .venv/bin/activate
pip install -e '.[dev,pi]' build
npm ci --prefix ui/terminal --ignore-scripts
npm test --prefix ui/terminal
python scripts/terminal_assets.py prepare
openrua --tui pi
```

Frontend developers need Node 22.19 or newer for npm. Re-run asset preparation
after changing frontend source, including before building simulator images from the checkout. No npm command or download runs when a user
starts OpenRUA. A source-only client can still attach to an already running
service with `npm start --prefix ui/terminal -- --endpoint PATH`.

```sh
python -m build
python scripts/check_dist.py --dist dist --version 0.5.0
```

Distribution builds check the asset manifest, source hashes, and locked
versions. The source archive carries prepared dependencies and can build its
wheel without npm. Upstream resources and license files remain intact. The
clean-install check exercises both the default frontend and Pi, including
pseudo-terminal configuration startup and detachment with no Node on PATH.

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
