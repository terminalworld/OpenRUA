# Pi terminal prototype

A keyboard-first OpenRUA client built with
[`@earendil-works/pi-tui`](https://github.com/earendil-works/pi/tree/main/packages/tui).
It reuses Pi's main-screen renderer, multiline editor, slash-command completion,
Markdown, and keyboard selection lists. It connects to the existing OpenRUA
session API; the execution service continues to own the native agent, robot,
queue, and durable history. No Pi agent runtime or model provider is imported.

This source-only prototype does not change the default `openrua` interface and
is not included in the current PyPI wheel.

## Try it

Use Node 22.19 or newer. From the repository root:

```sh
npm ci --prefix ui/terminal --ignore-scripts
npm start --prefix ui/terminal -- --help
```

Start a shared session through the existing OpenRUA CLI or TUI. For example,
after completing the installation, image build, and agent login instructions:

```sh
openrua serve panda --sim robosuite --name pi-demo --agent claude-code
```

Attach from another terminal:

```sh
npm start --prefix ui/terminal -- --endpoint "$HOME/.openrua/sandboxes/pi-demo/endpoint.json"
```

The endpoint belongs to the running session. If you changed OpenRUA's home,
supply that session's actual endpoint path. The file contains a private token;
pass the path, never copy its contents into a URL or a public report.

The two-terminal setup is for this isolated prototype. It is not a proposal to
make the finished product require two terminals or a separate Node install.

## Keyboard interaction

| Input | Result |
| --- | --- |
| Enter | Send through the shared queue |
| Ctrl+J | Insert a newline |
| `/` | Complete available commands |
| `/tools` | Select a tool result to expand or collapse |
| `/queue` | Show waiting instructions |
| Esc or `/interrupt` | Open an interruption confirmation |
| `/continue` | Confirm continuation of the paused queue |
| `/retry` | Retry an unconfirmed send using its original request ID |
| Ctrl+D on an empty input, or `/quit` | Detach without ending the session |
| Ctrl+C | Clear a draft; detach if the input is empty |

Selection lists use arrows, Enter, and Escape. Interruption and continuation
select Cancel initially. Reconnecting replays the server's events, not robot
actions. Messages from another client appear in the same transcript.

Setup, `/resume` selection, queue editing, answering agent questions, and ending
a session still use the existing clients. Unknown execution results also need
those clients for explicit resolution. This prototype does not auto-resume a
paused queue or start a second native agent.

## Verify

Install the Python development environment first, then:

```sh
. .venv/bin/activate
npm ci --prefix ui/terminal --ignore-scripts
npm test --prefix ui/terminal
```

Tests run against the real local HTTP service and SQLite store, with a controlled
native-agent transport. They cover shared input, event replay, tool expansion,
interruption, detachment, and idempotent retries. A keyboard test exercises Pi's
editor and selection list through a terminal test double. No model credentials,
paid calls, simulator, or robot are required.

These checks do not establish real-terminal IME behavior, SSH latency handling,
or usability with long real-agent transcripts. Those need interactive trials
before changing the default UI.

## Boundaries and upstream

- `src/client.mjs`: existing authenticated local HTTP API only.
- `src/controller.mjs`: journal cursor and unconfirmed request identity.
- `src/view.mjs`: session events presented with Pi components.
- `src/app.mjs`: keyboard actions and component composition.
- `src/main.mjs`: executable entry point; owns process exit handling.

Pi is pinned to 1.0.2 in `package-lock.json`. Its
[chat-simple example](https://github.com/earendil-works/pi/blob/main/packages/tui/examples/chat-simple.ts)
and interactive UI informed the component composition. Pi is MIT-licensed;
its license remains in the installed dependency. A future bundled distribution
must retain Pi and transitive dependency notices alongside the runtime notices.
