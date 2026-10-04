# Changelog

## 0.4.0

### Added

- Optional Pi terminal, started with `openrua --tui pi` after installing
  `openrua[pi]`. The default Textual frontend remains available.
- Keyboard configuration and searchable `/resume` history, reusing the existing
  configuration, readiness checks, background launcher, and retained sessions.
- Pi slash commands for tool expansion, queue edits/withdrawal, agent questions,
  interruption, queue continuation, explicit end, and client detachment.
- Prepared frontend resources and licenses in wheels and source archives;
  the optional Node runtime is installed by pip, without a system Node requirement.
- Build manifest verification and clean-install pseudo-terminal checks alongside
  session integration tests and the existing Python/browser CI jobs.

### Scope

Pi uses the shared execution service and native agent plugins; it does not add
an agent loop. Linux x86_64 packaging, simulated keyboard input, and local
pseudo-terminal startup are verified. Real-terminal IME/SSH behavior and live
model/robot interaction still need user trials. Secret inputs and resolving
unknown execution results use existing clients. Agent/model hot switching
and persistent multi-panel layouts remain deferred.

## 0.3.0

### Added

- Unnamed product launches create unique session IDs automatically. History
  titles use the first user instruction, without an extra model call.
- `/resume` searches history from both the setup form and chat. `--resume [ID]`
  opens the selector or a specific conversation; live sessions reuse their
  existing execution owner and native conversation.
- Ended or unavailable conversations can be inspected read-only in the TUI.
  Selection never starts a robot, replays a task, or changes retained state.

### Changed

- Bare `openrua` now prepares a new session instead of implicitly reconnecting
  to a fixed default name. Use `/resume` or `--resume` to find earlier work.
- Explicit `--name` and existing named directories remain compatible. Native
  `run`, foreground `serve`, and administrative subcommands keep their defaults.
- README, installation, terminal guide, walkthrough, and CLI reference document
  the same new-session and history behavior.

### Scope

Automated checks cover legacy history, distinct generated IDs, live-owner
reconnection, stale endpoints, read-only transcripts, and local slash commands.
The web UI still represents one live session; select it with
`openrua --gui --resume ID`. Restarting ended execution owners, renaming history
entries, and migrating native agent conversations remain outside this release.

## 0.2.0

### Added

- `openrua` opens a setup form for a new shared session or reconnects to the
  named running session. `--gui` and `--cli` select browser or plain text chat.
- `--setup` edits the same validated `config.yaml` used by `config set` and
  direct file edits. The form reports missing preparation without automatically
  building images, logging in, or submitting robot tasks.
- Background startup reuses the existing `serve` command, waits for its API,
  retains startup logs, and prevents competing startup attempts.

### Changed

- TUI dependencies are included in the default installation. The `[tui]` extra
  remains accepted for compatibility. Administrative commands still import the
  interface lazily.
- Configuration updates preserve unrelated fields and use a lock and atomic
  replacement; `config set --bench null` clears the benchmark default.
- README, installation, shared-session walkthroughs, command reference, and
  architecture documentation describe the same startup and configuration paths.

### Upgrade and scope

Install with `pip install -U 'openrua==0.2.0'`. Existing native-terminal and
benchmark commands keep their behavior. Closing a shared client leaves its
service running; explicitly end it to stop resources. Retained names are not
reused automatically, and startup flags cannot reconfigure a running session.

Tests cover shared configuration, headless setup interaction, real child-process
startup and API readiness, duplicate startup, and retained failure output.
These additions do not establish physical-hardware reliability or execution-owner
crash recovery. In-session agent/model switching remains outside this release.

## 0.1.0

This release adds shared robot conversations and an optional terminal UI while
preserving the original native-agent terminal and benchmark commands.

### Added

- A local session service (`openrua serve`) with a persistent message queue,
  retained execution events, and native conversation adapters for Codex and
  Claude Code. Multiple clients can submit instructions to the same queue.
- Browser chat with streamed replies, tool output, pending agent questions,
  queue editing and withdrawal, interruption, and explicit queue resumption.
- Read-only browsing and previews of saved workspace images and files.
- An optional chat-first TUI (`openrua chat --tui`), using the same session API.
  Install it with `pip install 'openrua[tui]==0.1.0'`.
- Plain CLI chat and session-management commands for reconnection, status,
  queue operations, and reconciliation of unknown execution outcomes.
- Real-robot task startup without requiring a benchmark configuration.

### Changed

- Ending an interactive session retains its workspace and records. Deletion is
  a separate explicit operation. Closing a shared chat client leaves the
  execution service running.
- Interrupted, failed, or unknown execution pauses the queue and retains pending
  instructions. Restarting that queue requires an explicit action.
- Resource shutdown is handled by the process that owns the robot and native
  agent. Partial shutdown failures remain visible and retryable.
- The default Codex reasoning-effort setting is now high, matching the existing
  Claude Code manifest. Explicitly configure effort when reproducing older runs.

### Fixed

- Native terminal entry points retain the selected agent plugin and version.
- Resumed trials that exhaust their execution budget are no longer discarded
  as quota-limited runs.
- Workspace starter-tool indexing and its corresponding template fingerprint
  are consistent. Existing trial fingerprints remain unchanged.
- CLI reference generation is consistent across supported Python versions.

### Upgrade and validation

Install with `pip install -U 'openrua==0.1.0'`, or use the `tui` extra above.
Run `openrua doctor` for your chosen configuration and rebuild any images it
reports as stale. Follow `examples/shared-session.md` for a complete shared
session walkthrough.

Automated checks cover supported Python versions, the shared HTTP/SQLite
session lifecycle, browser interactions, and terminal interactions. A live Codex
simulation conversation has been exercised. Claude Code's conversation adapter
has protocol and error-path coverage, but successful live model turns have not
yet been validated. Physical hardware has not been validated.

Shared interfaces remain experimental within this regular release. The TUI
currently attaches to a separately started service; a single-command default
TUI, in-session agent/model switching, a TUI file panel, and a dedicated mobile
app are not implemented. Client reconnection does not provide execution-owner
crash recovery or a native-agent TUI handoff.

Older releases are recorded in [GitHub Releases](https://github.com/terminalworld/OpenRUA/releases).
