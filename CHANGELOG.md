# Changelog

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
