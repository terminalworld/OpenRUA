# Changelog

## 0.9.1

- Read a fresh session snapshot after message submission instead of reusing an earlier in-flight poll. Newly queued instructions now appear immediately in the queue menu, and a failed poll does not block the next refresh.
- Document a bounded real-provider file task through the experimental ZCode container plugin. Robot-task performance and subscription reuse remain unvalidated.

## 0.9.0

- Add keyboard authentication selection to setup. Native CLI login remains the default; API billing requires an explicit choice and a private key file path. Choices stay separate per agent, unsupported plugins do not offer API mode, and invalid selections cannot save defaults or start resources.
- Preserve explicit user authentication when composing benchmark configurations. Schema-generated native defaults no longer silently replace an API choice; authentication actually written in a benchmark retains its documented precedence.

- Add an experimental ZCode external plugin example with a pinned official source build, explicit BigModel API profile, and native shared-conversation adapter. Container lifecycle checks use a local model fixture; subscription reuse and robot-task performance remain unvalidated. The example is not a bundled setup option.

## 0.8.1

- Support explicit Claude Code API authentication through `config set --agent claude-code --auth api --api-key-file ...`. The native CLI reads a private session-local key through `apiKeyHelper`; subscription credentials are not mounted, and keys stay out of launch arguments. The same configuration works for shared chat, native terminal sessions, and benchmark trials. Native login remains the default.

## 0.8.0

- Add explicit Codex API authentication through `config set --auth api --api-key-file ...`, shared by interactive and benchmark startup. Native login remains the default; API profiles omit subscription credentials, and unsupported plugins reject the selection before updating defaults. Local checks do not spend tokens or validate quota.
- Let agent plugins prepare native profiles and inspect local login material for discovery and doctor. Custom layouts can share credential and refresh-lock directories without adding vendor-specific logic to the runner. Local checks do not authenticate online or switch billing sources.

## 0.7.1

- Retain the shared service endpoint when abnormal exit cannot confirm resource shutdown. This keeps `clean`, including `clean --all`, from deleting the session's files until the remaining resources have been inspected. Successful shutdown still removes the endpoint and preserves the conversation and workspace.
- Add a lifecycle regression covering two clients, queued execution, reconnection, shutdown, offline file access and explicit deletion, using real local HTTP, SQLite and a controlled child process. This verifies session behavior, not model or physical robot performance.
- Include the development-only ZCode conversation adapter and its documented validation limits. Kimi Code and ZCode are not yet selectable product integrations.

## 0.7.0

- Browse retained workspace files with `/files` after opening an ended or unavailable conversation through `/resume`. The same keyboard directory, text and image viewers now work without restarting the robot or agent.
- Use the recorded workspace path, including external directories. Missing files leave the transcript accessible. The temporary local viewer rejects mutation requests, preserves the existing file access restrictions, and closes when the history screen exits.
- Verify archived file browsing through a real terminal in the isolated distribution check, alongside Python and terminal integration tests. Image rendering still depends on terminal support; saved observations are not a live video feed.

## 0.6.0

- Make the Pi keyboard terminal the default for `openrua`, setup, history, and `chat --tui`. Remove the Textual frontend and dependency. Ordinary installation now includes the terminal runtime; `[pi]` and `[tui]` remain compatibility extras.
- Use Enter to send, Ctrl+J for newlines, slash commands for tools/files/queues, and keyboard selectors for configuration and confirmations. Ctrl+D on an empty input detaches. No mouse is required.
- Discover native Claude Code and Codex file-based logins and create absent OpenRUA profile aliases on launch, without copying OAuth credentials. Explicit profile paths, native environment overrides, and existing account directories are preserved. Doctor shows the effective credential source; cached files do not prove that a login is still valid.
- Avoid repeating a native error when it arrives as both a message and a final turn result.
- Keep browser/plain CLI startup independent of terminal runtime checks. Synchronize installation, tutorials, keyboard references and package validation with the new default.

## 0.5.0

- Browse saved workspace files from either terminal frontend without sending agent instructions. Pi uses its keyboard selector, scroll view and image component; the default TUI previews text and shows image metadata. Both reuse the existing read-only workspace API.
- Preserve the chat draft while browsing. Refresh changed files, report missing/oversized paths, and reject symlinks through the existing server checks. Image rendering depends on terminal support; offline history remains transcript-only.
- Check Docker daemon access and reject missing, empty or non-file credentials in readiness reports. Login-file presence is explicitly separate from authentication and quota. Existing images and shared proxy containers are not automatically replaced.
- Give README readers separate TUI and benchmark/demo entry points. Explain How it works through the paper’s workspace-as-harness, file-I/O perception and coding-based control.

## 0.4.3

This release includes the startup and diagnostic changes prepared for 0.4.2.
The 0.4.2 publishing gate caught a test that relied on a pre-existing Docker
network; no 0.4.2 package was published. Proxy integration tests now create
and clean up isolated networks and use unique container/image names, so they
exercise the same behavior on a clean CI runner and a development machine.


### Fixed

- Proxy startup verifies Docker network membership before returning a URL and
  preserves Docker's original errors on failed startup or attachment. An
  existing proxy is never replaced to repair another session's startup.
- Readiness checks inspect the standing proxy container separately from the
  image, so rebuilding an image cannot hide its older running policy. Warning
  repair guidance is now visible in text reports as well as JSON.
- Missing Docker labels use legacy aggregate metadata correctly instead of
  interpreting `<no value>` as an agent hash.

### Changed

- Guided setup adds Panda in the native robosuite Lift scene and in CaP-Bench,
  each backed by real startup, reset, camera, joint-state and no-op trajectory
  checks. The existing RoboCasa365 entry remains available.
- Environment validation accepts native scene names without a benchmark task
  instruction, and checks agent executables through their plugin manifests
  rather than a fixed list of CLI names.
- README and GitHub citation metadata link to the published OpenRUA paper.

### Scope

The new environment records cover their default scenes with existing agent
sandbox images. They do not establish model task success, every benchmark task,
physical deployment, or an entirely fresh machine installation. Shared-proxy
policy changes still require an explicit maintenance step after affected
sessions end; ordinary startup does not interrupt them.

## 0.4.1

### Fixed

- Robot, simulator and benchmark fields now filter linked, tested combinations
  instead of offering an arbitrary cross-product of registered names. Recorded
  RoboCasa365 startup, reset, ROS observations and no-op trajectory checks back
  the initial guided entry. Other profiles remain available through explicit CLI options.
- Invalid setup selections cannot overwrite saved defaults. Clearing a benchmark
  stays cleared during checks and subprocess launch. Mismatched benchmark and
  simulator selections fail before touching resources.

### Changed

- Prepare and start builds missing simulation, sandbox and proxy images for the
  selection, with progress, retained logs and retry. Existing images are reused.
  Save and check remains read-only with respect to images and running resources.
- The same preparation path serves Textual, Pi, and root CLI/browser launches.
  Low-level up, run and serve retain explicit preparation behavior.

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
