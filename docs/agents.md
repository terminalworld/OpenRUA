---
summary: What the harness needs from a coding agent, and how to add your own
read_when:
  - You are adding an agent
  - You want to know what the harness needs from an agent and what it does without
---

# Agents

OpenRUA is agent-agnostic by construction: the robot, the sandbox, the
proxy and the trial runner never mention a particular agent. Everything
about one coding agent lives in two files, and every consumer reaches
the agent through `openrua.agents.get(name)`:

- the manifest, `configs/agents/<name>.yaml`: the facts (name, default
  model, install line, proxy whitelist, login layout, token variable,
  version, default options), no code;
- the module the manifest names under `entry_point`
  (`plugins/agents/<name>.py` for the bundled ones): the behaviour
  (launch command, interactive command, transcript parsing, quota
  handling, replay), a subclass of `openrua.agents.Agent` exposing
  `HOOKS`.

Selecting, pinning and logging an agent in is in [install.md](install.md).
An `@VERSION` selection keeps that agent's configured model, authentication,
profile directory and options. It overrides the configured CLI version;
it does not select a separate account or reset the agent's preferences.
When opening a native terminal with `openrua agent`, OpenRUA reuses the
plugin manifest and version `openrua up` recorded at robot startup.
An explicit `agent --agent` selection uses the replacement plugin's own
version rather than inheriting the previous plugin's pin.
A test (`tests/agents/test_agent_boundary.py`) fails the build if agent-specific
knowledge appears anywhere else.

## The manifest

`openrua/configs/agents/claude-code.yaml` is the reference. Keys:

| Key | Required | Meaning |
|---|---|---|
| `name` | yes | the name configs use under `agent.name` |
| `title` | | the agent's name as people write it (`Claude Code`), shown in the terminal and in the sign-in offer |
| `default_model` | yes | model when the config names none |
| `entry_point` | for launch | the module behind the manifest: a bundled name (`plugins/agents/<name>.py`) or a path relative to the manifest (`./my_agent.py`) |
| `binary` | | the CLI executable inside the sandbox |
| `install` | | one-line root shell chain that installs the CLI into the sandbox image; `{version}` in it is replaced by a pin when one is given |
| `version` | | a pin the manifest itself carries; normally absent (see below) |
| `whitelist` | | regexes of the hosts the CLI must reach (its model service and login); a run limited to the model service lets only these through its proxy |
| `credentials` | | `dirname`, `filename`, `config_env`, `mount_point`, optional `native_dir`: a profile-directory login with native discovery; optional `login`: the CLI's own sign-in command, which `openrua login` runs in the sandbox image |
| `token_env` | | environment variable carrying a long-lived token (passed by file) |
| `version_argv` | | command printing the CLI version (recorded per trial) |
| `min_version` | | the oldest CLI release that takes every flag the hooks pass; a build that produces an older one fails, and a session or trial on a sandbox with an older one is refused with the rebuild command |
| `latest_release` | | URL whose JSON names the newest release as `version` (an npm registry's `/latest`); `openrua` offers an update when the sandbox's CLI is older |
| `default_options` | | knobs a config may override under `agent.options` |

[config.md](config.md#agentmanifest) lists the same keys with their
descriptions.

## The hooks module

Plugins can declare `skills = SkillLocations(project, personal)` from
`openrua.agents.base`: the native project directory, relative to the project,
and the absolute sandbox mount for personal skill files. `None`, the default,
leaves an agent's native skill integration unsupported without affecting its
launch hooks. The project runner bridges declared project directories and
mounts the explicitly managed user skill directory. Native discovery and
invocation remain the plugin's responsibility; the generic file catalog does
not claim that a particular CLI has loaded a skill. See [skills](skills.md).
An optional `instructions` project-relative path names a native instructions
file; the project runner appends the same skills guidance there, preserving
existing text, for agents whose other instruction files can hide AGENTS.md.

A subclass of `openrua.agents.Agent` (`openrua/agents/base.py`). One
method is required, `launch_argv`: the command that runs the agent
headless on a task inside the sandbox (`openrua exec`, `openrua bench`),
transcript on stdout;
`self.exec_argv(sandbox, env, token_file)` gives the `docker exec`
prefix every agent shares. OpenRUA's own terminal, its browser, plain
chat and `openrua connect` hold a conversation through a second hook,
`conversation` ([structured conversations](#structured-conversations));
an agent without it runs headless and in its own terminal only.
Every other hook has a documented default, and a consumer that finds the
default does without: `interactive_argv` (`openrua agent`), `steps`
(what `openrua exec` shows while the agent works),
`sandbox_cli_check`, `login_hint`, `token_hint`, the quota hooks
(`quota_probe_argv`, `quota_window_open`, `matches_quota_anomaly`,
`read_rate_limits`, `quota_since`), the transcript hooks (`read_final`,
`scan_transcript`, `assistant_turns_before`), `replay_ops` and
`collect` (what to keep from the sandbox's profile directory once the
run is over: a CLI that writes its own session log there gives the
reading hooks something finer than the stdout stream; the files an
agent keeps are named by that agent alone and land beside the
transcript, so agents never read each other's). The
manifest's fields are available on `self`. An agent's `capabilities` is
the set of hooks its class overrides; `openrua agents` lists them.

### The run's limits

Every launch hook (`launch_argv`, `interactive_argv`, `conversation`)
receives the run's `proxy` and, as a keyword, its `limits`: a dict with
`internet`, `web_search` and `account_connectors`
([what the agent can reach](install.md#what-the-agent-can-reach)). An
empty `proxy` means none: the sandbox goes out directly, so set no proxy
variables. A key that is absent, or `limits=None`, is not limited: pass
nothing and let the CLI do what its user's own configuration says. When
`web_search` or `account_connectors` is `false`, turn that off with the
CLI's own flags. A CLI that reads such a setting only from its profile
gets it in `prepare_profile(..., limits=...)` and
`prepare_api_profile(..., limits=...)` instead.

### Preparing a native profile

Since 0.8.0, both interactive startup and benchmark
trials call `Agent.prepare_profile(source, dest, require_credentials=True,
limits=None)`.
The default preserves the existing JSON-settings and shared-credential-file
layout; existing plugins do not need to override it. The package-level
`openrua.agents.prepare_profile` tuple helper remains available for compatibility.

A plugin with a different native layout overrides this hook and returns
`PreparedProfile(directory, mounts, read_secrets=None)`, imported from
`openrua.agents.base`. `directory` is the session-local profile used for native
records and transcript collection. `mounts` is a tuple of the existing
`SOURCE:DESTINATION` sandbox mount specifications. For example, a plugin can
copy a TOML configuration and return separate mounts for shared credential and
refresh-lock directories without changing the runner or any frontend.

The hook receives the resolved source profile; it must not log in, choose a
different account or endpoint, or silently switch to API billing. Keep native
history and temporary session state separate from shared credentials. When
`require_credentials=False`, the caller supplies authentication separately;
do not mount the native login as a fallback. Validate source materials and
destination ownership before replacing session-local files. The default
implementation rejects source/destination overlap and destination symlinks,
and reports missing credentials before clearing an existing destination.

For credentials outside the legacy JSON layout, supply a `read_secrets` callable
returning the current secret values as nonempty strings. Trials call it before and after execution
and pass both sets to the existing archive redactor; do not print or serialize
those values. This callback is for artifact redaction, not authentication or
network access. It does not scrub private live conversation history.

Profile preparation alone does not establish a working agent integration.
Discovery, doctor checks, native launch, authentication, and conversation
lifecycle still need validation for the plugin's layout. The external-plugin
fixture in `tests/agents/test_profiles.py` exercises both startup paths with
isolated files; it does not authenticate to a model provider.

### Checking local login material

Version 0.8.0 also provides `Agent.inspect_login(source)`.
Doctor and automatic profile linking use this hook, so a plugin can recognize
credential directories or other native formats. Return
`ProfileLogin(available, detail)` from `openrua.agents.base`, or `None` when
no local profile login applies. The default checks the declared credential
file for readability and nonempty content. A new alias is created only when
the selected source reports available login material; existing aliases and
explicit account selections remain intact.

This is a local, read-only check. Do not contact a provider, refresh tokens,
log in, or fall back to another credential source. Keep secret values out of
`detail`. A positive result does not establish an active subscription, a
valid online login, or available quota; the native agent checks those during
use. Doctor reports this distinction rather than declaring the account valid.

### Explicit API profiles

Since 0.8.0, plugins can implement
`prepare_api_profile(key_file, dest, limits=None) -> PreparedProfile`. The caller invokes
this hook only for an explicit `auth.mode: api` selection. The plugin converts
the selected raw key into its CLI's native session-local configuration and
provides `read_secrets` for trial redaction. No native login profile is supplied,
so there is no subscription fallback to accidentally mount. The default rejects
API mode. Codex implements this with native `auth.json` and a file-backed API
login configuration. Claude Code uses native `apiKeyHelper` settings and a private
key file. Core startup and frontends contain no vendor branch.

The hook must not contact a provider or start a task. Reject invalid input
before clearing the destination, keep secret files private, and leave the source
key and the user's native login untouched. Host checks cannot establish online
key validity. See [installation](install.md#explicit-api-authentication) for
configuration and the distinction from the existing benchmark `--token-file`.

## The smallest agent

```yaml
# my-agent.yaml
name: my-agent
default_model: some-model
install: pip install my-agent-cli
whitelist: ['^api\\.example\\.com$']
entry_point: ./my_agent.py
```

```python
# my_agent.py, next to the manifest
from openrua.agents import Agent

class MyAgent(Agent):
    def launch_argv(self, sandbox, prompt, model, max_turns, proxy, limits=None, **_):
        env = ["-e", f"HTTPS_PROXY={proxy}"] if proxy else []
        offline = ["--no-web"] if (limits or {}).get("web_search") is False else []
        return [*self.exec_argv(sandbox, env),
                "my-agent", "--model", model, "--max-turns", str(max_turns), *offline, prompt]

HOOKS = MyAgent
```

Pass the manifest where an agent name is expected (`openrua exec
--agent ./my-agent.yaml`, `openrua build sandbox --agent
./my-agent.yaml`); the module is found next to it. To ship an agent
with OpenRUA, put the manifest under `openrua/configs/agents/`, the
module under `openrua/plugins/agents/`, name it by its bare module name
(`entry_point: my_agent`) and open a pull request.

Conformance, from your own tests:

```python
from openrua.testing import check_manifest

def test_conforms():
    check_manifest("./my-agent.yaml")
```

## Bundled

`claude-code` (`configs/agents/claude-code.yaml`,
`plugins/agents/claude_code.py`): Claude Code, headless `claude -p` with
the stream-json transcript, profile-directory, explicit API, or token login, quota and
transcript accounting, shell/write/edit replay. With `web_search: false`
it disallows the WebSearch and WebFetch tools; with
`account_connectors: false` the session's profile turns off the claude.ai
connectors and the skills and plugins the account syncs. It is the
reference implementation for all of the above.

`codex` (`configs/agents/codex.yaml`, `plugins/agents/codex.py`): Codex,
headless `codex exec --json` with approvals and the CLI's own sandbox
switched off (the container is the sandbox); `web_search: false` passes
`web_search="disabled"` and `account_connectors: false` turns off the
ChatGPT apps (`features.apps=false`); login through a `CODEX_HOME` profile directory (`openrua login codex`, or
`CODEX_HOME=<dir> codex login` with the CLI on the host) or an API key by file; shell commands replayed (its file-change
events carry no content). Its stdout stream counts one turn per prompt,
so `collect` keeps the CLI's own session log as `rollout.jsonl`, from
which turns are model responses (the unit Claude Code's transcript
counts), token usage is per response, and the account's rate limits are
read. The CLI has no turn budget flag, so `protocol.max_turns` is
carried by the runner across segments but not enforced inside a
segment.

Sandbox images carry one label per agent baked in (the hash of its
install line); `openrua doctor` reads them, so one sandbox image can
carry several agents and doctor still says which manifest changed.


## Structured conversations

A plugin may implement `conversation(sandbox, model, proxy, options=None,
session_id=None, limits=None, permissions="auto")` in addition to batch execution and the native terminal.
The default returns `None`; existing plugins need no migration. The hook
returns a `Conversation` containing a piped native-process command and a
`ConversationProtocol`. Construction starts no process. This capability is
consumed by OpenRUA's shared sessions: its terminal (`openrua`), the browser
(`openrua --gui`), plain chat (`--cli`) and `openrua connect`, all through the
same session API.

The protocol is independent of transport and contains all vendor-specific
message knowledge. An execution owner serializes calls, writes each returned
`Update.outbound` frame in order, and feeds native frames to `receive`.
`Update.events` carries normalized events alongside diagnostic `native`
events. The owner records raw frames, owns the queue and keeps the process
alive independently of client connections.

- `begin()` initializes or resumes the native conversation. A `ready` event
  carries its `session_id`; save this identity for a later connection.
  A CLI that accepts an explicit ID at creation may report it with
  `identity_confirmed: false` until a native `session` event confirms it.
- `can_submit` indicates readiness for one turn. `submit(turn_id, text)` uses
  a new caller-assigned identity; subsequent messages wait in the owner's queue.
- `can_steer` indicates that a user message can join the running turn, as a
  coding agent's own terminal passes on what is typed while it works.
  `steer(turn_id, message_id, text)` adds it; the plugin then emits
  `steer_delivered`, or `steer_rejected` so the owner queues the message again.
  The default is never: the message waits for the turn to end.
- `turn_started`, `text_delta`, `item`, and `turn_finished` events retain that
  identity. A completed agent turn is not proof of robot task success. A turn
  the agent starts by itself (a background task finished) arrives as
  `turn_started` with a turn ID the owner never submitted and `data.origin`
  `agent`.
- `interrupt(turn_id)` targets the current turn. `interrupt_acknowledged`
  does not mean it has stopped; only a terminal event ends the turn.
- `input_required` supplies questions and choices. `respond(request_id,
  answers)` maps question IDs to lists of strings; replies are separate from
  ordinary queued messages. Each question has `id`, `text`, `choices`, and
  optional booleans `multiple`, `allow_other`, and `secret` (default false).
  Plugins normalize native fields here; clients never interpret vendor schemas.
  Answers use choice labels, with one additional text value only when
  `allow_other` is true (questions without choices accept text).
  Unknown requests emit `unsupported_request` and
  never receive a fabricated approval.
- `configure(setting, value)` changes what the agent does next, as its own
  `/permissions` and `/model` do: `permissions` is `ask` (ask before each
  command that changes something) or `auto`; `model` is a model ID. The
  plugin reports `setting` once in effect (and the starting values when
  ready), or `setting_rejected`. The default raises `NotImplementedError`.
- `commands` lists the agent's own commands (`name`, `description`); a
  client sends them as a message starting with `/`. A turn whose text is
  `/clear` starts a new native conversation: the plugin then reports
  `session` with the new `session_id` and `cleared: true`.
- After `protocol_error`, malformed output, or transport loss, the owner must
  pause and reconcile outstanding work. It must not retry an uncertain action
  or interpret a disconnected client as cancellation.

The first implementation uses the original Codex app-server and its existing
sandbox login profile. It resumes the exact recorded thread, never the latest
thread. Protocol tests cover early notifications, cancellation races, foreign
thread events and input responses. The wire fields were checked against
Codex CLI 0.159.1's generated schemas and the
[official App Server documentation](https://learn.chatgpt.com/docs/app-server).
Live host-side checks with Codex CLI 0.159.1 and `gpt-5.6-sol` completed two
consecutive turns, including reading two integers and writing their sum to a
file. A subsequent check resumed the exact native thread, interrupted a running
shell tool, kept the next client message queued while paused, then completed it
after explicit resume. These checks used the public protocol and execution
owner with an isolated host workspace. A separate Docker integration check
with Codex CLI 0.153.4 and the same model exercised ROS topic discovery, camera
acquisition and a simulated gripper command through `serve`, the browser and
CLI, followed by retained-state shutdown. See [sessions.md](sessions.md#validation-scope)
for the scope and limitations of that check.


Claude Code implements the same optional contract through its original CLI's
bidirectional JSON mode. It retains native workspace instructions, settings,
login and session persistence. Unlike Codex, its interrupt request has no
native turn target: the plugin validates the caller's turn ID, permits one
outstanding turn, and keeps `can_submit` false until both the result and
interrupt acknowledgement arrive. After an interrupt the owner's queue goes
on, whether the native result reports completion or failure; a failed turn
pauses it only when the person did not interrupt that turn. A result without the expected echoed user message is treated as
uncertain, not silently assigned to the current input.

Claude tool approvals and `AskUserQuestion` are converted to the same
`input_required` questions. Replies apply only to the pending request; a
native cancellation expires it. The protocol follows the
[CLI reference](https://code.claude.com/docs/en/cli-reference) and
[Anthropic's control-protocol implementation](https://github.com/anthropics/claude-agent-sdk-python/blob/main/src/claude_agent_sdk/_internal/query.py),
checked with Claude Code 2.1.284. A live check confirmed initialization, the
user-message echo and failure reporting: the account returned a weekly quota
limit, which the owner recorded as a failed turn and a paused queue. Later
Claude Opus 5 simulation checks completed observation and gripper-control turns
in one native conversation; see [sessions.md](sessions.md#validation-scope).
The earlier quota failure is not evidence of successful task execution.

The shared queue and execution owner consume this contract without vendor
branches; their boundaries and current validation scope are described in
[sessions.md](sessions.md).

## Additional native integrations under development

Kimi Code and ZCode are not yet selectable bundled agents. A successful native
API file task is a separate check from a complete OpenRUA robot integration.
The default remains the configured native login; saving a provider key does
not select it or authorize an automatic API fallback.

`plugins/agents/zcode_conversation.py` provides a version-specific conversation
adapter for external plugin development, checked against the official ZCode
CLI 0.16.9 source at commit `29628c9`. It uses the native `app-server` session
compatibility methods. These methods are being replaced upstream; a future
CLI version must be revalidated before this adapter is used with it. The
module does not install ZCode, configure authentication, or register a robot
setup option.

The adapter runs through the existing `Conversation` and `Execution` contracts.
With the real CLI and a local model fixture, validation covered two clients
sharing one native session, an explicitly approved file write, exact session
resume after restarting the native process, interruption with a retained
paused queue, and a provider error that pauses subsequent messages. Protocol
regressions additionally cover late acknowledgements, duplicate and foreign
turn events, permission retries, and unknown completion statuses. These checks
exercise protocol behavior, not model capability or robot control. Subscription
callbacks, structured user questions and robot tasks remain to be validated.
The external example below additionally validates container installation and
the same lifecycle checks in that container. Unsupported client requests are rejected rather than
answered on the user's behalf.

Kimi Code 2.1.1's ACP reports `end_turn` for some non-authentication provider
errors as well as normal completion. A local error fixture reproduced this
behavior. Its official local server instead exposes a correlated transcript
turn with `state: failed` and the provider error. The experimental Kimi example below uses that native status source, with
prompt-correlated completion and the existing shared queue.

Upstream references: [ZCode source](https://github.com/zai-org/ZCode),
[Kimi ACP reference](https://github.com/MoonshotAI/kimi-code/blob/main/docs/en/reference/kimi-acp.md),
and [Kimi local server reference](https://github.com/MoonshotAI/kimi-code/blob/main/docs/en/reference/server-api.md).

### Experimental ZCode external example

The [ZCode example](../examples/plugins/zcode/README.md) adds a pinned source
installation and an explicit API profile to the existing conversation adapter.
It uses the standard external-manifest entry point and shared session service.
Native subscription reuse and robot-task performance are not validated, so it
is not listed as a bundled agent or offered in guided setup. See the example
for its supported endpoint, container lifecycle checks, and remaining
limitations. Bounded real-provider checks completed a file task and two
shared-chat turns in the same native session, retaining conversational context.

### Experimental Kimi Code external example

The [Kimi Code example](../examples/plugins/kimi/README.md) supplies a pinned
2.1.1 install, an isolated native profile for explicitly selected Moonshot API
authentication, and shared chat through the official local server. A small
plugin-owned pipe/HTTP bridge keeps the existing execution owner and user
queue unchanged. Native process and container checks cover file tools,
approvals, exact resume, cancellation and failure pause using a local model
fixture. A bounded real-provider check also completed two shared-chat turns
in the same native session and retained conversational context. Native
subscription reuse and robot-task performance remain unvalidated; the example
is not a default setup option.

### Staging new integrations

New integrations remain external manifests until their documented capabilities
have been validated. Being present in `examples/plugins/` does not make a plugin
a bundled setup choice. Record which paths have offline checks, real-provider
checks and robot-task evidence separately before considering promotion.

The Kimi example now stages native file-based OAuth profiles; the ZCode example
imports self-contained, selected native Coding Plan key profiles. These paths
have offline checks only. ZCode OAuth accounts remain unsupported. Both retain
explicit API mode and reject unsupported native profiles without switching
billing modes. See each example for prerequisites and the next live checks.
