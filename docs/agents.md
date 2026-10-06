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
When opening a native terminal with `openrua agent` or `openrua run`,
OpenRUA reuses the plugin manifest and version recorded at robot startup.
An explicit `agent --agent` selection uses the replacement plugin's own
version rather than inheriting the previous plugin's pin.
A test (`tests/agents/test_agent_boundary.py`) fails the build if agent-specific
knowledge appears anywhere else.

## The manifest

`openrua/configs/agents/claude-code.yaml` is the reference. Keys:

| Key | Required | Meaning |
|---|---|---|
| `name` | yes | the name configs use under `agent.name` |
| `default_model` | yes | model when the config names none |
| `entry_point` | for launch | the module behind the manifest: a bundled name (`plugins/agents/<name>.py`) or a path relative to the manifest (`./my_agent.py`) |
| `binary` | | the CLI executable inside the sandbox |
| `install` | | one-line root shell chain that installs the CLI into the sandbox image; `{version}` in it is replaced by a pin when one is given |
| `version` | | a pin the manifest itself carries; normally absent (see below) |
| `whitelist` | | regexes of the hosts the CLI must reach through the proxy |
| `credentials` | | `dirname`, `filename`, `config_env`, `mount_point`, optional `native_dir`: a profile-directory login with native discovery |
| `token_env` | | environment variable carrying a long-lived token (passed by file) |
| `version_argv` | | command printing the CLI version (recorded per trial) |
| `instruction_file` | | the instructions file the CLI reads on its own |
| `default_options` | | knobs a config may override under `agent.options` |

`openrua config schema` prints the same list with descriptions
(`AgentManifest`).

## The hooks module

A subclass of `openrua.agents.Agent` (`openrua/agents/base.py`). One
method is required, `launch_argv`: the command that runs the agent
headless on a task inside the sandbox, transcript on stdout;
`self.exec_argv(sandbox, env, token_file)` gives the `docker exec`
prefix every agent shares.
Every other hook has a documented default, and a consumer that finds the
default does without: `interactive_argv` (`openrua agent`),
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
    def launch_argv(self, sandbox, prompt, model, max_turns, proxy, **_):
        return [*self.exec_argv(sandbox, ["-e", f"HTTPS_PROXY={proxy}"]),
                "my-agent", "--model", model, "--max-turns", str(max_turns), prompt]

HOOKS = MyAgent
```

Pass the manifest where an agent name is expected (`openrua run
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
the stream-json transcript, profile-directory or token login, quota and
transcript accounting, shell/write/edit replay. It is the reference
implementation for all of the above.

`codex` (`configs/agents/codex.yaml`, `plugins/agents/codex.py`): Codex,
headless `codex exec --json` with approvals and the CLI's own sandbox
switched off (the container is the sandbox) and web search disabled;
login through a `CODEX_HOME` profile directory (`CODEX_HOME=<dir> codex
login`) or an API key by file; shell commands replayed (its file-change
events carry no content). Its stdout stream counts one turn per prompt,
so `collect` keeps the CLI's own session log as `rollout.jsonl`, from
which turns are model responses (the unit Claude Code's transcript
counts), token usage is per response, and the account's rate limits are
read. The CLI has no turn budget flag, so `protocol.max_turns` is
carried by the runner across segments but not enforced inside a
segment.

Images carry one label per agent baked in (the hash of its install line
or whitelist); `openrua doctor` reads them, so one sandbox image can
carry several agents and doctor still says which manifest changed.


## Structured conversations (experimental)

A plugin may implement `conversation(sandbox, model, proxy, options=None,
session_id=None)` in addition to batch execution and the native terminal.
The default returns `None`; existing plugins need no migration. The hook
returns a `Conversation` containing a piped native-process command and a
`ConversationProtocol`. Construction starts no process. This capability is
consumed by the experimental shared `serve` / `chat` CLI and browser client
(`openrua session --name NAME web`), using the same session API.

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
- `turn_started`, `text_delta`, `item`, and `turn_finished` events retain that
  identity. A completed agent turn is not proof of robot task success.
- `interrupt(turn_id)` targets the current turn. `interrupt_acknowledged`
  does not mean it has stopped; only a terminal event ends the turn.
- `input_required` supplies questions and choices. `respond(request_id,
  answers)` maps question IDs to lists of strings; replies are separate from
  ordinary queued messages. Unknown requests emit `unsupported_request` and
  never receive a fabricated approval.
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
interrupt acknowledgement arrive. The owner still pauses its queue after any
user interruption, regardless of whether the native result reports completion
or failure. A result without the expected echoed user message is treated as
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
callbacks, structured user questions, container installation and robot tasks
remain to be validated. Unsupported client requests are rejected rather than
answered on the user's behalf.

Kimi Code 2.1.1's ACP reports `end_turn` for some non-authentication provider
errors as well as normal completion. A local error fixture reproduced this
behavior. Its official local server instead exposes a correlated transcript
turn with `state: failed` and the provider error. Further integration will use
an unambiguous native status source before enabling automatic queue progression.

Upstream references: [ZCode source](https://github.com/zai-org/ZCode),
[Kimi ACP reference](https://github.com/MoonshotAI/kimi-code/blob/main/docs/en/reference/kimi-acp.md),
and [Kimi local server reference](https://github.com/MoonshotAI/kimi-code/blob/main/docs/en/reference/server-api.md).
