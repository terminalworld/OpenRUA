# Kimi Code external plugin (experimental)

This example connects the original **Kimi Code 2.1.1** to OpenRUA shared chat.
It uses an external manifest and requires an OpenRUA source checkout with
OpenRUA **0.10.0 or later**. It is not a bundled agent or a guided-setup option.

The example supports **explicit Moonshot API access**, using `kimi-k2.6` at
`https://api.moonshot.ai/v1`. It does not yet reuse a native Kimi subscription.
A Kimi Coding subscription/key is not interchangeable with an Open Platform
key. Missing native login never selects API billing automatically.

## Configure and start

With OpenRUA 0.10.1 or later, `openrua --setup` also accepts this manifest
through **Coding agent → Enter a name or profile path**. Select **API key
file** explicitly and enter the private key file's path. Selecting the agent
alone does not install it or validate an account; follow the build steps below.

From the OpenRUA checkout root, after ordinary
[installation and simulation setup](../../../docs/install.md):

```bash
plugin="$(pwd)/examples/plugins/kimi/agent.yaml"
chmod 600 /absolute/path/to/moonshot.key
openrua config set --agent "$plugin" --auth api --api-key-file /absolute/path/to/moonshot.key
openrua doctor --agent "$plugin"
```

The file contains only the raw key. Keep it outside the repository and robot
workspace. Doctor checks local configuration and files, not online quota.
OpenRUA prepares a private per-session native `config.toml`; it does not copy
or modify your ordinary Kimi login. The profile, native session history and
private server log remain outside the robot workspace. They may contain
credentials or a local server token; do not publish them as workspace artifacts.

Build the sandbox and proxy with the external plugin. For CaP-Bench, use
Humble; retain the existing agents when rebuilding the shared images:

```bash
openrua build sandbox --distro humble --agent claude-code --agent codex --agent "$plugin"
openrua build proxy --agent claude-code --agent codex --agent "$plugin"
openrua --agent "$plugin" --robot panda --bench capbench --task-id 0
```

Start with a read-only message such as asking the agent to list the available
ROS interfaces. This is an experimental robot-chat entry point, not evidence
that Kimi has solved this robot task. Tool approvals appear in the existing
OpenRUA interface; the plugin never answers them on your behalf. Native Kimi
policy decides which operations need approval.

## How it works

`agent.yaml` owns the version, installation, model and endpoint. `adapter.py`
prepares the native API profile and supplies commands through OpenRUA's
ordinary agent contract. Node 24.14.0 is installed privately so another agent's
Node installation cannot change Kimi's interpreter.

For shared chat, `bridge.py` starts `kimi web --no-open` on an automatically
allocated **loopback-only** port inside the sandbox. It discovers that child
process's port from Kimi's own instance registry and uses its native bearer
token. The standalone Python bridge is passed explicitly to the sandbox;
there is no second OpenRUA installation there. It translates pipe requests
into native HTTP calls and reads active-turn transcript snapshots. It does
not make model requests, plan robot actions, or maintain a second user queue.
Closing the execution connection stops the owned native server. Closing a
client leaves the shared execution service running, as for other agents.

`KimiConversation` correlates completion with the submitted prompt ID and
native transcript state. Kimi 2.1.1's ACP can return `end_turn` after some
provider errors, so that signal alone is unsuitable for queue progression.
Native failed turns pause the shared queue; cancellation acknowledgement is
not treated as completion. A lost connection is not silently retried or
replayed. Resume requests target the exact native session and reject another
workspace or still-pending native work.

## Validation and limits

On Linux x86-64, the manifest's installation recipe built Kimi 2.1.1 inside a
ROS 2 sandbox. The actual native process, adapter and existing OpenRUA session
owner were checked with a local deterministic model endpoint and synthetic key:

- Two clients enqueue messages in one native session.
- The native file tool writes a file whose contents are checked independently.
- Native permission requests reach the user and accept explicit allow/deny decisions.
- Restarting the connection resumes the exact native session.
- Interruption retains and pauses queued messages.
- A provider error fails the current message and pauses subsequent messages.

Protocol regressions also check duplicate/foreign snapshots, incremental text,
tool output, late cancellation acknowledgements, structured answer encoding,
and rejection of unknown completion states. Structured questions have protocol
coverage but have not yet been exercised with an actual native question tool.

A separate bounded real-provider check of native Kimi Code 2.1.1 against the
same Moonshot endpoint completed a simple file task with two API requests and
32 output tokens. A subsequent check used this container plugin and OpenRUA's
shared execution owner for two real-model conversation turns from different
clients. Both completed in the same native session, and the second correctly
recalled a marker supplied in the first. That check used two API requests and
13 output tokens, with no tools or robot actions. It establishes basic shared
chat with this provider, not robot-task performance.

Native subscription reuse, physical robots, ARM installation, interactive
native TUI behavior, and benchmark accounting/reproduction remain unvalidated.
Headless and native-terminal commands are supplied, but shared chat has the
lifecycle checks listed above. To change the model or endpoint, edit the
manifest and validate it; this example rejects runtime overrides that would
disagree with the prepared profile. Future Kimi versions require revalidation
because the upstream local server API is experimental.

See the [official local server API](https://github.com/MoonshotAI/kimi-code/blob/main/docs/en/reference/server-api.md)
and [Kimi source](https://github.com/MoonshotAI/kimi-code).
