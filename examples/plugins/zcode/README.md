# ZCode external plugin (experimental)

This example connects the original **ZCode CLI 0.16.9** to OpenRUA's shared
chat through an external manifest. It is not a bundled agent or a setup-menu
option. Use an OpenRUA source checkout and OpenRUA **0.8.1 or later**.

The example supports a limited native Coding Plan profile path and explicitly
selected BigModel prepaid API access. **Neither native subscription validity nor
OAuth account reuse is claimed.** Missing native login never selects API billing
automatically.

## Native Coding Plan preparation (experimental)

This first version imports the **currently selected Coding Plan key provider**
from native ZCode 0.16.9. It discovers `~/.zcode/v2/provider_config.json` or the
same native layout under `ZCODE_DATA_BASE_DIR`, using the standard OpenRUA profile
alias mechanism. A Coding Plan key pays against the provider's subscription;
ordinary `api-key` providers are rejected in native mode even when one exists.

Configure and select the plan provider in native ZCode first. Its selected rule
must be self-contained, with `access.type: zhipu-coding-plan-api-key`, its key,
and the official BigModel or Z.ai Anthropic endpoint. The pinned native CLI
routes these endpoints through its official `zcode.z.ai` Coding Plan gateway. Template-only rules are
not resolved by this example. Set this manifest's `default_model` to the native
selection's `modelId`, then:

```bash
plugin="$(pwd)/examples/plugins/zcode/agent.yaml"
openrua config set --agent "$plugin" --auth native
openrua doctor --agent "$plugin"
```

Follow the image build and launch steps below. Each session receives only the
selected provider in a private mode-0600 configuration, without other API
providers, OAuth credentials or native history. Key changes take effect in new
sessions. Local checks and synthetic fixtures do not establish subscription
entitlement or successful robot tasks.

**OAuth login accounts remain unsupported.** Upstream stores encrypted OAuth
credentials beside other state and binds the default encryption secret to the
host environment. This example neither copies rotating tokens nor mounts the
entire native home into every session. Supporting that path requires a separate
integration and real account validation; a directory link alone is insufficient.
See the pinned [credential cipher](https://github.com/zai-org/ZCode/blob/29628c9acdb81b703bbd4080c207a0e7ce5e276e/apps/zcode-cli/packages/adapters/src/auth/credential-cipher.ts)
and [provider schema](https://github.com/zai-org/ZCode/blob/29628c9acdb81b703bbd4080c207a0e7ce5e276e/packages/provider/src/config/provider-data-schema.ts).

## Explicit prepaid API access

The separately selected API mode uses `glm-4.5-air` at
`https://open.bigmodel.cn/api/paas/v4`. Without an explicit API selection, a
missing or unsupported native profile stops with configuration instructions.

## What was checked

On Linux x86-64, the manifest's installation command built the pinned official
source inside a ROS 2 sandbox image and reported CLI version 0.16.9. The actual
container CLI, this adapter, and OpenRUA's shared session owner passed:

- Two clients queue messages in one native conversation.
- A native file-writing tool executes after explicit permission approval.
- Restarting the connection resumes the exact native session.
- Interruption retains and pauses queued messages.
- A model error fails the current message and pauses the queue.

These lifecycle checks use a deterministic local model endpoint and a synthetic
key. Separately, the adapter's headless launch and prepared API profile were
checked in the same container against the official BigModel endpoint with
`glm-4.5-air`: the native agent wrote a requested file, its contents were checked
independently, and the CLI exited successfully. This bounded check used two API
requests and 51 output tokens.

A separate real-provider check through the container plugin and shared session
service completed two turns from different clients in the same native
conversation. The second turn correctly recalled a marker from the first,
without invoking tools. It used two API requests and 11 output tokens with the
same model and endpoint.

These checks establish a simple file task and conversational context retention,
not robot-task performance, subscription compatibility, ARM support, or a
guarantee of account quota. The
native terminal command is provided but its interactive UI has not been checked here.
Headless launch is available; benchmark turn accounting, quota recovery and
published benchmark reproduction are not validated for this example.

## Configure and try shared chat

With OpenRUA 0.10.1 or later, `openrua --setup` also accepts this manifest
through **Coding agent → Enter a name or profile path**. Select **API key
file** explicitly and enter the private key file's path. Selecting the agent
alone does not install it or validate an account; follow the build steps below.

From the root of the OpenRUA checkout, after completing the ordinary
[installation and simulation setup](../../../docs/install.md):

```bash
plugin="$(pwd)/examples/plugins/zcode/agent.yaml"
chmod 600 /absolute/path/to/bigmodel.key
openrua config set --agent "$plugin" --auth api --api-key-file /absolute/path/to/bigmodel.key
openrua doctor --agent "$plugin"
```

The key file contains only the raw key and stays outside the repository and
robot workspace. Doctor checks the file locally; it does not verify API balance.

Build the sandbox and proxy for your robot's ROS distribution. The following
commands use **Humble**, as required by CaP-Bench, and include the existing
Claude Code and Codex plugins so those agents remain available in the rebuilt
shared images:

```bash
openrua build sandbox --distro humble --agent claude-code --agent codex --agent "$plugin"
openrua build proxy --agent claude-code --agent codex --agent "$plugin"
openrua --agent "$plugin" --robot panda --bench capbench --task-id 0
```

The source build uses a private Node 24.14.0 installation; its generated CLI
keeps that interpreter even when another agent installs a different Node
version. The official source and dependencies remain in the image, so expect a
larger image than for a packaged CLI. Existing running containers are not
replaced by rebuilding an image.

This last command is an experimental robot-chat entry point, not a claim that
the task has been solved with this plugin. Start with a read-only instruction
such as listing the available ROS interfaces.

## How the plugin fits

`agent.yaml` owns the installation pin, permitted host and default model.
`adapter.py` prepares a private native provider configuration and supplies
native commands plus the existing `ZCodeConversation` protocol adapter.
OpenRUA's existing session service owns the queue, connection and workspace;
there is no second agent loop and no ZCode-specific branch in core startup.

The pinned compatibility protocol may change in future ZCode versions. The
adapter rejects another version rather than claiming it is compatible. To try
another model or endpoint, change the external manifest and validate it before
use; runtime model/endpoint overrides are deliberately rejected by this example.
Do not add a subscription credential file to its API profile. ZCode's native
subscription credential encryption depends on the original host environment,
so mounting a login directory alone does not establish compatibility.

## Native preparation validation

Synthetic profiles were loaded by the real pinned CLI in a container with
`--network none`, and a native session was created without submitting a prompt.
Unit tests cover discovery, private session configs, rejection of unsupported
native profiles and preservation of explicit API mode. Kimi additionally shares
the upstream refresh-lock directory and observes atomic token-file replacement.
These checks do not verify paid entitlement, online token refresh or robot tasks.
After obtaining a subscription, validate one small native conversation and its
continuation before testing robot control; do not switch to API on login failure.
