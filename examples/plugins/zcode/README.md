# ZCode external plugin (experimental)

This example connects the original **ZCode CLI 0.16.9** to OpenRUA's shared
chat through an external manifest. It is not a bundled agent or a setup-menu
option. Use an OpenRUA source checkout and OpenRUA **0.8.1 or later**.

The current example supports **explicit BigModel prepaid API access**, using
`glm-4.5-air` at `https://open.bigmodel.cn/api/paas/v4`. Native subscription
reuse is not supported yet. It never chooses API billing after a login error;
without an explicit API selection it stops with configuration instructions.
Coding Plan and prepaid endpoints are different; see the
[official ZCode configuration guide](https://zcode.z.ai/en/docs/configuration).

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

This establishes a simple real-provider file task, not robot-task performance,
subscription compatibility, ARM support, or a guarantee of account quota. The
native terminal command is provided but its interactive UI has not been checked here.
Headless launch is available; benchmark turn accounting, quota recovery and
published benchmark reproduction are not validated for this example.

## Configure and try shared chat

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
