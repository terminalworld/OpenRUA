# Kimi Code (preview)

Kimi Code is included in the normal OpenRUA installation as `kimi`. Setup
lists it after the environment choice. The bundled integration pins the
original Kimi Code CLI to **2.1.1** and uses model **kimi-k2.6**.

## Choose it in setup

Run `openrua --setup`, choose an environment, then choose **Kimi Code (preview)**.
Review its model and authentication, choose **Continue to review**, then start.
The selected agent belongs to this project. Selecting it does not start a model
or change the billing mode.

**Kimi Code account** uses an existing Kimi profile. **API key (billed separately)**
is an explicit, separate choice. An absent native login never selects API billing.

```sh
openrua config set --user --agent kimi --auth api --api-key-file /absolute/path/to/moonshot.key
openrua --setup
```

The API file contains only the raw key for the selected provider. The bundled
configuration uses a Moonshot Open Platform key and
`https://api.moonshot.ai/v1`. Kimi Coding subscription credentials
and Open Platform API keys are different authentication sources. Local checks
inspect files; provider quota is established by the provider when used.

For a preconfigured project, `openrua --agent kimi` selects it for that new
session. Missing sandbox images are prepared through the ordinary startup flow.

## Other API providers

Copy the [bundled manifest](../../openrua/configs/agents/kimi.yaml) into your
project, keep its install recipe and `entry_point: kimi`, and edit the model,
provider options and network whitelist. Choose the file through **Other agent
or configuration file…** and select **API key (billed separately)**.

The manifest's `default_options` maps to Kimi's native configuration:

| Option | Meaning |
|---|---|
| `provider_type` | Native provider protocol, such as `kimi` or `openai` for OpenAI Chat Completions |
| `api_base_url` | That provider's API base URL |
| `max_context_size` | The selected model's context limit, in tokens |
| `capabilities` | The model's native capability names, such as `tool_use`, `thinking` and `image_in` |

For example, an AWS Bedrock Kimi K3 route can use these manifest fields:

```yaml
default_model: us.moonshotai.kimi-k3
default_options:
  provider_type: openai
  api_base_url: https://bedrock-runtime.us-east-1.amazonaws.com/openai/v1
  max_context_size: 1048576
  capabilities: [tool_use, thinking, image_in]
whitelist:
- ^bedrock-runtime\.us-east-1\.amazonaws\.com$
```

Use the endpoint and model ID available in your AWS Region. This route expects
a **Bedrock API key** as a bearer token; an IAM access key is not interchangeable
with it. The profile stores the provided key and does not generate SigV4
signatures or refresh expiring AWS tokens. See the
[Bedrock model card](https://docs.aws.amazon.com/bedrock/latest/userguide/model-card-moonshot-ai-kimi-k3.html)
and [authentication options](https://docs.aws.amazon.com/bedrock/latest/userguide/inference-chat-completions.html).

Kimi owns the protocol implementation. OpenRUA passes the declared provider
type and model metadata to it without inferring them from the endpoint URL.
Only declare image input when the selected model supports it. The endpoint
must support the native CLI's streaming chat and tool calls, including tool
results in subsequent turns; image tasks also need compatible image input.
The generated configuration is tested with the native CLI. A bounded K3
check through Bedrock verified image input and a function call. A separate
native Kimi Code check wrote the requested file and completed its turn.
Robot tasks remain a separate validation step.

## Native profile scope

The pinned adapter can reuse a selected official Kimi Code OAuth file profile
under `~/.kimi-code` or `KIMI_CODE_HOME`. It currently supports the mainland
`api.kimi.com/coding/v1` service, file storage, OAuth key `oauth/kimi-code`, and a
selected model whose ID matches `kimi-k2.6`. Configure that native profile in
Kimi first with `/login`, then select Kimi Code account in OpenRUA.

Each OpenRUA session gets a private config containing only that managed model.
The native `credentials/` and `oauth/` directories are shared mounts so token
replacement and upstream refresh locks remain visible. Native history and
server state stay session-local. OpenRUA does not implement refresh itself.
Paid native entitlement and refresh have not been verified with a live account;
keyring profiles and other OAuth endpoints are outside this preview's scope.

## Implementation and validation

The [manifest](../../openrua/configs/agents/kimi.yaml) owns the install recipe,
model and endpoint. `openrua.plugins.agents.kimi` owns profile preparation and
launching. The plugin uses Kimi's official local server and a stdlib pipe/HTTP
transport in `kimi_server.py`; the existing session owner retains the queue.
Kimi's ACP `end_turn` alone cannot distinguish all provider failures, so
completion follows the correlated native transcript state.

The real pinned CLI and a local deterministic provider exercise shared
conversation, a native file tool, explicit approvals, structured questions,
exact session resume, interruption and provider-error pause. Previous bounded
real-provider checks also completed a file task and retained context across
two shared-chat turns. These establish the documented chat paths; robot-task
performance, native subscription validity and benchmark reproduction remain
separate validation work.

The normal model field cannot override a profile that was prepared for another
model. Change `default_model` and its provider metadata together in the project
manifest before preparing a new API profile. See
[Kimi's provider documentation](https://moonshotai.github.io/kimi-code/en/configuration/providers.html)
for its native configuration semantics.
