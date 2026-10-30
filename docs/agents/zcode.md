# ZCode (preview)

ZCode is included in the normal OpenRUA installation as `zcode`. Setup lists it
after the environment choice. The integration pins the original ZCode CLI to
**0.16.9**, source commit `29628c9`, and model **glm-4.5-air**.

## Choose it in setup

Run `openrua --setup`, choose an environment, then choose **ZCode (preview)**.
Review the model and authentication, choose **Continue to review**, then start.
Choosing an agent does not start a model or change the billing mode.

**API key (billed separately)** selects an explicit BigModel prepaid API key:

```sh
openrua config set --user --agent zcode --auth api --api-key-file /absolute/path/to/bigmodel.key
openrua --setup
```

The file contains only the raw key. The bundled API endpoint is
`https://open.bigmodel.cn/api/paas/v4`. For a preconfigured project,
`openrua --agent zcode` selects it for that new session. Startup prepares the
missing sandbox through the ordinary image-installation path.

## Other API providers

The API profile uses ZCode's native `openai-chat-completions` provider. To use
another compatible service, copy the
[bundled manifest](../../openrua/configs/agents/zcode.yaml) into your project,
keep its install recipe and `entry_point: zcode`, then set `default_model`,
`default_options.api_base_url` and `whitelist` for that service. Select the
file through **Other agent or configuration file…** with **API key (billed
separately)** and that provider's key file.

For a Mistral-hosted GLM model, the relevant fields are:

```yaml
default_model: zai-glm-5-3-flash
default_options:
  api_base_url: https://api.mistral.ai/v1
  model_config:
    properties:
      inputFormat:
        supportsImage: true
    optionSpecs:
      reasoningLevel:
        values: [low, high, max]
        map: '{"reasoning_effort": reasoningLevel}'
      maxOutputTokens:
        map: '{"max_tokens": maxOutputTokens}'
  model_options:
    reasoningLevel: low
whitelist:
- ^api\.mistral\.ai$
```

Use the model ID available to your Mistral account. Mistral's
[GLM 5.3](https://docs.mistral.ai/models/zai-glm-5-3) is a text model;
[GLM 5.3 Flash](https://docs.mistral.ai/models/zai-glm-5-3-flash) supports
multimodal input. Choose a model with image input for tasks where the agent
needs to inspect camera images. `model_config` is ZCode's native model rule
for this provider and model; `model_options` selects its initial native
options. ZCode validates the native fields. Empty objects preserve its defaults.
The explicit option maps above keep ZCode's generic reasoning defaults from
sending fields intended for other providers to Mistral.
The selected Flash model accepts `low`, `high` and `max` reasoning levels;
its API rejects `none`. The example selects `low`.

The provider must support streaming chat, tool calls and tool-result messages
through ZCode's native client. A bounded direct API check verified this Flash
model's image input and function calls. A separate native ZCode check wrote
the requested file and completed the turn through the same endpoint.
Robot tasks remain a separate validation step.

## Native profile scope

**ZCode account** imports an already selected Coding Plan key provider from
ZCode's `provider_config.json`. Supported profiles contain a self-contained,
enabled `zhipu-coding-plan-api-key` rule, model `glm-4.5-air`, and the official
BigModel or Z.ai Anthropic endpoint. The adapter discovers `~/.zcode` or
`ZCODE_DATA_BASE_DIR`, then stages only that selected rule in a private config.

Configure and select the provider in native ZCode first. Template-only rules,
ordinary prepaid providers in native mode and encrypted OAuth login accounts
are not imported. A missing or unsupported native profile reports the required
configuration; it never selects API billing. Coding Plan entitlement has not
been checked with a live account. Local profile checks establish file shape,
not account validity.

## Implementation and validation

The [manifest](../../openrua/configs/agents/zcode.yaml) owns installation, version,
model and endpoint. `openrua.plugins.agents.zcode` prepares native profiles and
starts the CLI's `app-server`; its version-specific protocol adapter uses the
existing shared execution owner. Another native CLI version requires protocol
validation before changing the pin.

The pinned CLI with a deterministic local provider exercises two clients,
an explicitly approved native file write, clarification questions with multiple
choices and free-text answers, exact session resume, interruption and
provider-error pause. Questions appear in the existing terminal and browser
controls. A native reminder of the same unanswered question keeps your current
selection and draft. Esc hides the dialog; `/questions` opens it again.
Previous bounded real-provider checks completed a
native file task and two shared-chat turns with context retention. Robot-task
performance, OAuth account reuse and benchmark reproduction remain separate
validation work.

The normal model field cannot override a profile that was prepared for a
different model. Change `default_model` in the project manifest before
preparing a new API profile.
