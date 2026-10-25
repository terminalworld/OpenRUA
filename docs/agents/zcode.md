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
an explicitly approved native file write, exact session resume, interruption
and provider-error pause. Previous bounded real-provider checks completed a
native file task and two shared-chat turns with context retention. Robot-task
performance, OAuth account reuse and benchmark reproduction remain separate
validation work.

For another model or endpoint, copy the bundled manifest into the project and
select it through **Other agent or configuration file…**. The normal model field cannot
override a profile that was prepared for a different model.
