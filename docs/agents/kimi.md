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

The API file contains only the raw Moonshot Open Platform key. The bundled
endpoint is `https://api.moonshot.ai/v1`. Kimi Coding subscription credentials
and Open Platform API keys are different authentication sources. Local checks
inspect files; provider quota is established by the provider when used.

For a preconfigured project, `openrua --agent kimi` selects it for that new
session. Missing sandbox images are prepared through the ordinary startup flow.

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

For another model or endpoint, copy the bundled manifest into the project,
change its owned settings, then choose **Other agent or configuration file…**. The normal
model field cannot override a profile that was prepared for another model.
