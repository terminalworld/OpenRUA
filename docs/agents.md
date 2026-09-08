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
- the hooks module, `plugins/agents/<hooks>.py`: the behaviour (launch
  command, interactive command, transcript parsing, quota handling,
  replay), a subclass of `openrua.agents.Agent` exposing `HOOKS`.

Selecting, pinning and logging an agent in is in [install.md](install.md).
A test (`tests/agents/test_agent_boundary.py`) fails the build if agent-specific
knowledge appears anywhere else.

## The manifest

`openrua/configs/agents/claude-code.yaml` is the reference. Keys:

| Key | Required | Meaning |
|---|---|---|
| `name` | yes | the name configs use under `agent.name` |
| `default_model` | yes | model when the config names none |
| `hooks` | for launch | hooks module name (`plugins/agents/<hooks>.py`) |
| `binary` | | the CLI executable inside the sandbox |
| `install` | | one-line root shell chain that installs the CLI into the sandbox image; `{version}` in it is replaced by a pin when one is given |
| `version` | | a pin the manifest itself carries; normally absent (see below) |
| `whitelist` | | regexes of the hosts the CLI must reach through the proxy |
| `credentials` | | `dirname`, `filename`, `config_env`, `mount_point`: a profile-directory login |
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
`scan_transcript`, `assistant_turns_before`) and `replay_ops`. The
manifest's fields are available on `self`. An agent's `capabilities` is
the set of hooks its class overrides; `openrua agents` lists them.

## The smallest agent

```yaml
# ~/.openrua/agents/my-agent.yaml
name: my-agent
default_model: some-model
install: pip install my-agent-cli
whitelist: ['^api\\.example\\.com$']
hooks: my_agent
```

```python
# ~/.openrua/plugins/agents/my_agent.py
from openrua.agents import Agent

class MyAgent(Agent):
    def launch_argv(self, sandbox, prompt, model, max_turns, proxy, **_):
        return [*self.exec_argv(sandbox, ["-e", f"HTTPS_PROXY={proxy}"]),
                "my-agent", "--model", model, "--max-turns", str(max_turns), prompt]

HOOKS = MyAgent
```

Both halves are looked up bundled first, then in the user directory. A
user manifest carrying a bundled name is reported by `openrua doctor`
and ignored. To ship an agent with OpenRUA, put the two files under
`openrua/configs/agents/` and `openrua/plugins/agents/` and open a pull
request.

Conformance, from your own tests:

```python
from openrua.testing import check_manifest

def test_conforms():
    check_manifest("~/.openrua/agents/my-agent.yaml")
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
login`) or an API key by file; token usage from the transcript; shell
commands replayed (its file-change events carry no content). The CLI
has no turn budget flag, so `protocol.max_turns` is carried by the
runner across segments but not enforced inside a segment.

Images carry one label per agent baked in (the hash of its install line
or whitelist); `openrua doctor` reads them, so one sandbox image can
carry several agents and doctor still says which manifest changed.
