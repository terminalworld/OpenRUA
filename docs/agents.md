---
summary: How coding agents plug into RoboCLI and how to add your own
read_when:
  - You want to run a coding agent other than the bundled one
  - You are adding an agent
  - You want to know what the harness needs from an agent and what it does without
---

# Agents

RoboCLI is agent-agnostic by construction: the robot, the sandbox, the
proxy and the trial runner never mention a particular agent. Everything
about one coding agent lives in two files, and every consumer reaches
the agent through `robocli.agents.get(name)`:

- the manifest, `configs/agents/<name>.yaml`: the facts (name, default
  model, install line, proxy whitelist, login layout, token variable,
  version, default options), no code;
- the hooks module, `plugins/agents/<hooks>.py`: the behaviour (launch
  command, interactive command, transcript parsing, quota handling,
  replay), a subclass of `robocli.agents.Agent` exposing `HOOKS`.

A test (`tests/test_agent_boundary.py`) fails the build if agent-specific
knowledge appears anywhere else.

## The manifest

`robocli/configs/agents/claude-code.yaml` is the reference. Keys:

| Key | Required | Meaning |
|---|---|---|
| `name` | yes | the name configs use under `agent.name` |
| `default_model` | yes | model when the config names none |
| `hooks` | for launch | hooks module name (`plugins/agents/<hooks>.py`) |
| `binary` | | the CLI executable inside the sandbox |
| `version` | | the CLI version the install line pins; `{version}` in `install` is replaced with it |
| `install` | | one-line root shell chain that installs the CLI into the sandbox image |
| `whitelist` | | regexes of the hosts the CLI must reach through the proxy |
| `credentials` | | `dirname`, `filename`, `config_env`, `mount_point`: a profile-directory login |
| `token_env` | | environment variable carrying a long-lived token (passed by file) |
| `version_argv` | | command printing the CLI version (recorded per trial) |
| `instruction_file` | | the instructions file the CLI reads on its own |
| `default_options` | | knobs a config may override under `agent.options` |

`robocli config schema` prints the same list with descriptions
(`AgentManifest`).

## The hooks module

A subclass of `robocli.agents.Agent` (`robocli/agents/base.py`). One
method is required, `launch_argv`: the `docker exec` command that runs
the agent headless on a task inside the sandbox, transcript on stdout.
Every other hook has a documented default, and a consumer that finds the
default does without: `interactive_argv` (`robocli agent`),
`sandbox_cli_check`, `login_hint`, `token_hint`, the quota hooks
(`quota_probe_argv`, `quota_window_open`, `matches_quota_anomaly`,
`read_rate_limits`, `quota_since`), the transcript hooks (`read_final`,
`scan_transcript`, `assistant_turns_before`) and `replay_ops`. The
manifest's fields are available on `self`. An agent's `capabilities` is
the set of hooks its class overrides; `robocli agents` lists them.

## The smallest agent

```yaml
# ~/.robocli/agents/my-agent.yaml
name: my-agent
default_model: some-model
install: pip install my-agent-cli
whitelist: ['^api\\.example\\.com$']
hooks: my_agent
```

```python
# ~/.robocli/plugins/agents/my_agent.py
from robocli.agents import Agent

class MyAgent(Agent):
    def launch_argv(self, sandbox, prompt, model, max_turns, proxy, **_):
        return ["docker", "exec", "-u", "robot", "-w", "/workspace",
                "-e", f"HTTPS_PROXY={proxy}", sandbox,
                "my-agent", "--model", model, "--max-turns", str(max_turns), prompt]

HOOKS = MyAgent
```

Both halves are looked up bundled first, then in the user directory. A
user manifest carrying a bundled name is reported by `robocli doctor`
and ignored. To ship an agent with RoboCLI, put the two files under
`robocli/configs/agents/` and `robocli/plugins/agents/` and open a pull
request.

Conformance, from your own tests:

```python
from robocli.testing import check_manifest

def test_conforms():
    check_manifest("~/.robocli/agents/my-agent.yaml")
```

## Using an agent

- Configs: `agent.name: my-agent`, `agent.model`, `agent.options`
  (over the manifest's `default_options`), `agent.credentials_dir`.
  The same keys in `~/.robocli/config.yaml` are your defaults; the
  package default agent is in `robocli/configs/config.yaml`.
- Images: `robocli build sandbox --agent my-agent` and `robocli build
  proxy --agent my-agent` take the install line and the whitelist from
  the manifest. Several `--agent` bake several agents into one image;
  the images carry a label with the hash of what went in, and `robocli
  doctor` compares it with what the selected agents' manifests say today.
- Login, two ways. A profile directory (`~/.robocli/credentials/<name>/`
  by default, or `agent.credentials_dir`): log in once on the host with
  that directory as the CLI's config dir; `login_hint` prints the
  command, and the credentials file is bind-mounted into every sandbox
  as one shared file. Or a token: `robocli run --token-file <path>`
  points at a `KEY=value` file that docker hands to the agent process
  only; `token_hint` says how to mint one. The token is scrubbed from
  the trial record like any other secret.
- Post-hoc: every trial's `operator_meta.agent` names the agent that
  ran, so audit tools resolve the same one.

## Bundled

`claude-code` (`configs/agents/claude-code.yaml`,
`plugins/agents/claude_code.py`): Claude Code, headless `claude -p` with
the stream-json transcript, profile-directory or token login, quota and
transcript accounting, shell/write/edit replay. It is the reference
implementation for all of the above.
