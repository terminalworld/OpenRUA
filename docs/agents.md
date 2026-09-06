---
summary: How coding agents plug into RoboCLI and how to add your own
read_when:
  - You want to run a coding agent other than the bundled one
  - You are writing an adapter for a CLI agent
  - You want to know what the harness needs from an agent and what it does without
---

# Agents

RoboCLI is agent-agnostic by construction: the robot, the sandbox, the
proxy and the trial runner never mention a particular agent. Everything
about one coding agent lives in one adapter, a small Python class, and
every consumer reaches it through `robocli.agents.get(name)`. A test
(`tests/test_agent_boundary.py`) fails the build if agent-specific
knowledge appears anywhere else.

## What an adapter declares

An adapter is a subclass of `robocli.agents.Agent`
(`robocli/agents/base.py`). Two attributes and one method are required;
everything else has a default.

| Member | Required | Meaning |
|---|---|---|
| `name` | yes | the name configs use under `agent.cli` |
| `default_model` | yes | model when the config names none |
| `launch_argv(...)` | yes | the `docker exec` command that runs the agent headless on a task inside the sandbox, transcript on stdout |
| `binary` | | the CLI executable inside the sandbox |
| `install` | | one-line root shell chain that installs the CLI into the sandbox image (`robocli build sandbox` bakes it in) |
| `whitelist` | | hostname regexes the CLI must reach; the proxy allows nothing else |
| `credentials` | | `Credentials(dirname, filename, config_env, mount_point)` when the CLI logs in through a profile directory |
| `token_env` | | environment variable carrying a long-lived token, when the CLI accepts one (passed by file, never by value) |
| `version_argv` | | prints the CLI version, recorded in every trial's provenance |
| `instruction_file` | | the instructions file this CLI reads on its own at start; declared, not yet used |
| `default_options` | | knobs a config may override under `agent.options` (the bundled adapter: `effort`, `autocompact`, bash timeouts) |

Optional hooks, each with a documented default the harness is happy
with: `interactive_argv` (for `robocli agent`), `sandbox_cli_check`,
`login_hint`, `token_hint`, the quota group (`quota_probe_argv`,
`quota_window_open`, `matches_quota_anomaly`, `read_rate_limits`,
`quota_since`), transcript accounting (`read_final`, `scan_transcript`,
`assistant_turns_before`) and `replay_ops`. An adapter's
`capabilities` is the set of hooks it overrides; `robocli agents`
prints it. A consumer that finds a default does without: no
interactive mode, no quota bookkeeping, no replay.

## The smallest adapter

```python
# ~/.robocli/agents/my-agent.py
from robocli.agents import Agent

class MyAgent(Agent):
    name = "my-agent"
    default_model = "some-model"
    binary = "myagent"
    install = "pip install my-agent-cli"
    whitelist = (r"^api\.example\.com$",)

    def launch_argv(self, sandbox, prompt, model, max_turns, proxy,
                    options=None, session_id=None, resume=False,
                    token_file=None, **_):
        return ["docker", "exec", "-u", "robot", "-w", "/workspace",
                "-e", f"HTTPS_PROXY={proxy}", "-e", f"HTTP_PROXY={proxy}",
                sandbox, self.binary, "--model", model,
                "--max-steps", str(max_turns), prompt]

AGENT = MyAgent()
```

Adapters are looked up bundled first (`robocli/agents/`), then in
`~/.robocli/agents/<name>.py` (the module must expose `AGENT`). A
file that fails to import is listed with its error by
`robocli agents` and hides nothing else. To ship one with RoboCLI,
put the module in `robocli/agents/` and open a pull request.

Prove it conforms from your own tests:

```python
from robocli.testing import check_agent
from my_agent import AGENT

def test_conforms():
    check_agent(AGENT)
```

## Using an agent

- Configs: `agent.cli: my-agent`, `agent.model`, `agent.options`
  (over the adapter's `default_options`), `agent.credentials_dir`.
  The same keys in `~/.robocli/config.yaml` are your defaults.
- Images: `robocli build sandbox --preinstall "$(python -m robocli.agents
  preinstall --cli my-agent)"` and `robocli build proxy --whitelist
  "$(python -m robocli.agents whitelist --cli my-agent)"`. Several
  `--cli` bake several agents into one image; the images carry a label
  with the hash of what went in, and `robocli doctor` compares it with
  what the selected agents would emit today.
- Login, two ways. A profile directory (`~/.robocli/credentials/<name>/`
  by default, or `agent.credentials_dir`): log in once on the host with
  that directory as the CLI's config dir; the adapter's `login_hint`
  prints the command, and the credentials file is bind-mounted into every
  sandbox as one shared file. Or a token: `robocli run --token-file
  <path>` points at a `KEY=value` file that docker hands to the agent
  process only; the adapter's `token_hint` says how to mint one. The
  token is scrubbed from the trial record like any other secret.
- Post-hoc: every trial's `operator_meta.cli` names the adapter that
  ran, so audit tools resolve the same one.

## Bundled

`claude-code` (`robocli/agents/claude_code.py`): Claude Code, headless
`claude -p` with the stream-json transcript, profile-directory or
minted-token login, quota and transcript accounting, shell/write/edit
replay. It is the reference implementation for all of the above.
