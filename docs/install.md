---
summary: Requirements, installing the package, building the three images, logging an agent in, and the doctor that checks it all
read_when:
  - You are setting OpenRUA up on a machine for the first time
  - openrua doctor is red and you want to know what a row means
  - You are updating or removing an install
---

# Install

## Requirements

| | |
|---|---|
| OS | Linux. Docker Engine, or rootless Podman providing the `docker` command ([podman.md](podman.md)) |
| Python | 3.10 or newer on the host; ROS 2 itself lives in the containers |
| Simulated robots | a simulator checkout and [uv](https://docs.astral.sh/uv/) for its venvs ([simulation.md](simulation.md)); a real robot needs neither |
| Agent | a Claude Code login (Claude subscription or API key) or a Codex login (ChatGPT subscription or OpenAI API key) |

## The package

```bash
pip install git+https://github.com/terminalworld/OpenRUA
openrua --version
```

The command is `openrua`; `openrua --help` lists the verbs and
[cli.md](cli.md) has every flag. Rendering demo videos (`openrua
demo`) needs the `demo` extra: `pip install
'openrua[demo] @ git+https://github.com/terminalworld/OpenRUA'`.

## The three images

OpenRUA runs three containers: the robot (when simulated), the sandbox
(the agent's terminal: `ros2`, `rclpy`, the docs, the agent's CLI) and
the proxy (the sandbox's only route out, limited to the agent's model
API). Build them once; `build` never runs on its own.

```bash
openrua build robot                      # --distro jazzy (default) | humble
openrua build sandbox                    # --distro likewise; --agent claude-code (default), --agent codex, repeatable
openrua build proxy                      # the whitelist comes from the same manifests
```

Images are named after the ROS 2 distro (`openrua-sim-jazzy`,
`openrua-sandbox-humble`), and a robot profile names its distro once
as `ros_distro`; the robot and sandbox images follow from it, so a
Humble robot needs `build robot --distro humble` and `build sandbox
--distro humble` and nothing else. The sandbox and proxy images take
the agent's install line and host whitelist from its manifest and
carry a label per agent with the hash of what went in, which is how
`doctor` later knows whether an image is stale.

A manifest pins no version, so `build sandbox` installs the agent's
current release. To pin one, say so where you build and where you run:
`openrua build sandbox --agent claude-code@2.1.226` and `agent.version:
"2.1.226"` in the config; preflight then checks the sandbox's CLI
against it, and every trial records the version it saw.

## Logging an agent in

Each agent logs in to a profile directory of its own under
`~/.openrua/credentials/<agent>/`, never your personal one: OAuth
refresh tokens rotate, and a second copy of a credentials file
invalidates the first.

```bash
CLAUDE_CONFIG_DIR=~/.openrua/credentials/claude-code claude login
CODEX_HOME=~/.openrua/credentials/codex codex login
```

`openrua doctor` prints the command for whichever agent it finds
logged out. The other route is a token by file: `openrua run
--token-file <path>` names a one-line `KEY=value` file (for Claude Code
a `claude setup-token` value as `CLAUDE_CODE_OAUTH_TOKEN`, for Codex an
`OPENAI_API_KEY`) that docker hands to the agent process only. The
token is scrubbed from the trial record like every other secret.

## Your defaults

`~/.openrua/config.yaml` holds what is true on this machine and nowhere
else: the default agent and model, a credentials directory, the robot
`up` opens when none is named, and under rootless Podman the sandbox's
`--userns=keep-id`. Every key is optional; a benchmark config overrides
the file, and command-line flags override both. The keys are listed in
[config.md](config.md#userconfig).

## Check it

```bash
openrua doctor panda-sim
```

Read-only. One row per fact: the container engine, each image and
whether its label still matches the manifests, the simulator venv the
profile names, the agent login, the user directory. An error row
carries the command that fixes it; the exit code is 1 only when an
error is present, and on a pipe the report is JSON.

## Update or remove

```bash
pip install -U git+https://github.com/terminalworld/OpenRUA   # then rebuild the sandbox if an agent manifest changed
pip uninstall openrua
rm -r ~/.openrua                                              # profiles, logins, simulators, workspaces
docker rmi openrua-sim-jazzy openrua-sandbox-jazzy openrua-proxy
```
