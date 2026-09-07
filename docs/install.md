---
summary: Requirements, the three images, logging an agent in, and the doctor that checks it all
read_when:
  - You are setting RoboCLI up on a machine for the first time
  - robocli doctor is red and you want to know what each row means
---

# Install

## Requirements

- Linux with Docker Engine, or rootless Podman with the `docker`
  command ([podman.md](podman.md)).
- Python 3.10 or newer on the host.
- For a simulated robot: a simulator checkout ([simulation.md](simulation.md)).
  A real robot needs none.
- A coding agent account: Claude Code (Claude subscription or API key)
  or Codex (ChatGPT subscription or OpenAI API key).

```bash
pip install robocli
robocli --version
```

## The three images

RoboCLI runs three containers: the robot (simulated), the sandbox (the
agent's terminal, with `ros2`, `rclpy`, the docs, and the agent's CLI),
and the proxy (the only route from the sandbox to the internet, limited
to the agent's model API). Build them once:

```bash
robocli build robot              # --distro jazzy (default) | humble
robocli build sandbox            # --agent claude-code (default), repeatable; --agent codex
robocli build proxy              # the whitelist comes from the same manifests
```

The sandbox and proxy images take the install line and the host
whitelist from the agents' manifests. Several `--agent` bake several
agents into one image. A manifest pins no version; `--agent
claude-code@2.1.226` pins one at build time, and `agent.version` in a
config makes preflight check the sandbox CLI against it.

## Logging an agent in

Each agent keeps its login in a profile directory of its own under
`~/.robocli/credentials/<agent>/`, never your personal one: OAuth
refresh tokens rotate, and a second copy of a credentials file
invalidates the first.

```bash
CLAUDE_CONFIG_DIR=~/.robocli/credentials/claude-code claude login
CODEX_HOME=~/.robocli/credentials/codex codex login
```

`robocli doctor` prints the exact command for the agent it finds
missing. The alternative is a token passed by file: `robocli run
--token-file <path>` points at a one-line `KEY=value` file that docker
hands to the agent process only; the token is scrubbed from every
trial record.

## Your defaults

`~/.robocli/config.yaml` holds what is true on this machine: the default
agent and model, a credentials directory, and under rootless Podman the
sandbox's `--userns=keep-id`. Every key is optional; a benchmark config
overrides it, and command-line flags override both. The full key list
is in [config.md](config.md).

## Doctor

```bash
robocli doctor panda-sim
```

Read-only. One row per fact: the container engine, each image and
whether it still matches the manifests, the simulator venv, the login,
the user directory. Errors carry the command that fixes them, and the
exit code is 1 only when an error is present; on a pipe the report is
JSON.
