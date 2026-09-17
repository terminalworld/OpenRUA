---
summary: Requirements, installing the package, building the images, logging an agent in, and the doctor that checks it all
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
| Python | 3.10 or newer on the host; everything else (ROS 2, the simulators, the agent CLIs) lives in the images |
| Agent | a Claude Code login (Claude subscription or API key) or a Codex login (ChatGPT subscription or OpenAI API key) |

## The package

```bash
pip install openrua
openrua --version
```

The command is `openrua`; `openrua --help` lists the verbs and
[cli.md](cli.md) has every flag. Rendering demo videos (`openrua
demo`) needs the `demo` extra: `pip install
'openrua[demo] @ git+https://github.com/terminalworld/OpenRUA'`.

## The images

OpenRUA runs three containers: the robot (when simulated), the sandbox
(the agent's terminal: `ros2`, `rclpy`, the docs, the agent's CLI) and
the proxy (the sandbox's only route out, limited to the agent's model
API). Each is an image; build them once, `build` never runs on its own.

```bash
openrua build                            # sandbox + proxy for the default agent, and the default benchmark's simulator image
openrua build --bench libero_pro         # one simulator image; --bench / --sim repeatable, --all: every bundled benchmark
openrua build sandbox --agent codex      # --distro jazzy (default) | humble; --agent claude-code (default), repeatable
openrua build proxy                      # the whitelist comes from the same manifests
```

A simulated robot's image holds the whole environment: ROS 2, the
simulator checkouts at their pinned commits, the assets, the Python
environment and this package. It is rendered from the simulator file's
`install:` (and a benchmark's over it) and named after the declaration
it came from, `openrua-sim-<name>`: `--bench libero_pro` makes
`openrua-sim-libero_pro`; a benchmark that adds nothing to its
simulator, like CaP-Bench on robosuite, runs in the simulator's
`openrua-sim-robosuite`. The first build of a distro also makes the
ROS base it stacks on (`openrua-sim-base-jazzy`); Docker's layer cache
makes an unchanged rebuild a no-op. Sizes and what each image carries
are in [simulation.md](simulation.md).

The sandbox and proxy images are named after the ROS 2 distro
(`openrua-sandbox-humble`), which a robot's declaration names once as
`ros_distro`; they take the agent's install line and host whitelist
from its manifest and carry a label per agent with the hash of what
went in. A simulator image carries the fingerprint of the declaration
it was rendered from. Those labels are how `doctor` later knows
whether an image is stale.

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
logged out. The other route is a token by file: `openrua bench
--token-file <path>` names a one-line `KEY=value` file (for Claude Code
a `claude setup-token` value as `CLAUDE_CODE_OAUTH_TOKEN`, for Codex an
`OPENAI_API_KEY`) that docker hands to the agent process only. The
token is scrubbed from the trial record like every other secret.

## Your defaults

`~/.openrua/config.yaml` holds what is true on this machine and nowhere
else: the robot, simulator, benchmark and agent a command uses when it
names none; under `agents.<name>`, what this machine knows about each
agent (the model it runs, a CLI version pin, its login directory, its
default options); and under rootless Podman the sandbox's
`--userns=keep-id`. `openrua config set` writes it (`--model`,
`--version` and `--credentials-dir` land under the agent named with
`--agent`, else the default one). Every key is optional; a benchmark
config overrides the file, and command-line flags override both. A
model belongs to an agent, so a benchmark's `agent.model` applies only
when that agent runs; `--agent` switching to another agent takes that
agent's facts and default model. The keys are listed in
[config.md](config.md#userconfig).

## The user directory

`~/.openrua` (`$OPENRUA_HOME`, `--home`) is written by the tool, never
by hand, and holds two kinds of thing. Yours: `config.yaml` and
`credentials/`. A sandbox's: `sandboxes/<name>/`, one directory per
live sandbox with its workspace, its copy of the agent's profile and
what `up` remembers for `agent` and `down`; it goes when the sandbox
goes. A crashed sandbox, or a bring-up that failed, leaves its
directory for you to read; `openrua clean` removes every one whose
containers are not running, `openrua clean --all` powers the running
ones off first. Neither touches `config.yaml` or `credentials/`.

## Check it

```bash
openrua doctor panda --bench libero_pro
```

Read-only. One row per fact: the container engine, each image and
whether its label still matches the manifests (sandbox, proxy) or the
declaration it was rendered from (the simulator image), the agent
login, the user directory. An error row carries the command that fixes
it; the exit code is 1 only when an error is present, and on a pipe
the report is JSON.

## Update or remove

```bash
pip install -U openrua                   # then rebuild: a simulator image carries the package too
openrua build --all                      # rebuild every simulator image (cached layers make it quick)
pip uninstall openrua
rm -r ~/.openrua                         # defaults, logins, sandboxes
docker image rm $(docker image ls -q 'openrua-*')
```
