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
pip install -U 'openrua>=0.4.1'
openrua --version
```

The default install includes the terminal UI. Run `openrua` to configure and
start a new shared session with an automatic ID. Use `openrua --resume` to find
a previous conversation, or `/resume` inside the TUI. Use
`openrua --gui` for the browser and `openrua --cli` for plain text chat.
The first-run form saves to the same file as `openrua config set`.
See [Your first shared robot session](../examples/shared-session.md) for a full
example, or [First task in simulation](../examples/first-task.md) for the
agent's original terminal.

`openrua --help` lists commands and startup options; [cli.md](cli.md) has every
flag. The old `[tui]` extra remains accepted for compatibility but is no longer
required. Rendering recorded demo videos still needs the `demo` extra:
`pip install 'openrua[demo]'`.

## The images

OpenRUA runs three containers: the robot (when simulated), the sandbox
(the agent's terminal: `ros2`, `rclpy`, the docs, the agent's CLI) and
the proxy (the sandbox's only route out, limited to the agent's model
API). **Prepare and start** in the product TUI automatically builds missing
images for the selected environment and agent. Root `--cli` and `--gui` launches
prepare them too. Browsing options or choosing **Save & check** never builds.
First-time simulation builds can download large datasets. Progress is displayed,
and logs are retained under `~/.openrua/preparation/` for retry.

For manual preparation, or the explicit `up`, `run` and `serve` commands (which
still require prepared images), use:

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

For the native login commands below, the corresponding agent CLI must also
be installed on the host. The sandbox build installs its own copy for execution;
it does not install the host command. Use the CLI's normal interactive login.

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

The readiness check verifies daemon access and that the selected credential
file is readable, nonempty, and a regular file. It does not contact a model API,
validate a token, or check quota. Authentication errors still come from the native
agent; repeat its login command when it reports an expired or invalid login.
A Docker CLI on PATH is insufficient if its daemon is stopped or inaccessible.
`doctor` retains the daemon error and reports the repair step.

## Your defaults

Use `openrua --setup` to edit defaults through the TUI, `openrua config set`
to update selected fields from a shell, or edit `~/.openrua/config.yaml`
directly. These are three interfaces to one file. For example:

```sh
openrua config set --robot panda --sim robosuite --agent codex
openrua config show
```

`config set --bench null` clears a saved benchmark selection. Changing defaults
affects subsequent starts; it does not reconfigure a running conversation.

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

`~/.openrua` (`$OPENRUA_HOME`, `--home`) holds your defaults in
`config.yaml`, logins in `credentials/`, and session files in
`sandboxes/<name>/`. Ending `run`, interrupting `up`, or calling `down`
stops the session's resources but retains its workspace, native agent
profile (including any conversation records written by the agent), logs
and state. This does not resume a stopped robot or agent conversation. A shared service
started by `openrua` runs independently of the client: use **End session**, or
`openrua session --name NAME end`, to stop it explicitly.

Startup output is retained in `launches/<name>/service.log`, including errors
before a session becomes ready. These startup diagnostics are separate from
the workspace and native agent logs under `sandboxes/<name>/`.

Run `openrua` for a new automatically named session while retaining the earlier
one. Explicit `--name` values must not overwrite retained work. An
existing session directory or nonempty `--workspace` is never overwritten.
The default workspace is `sandboxes/<name>/workspace/workspace/`;
`--workspace <directory>` places it at `<directory>/workspace/` instead.

Deletion is explicit: `openrua clean --name <name>` deletes one stopped
session; `openrua clean` deletes all stopped sessions. Adding `--all` also
stops eligible running containers before deleting their files. Sessions with a
resource-owner endpoint must be ended with `down` first; real sessions without
a confirmed shutdown are retained. Check your saved
work before using these commands. They never delete external workspace
directories, `config.yaml`, or `credentials/`. Benchmark trial directories
and their fresh-workspace protocol are unaffected.

## Check it

```bash
openrua doctor panda --bench libero_pro
```

Read-only. Each selected agent is checked against its own configured login
directory and version, including when `--agent` selects a different agent from
the benchmark default. Repeat `--agent` to check several agents. One row per fact: the container engine, each image and
whether its label still matches the manifests (sandbox, proxy) or the
declaration it was rendered from (the simulator image), the agent
login, the user directory. The standing proxy container is checked separately:
rebuilding its image does not update its running policy. Error and warning rows
include repair guidance when available; the exit code is 1 only when an error is present, and on a pipe
the report is JSON.

### Updating a shared proxy

The proxy is shared by sessions. If `doctor` reports a manifest mismatch,
rebuild the sandbox or proxy with the agents you need, repeating `--agent` to
include more than one. For example:

```sh
openrua build proxy --agent claude-code --agent codex
```

A `proxy-container` warning can remain after this build because the existing
container still uses its original image. End all sessions that use that proxy
before removing it with `docker rm -f openrua-proxy`. Starting a new session then
creates the proxy from the rebuilt image. OpenRUA never replaces a standing
proxy automatically, since doing so would disconnect other sessions.

If proxy network attachment fails, startup reports the Docker error instead
of returning an unreachable address. Restore the Docker network or daemon
access described in the error, then retry; the failure does not stop existing
sessions.

## Update or remove

```bash
pip install -U openrua                   # then rebuild: a simulator image carries the package too
openrua build --all                      # rebuild every simulator image (cached layers make it quick)
pip uninstall openrua
rm -r ~/.openrua                         # defaults, logins, sandboxes
docker image rm $(docker image ls -q 'openrua-*')
```

## Optional Pi terminal

For the keyboard-first Pi frontend, use `pip install -U 'openrua[pi]>=0.4.1'`
and `openrua --tui pi`. Pip installs the required runtime; a system Node/npm
installation is unnecessary. See the [terminal guide](terminal.md#pi-terminal-optional).
This path is verified on Linux x86_64; the existing default frontend remains
included in the ordinary installation.
