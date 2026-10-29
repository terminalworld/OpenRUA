---
summary: Requirements, Docker, installing the package, your agent's login, building the images, and the doctor that checks it all
read_when:
  - You are setting OpenRUA up on a machine for the first time
  - openrua doctor is red and you want to know what a row means
  - You are updating or removing an install
---

# Install

## Requirements

| | |
|---|---|
| OS | Linux, with [Docker](#docker) (or rootless Podman providing the `docker` command: [podman.md](podman.md)) |
| Python | 3.10 or newer on the host; everything else (ROS 2, the simulators, the agent CLIs) lives in the images |
| Agent | [Claude Code or Codex, signed in](#your-agents-login): a Claude or ChatGPT subscription, or an API key |

## Docker

OpenRUA runs the robot, the simulator and your agent's terminal in Docker
containers: they stay apart from each other and from the rest of the
computer, and the same images run on any Linux machine. A real robot's
drivers run in containers too.

When Docker is missing, your user may not use it yet, or its service is
stopped, `openrua` says so when you start a session or connect a robot,
and offers the fix: Docker's own
install script ([get.docker.com](https://get.docker.com)), joining the
`docker` group, or starting the service. It shows the commands first and runs
them only if you agree; sudo asks for your password. `openrua connect` and
`openrua login` do the same. To set it up yourself, follow
[Docker's install guide](https://docs.docker.com/engine/install/) and its
[post-install steps](https://docs.docker.com/engine/install/linux-postinstall/)
(Docker as a non-root user).

## The package

```bash
pip install -U openrua
openrua --version
```

The default install includes the terminal UI. Run `openrua` to configure and
start a new shared session with an automatic ID. Use `openrua --resume` to find
a previous conversation, or `/resume` inside the TUI. Use
`openrua --gui` for the browser.
Guided setup saves the environment selection in the project's
`.openrua/settings.json`. See [robot projects](projects.md) for settings and file locations.
See [Your first shared robot session](../examples/shared-session.md) for a full
example, or [First task in simulation](../examples/first-task.md) for one task
with `openrua exec` and the agent's original terminal.

`openrua --help` lists commands and startup options; [cli.md](cli.md) has every
flag. The old `[tui]` extra remains accepted for compatibility but is no longer
required. Rendering recorded demo videos still needs the `demo` extra:
`pip install 'openrua[demo]'`.

## Your agent's login

Your coding agent does the work, so OpenRUA needs your Claude Code or Codex
account: a Claude or ChatGPT subscription, or an API key. Every session's
agent uses the one login stored on this computer.

Already signed in on this computer (`~/.claude` or `~/.codex`)? OpenRUA uses
that login. If not, `openrua` offers to sign you in when you start a session
or connect a robot, or you run it yourself:

```bash
openrua login           # the configured agent
openrua login codex     # or the one you name
```

The agent's own sign-in runs inside OpenRUA's sandbox image, so its CLI need
not be installed on this computer: open the link it shows in any browser,
then paste the code back (Claude Code) or enter the code it shows on the page
(Codex). With the CLI installed here, its own sign-in does the same
(`claude auth login`, `codex login`). To bill an API key instead, choose it in
the setup menu or see [explicit API authentication](#explicit-api-authentication);
OpenRUA never switches to API billing after a login error.

OpenRUA reuses existing file-based native logins. With no explicit profile
selection, it discovers `~/.claude` or `~/.codex` and, when the OpenRUA account
path is absent, links `~/.openrua/credentials/<agent>` to that directory at
startup. OAuth credential files are not copied. Doctor is read-only and shows
the actual source selected for the sandbox.

Precedence is explicit `credentials_dir`, the agent's native environment
variable (`CLAUDE_CONFIG_DIR` or `CODEX_HOME`), an existing OpenRUA profile,
then the native default directory. Existing directories and symlinks are never
overwritten or silently switched to another account. To clear an obsolete
explicit path, use `openrua config set --user --agent claude-code --credentials-dir null`
(or `--agent codex`). Already-running sandboxes retain their selected login;
start a new session to use a changed source. If native re-login replaces the
credential file, also start a new sandbox so its file mount uses the new file.

Logins held only in an OS keychain cannot be discovered as files by a Linux
sandbox. Use the native CLI's supported file-based login on the execution host;
see [Claude authentication](https://code.claude.com/docs/en/authentication) and
[Codex authentication](https://developers.openai.com/codex/auth/).
An expired, unrefreshable native login requires signing in again. Discovering a
file does not repair revoked tokens. OpenRUA does not silently switch accounts
or retry failed robot instructions after authentication failure.

## The images

A session uses an agent sandbox with `ros2`, `rclpy`, documentation and the
agent CLI. A simulated robot runs in its own container. OpenRUA-managed
real-robot drivers run in the containers declared by the robot configuration;
the number depends on that configuration. If you connect to an existing robot service, stopping the OpenRUA session
does not stop that service.

When a recorded run limits the agent to its model service, a proxy is its
route out and, for a simulated robot, the simulator's only route out too.
See [what the agent can reach](#what-the-agent-can-reach).

**Save project and start** in the product TUI automatically builds missing
images for the selected environment and agent. `openrua --gui` launches
prepare them too. Browsing options or choosing **Save project settings and check setup**
never builds.
First-time simulation builds can download large datasets. Progress is displayed,
and logs are retained under `~/.openrua/preparation/` for retry.

For manual preparation, or the explicit `up`, `exec`, `bench` and `serve`
commands (which still require prepared images), use:

```bash
openrua build                            # the default agent's sandbox, the proxy image, and the default benchmark's simulator image
openrua build --bench libero_pro         # one simulator image; --bench / --sim repeatable, --all: every bundled benchmark
openrua build sandbox --agent codex      # --distro jazzy (default) | humble; --agent claude-code (default), repeatable
openrua build agent --agent codex        # CLI tools for model lookup, without ROS or robot dependencies
openrua build proxy                      # one image for every agent: each proxy gets its hosts when it starts
openrua build driver --robot ./my-robot.yaml   # a real robot's driver images (backend.build)
```

Setup's model picker reuses a compatible installed agent image, preferring newer
builds that match the selected CLI recipe, version and filesystem user. If none
is available, it prepares the small CLI image automatically. `build agent` also
lets you rebuild those tools explicitly; `--agent NAME@VERSION` pins the CLI.
The complete robot environment is prepared when you start a session.

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

The sandbox image is named after the ROS 2 distro
(`openrua-sandbox-humble`), which a robot's declaration names once as
`ros_distro`; it takes the agent's install line from its manifest and
carries a label per agent with the hash of what went in. A simulator
image carries the fingerprint of the declaration it was rendered from.
Those labels are how `doctor` later knows whether an image is stale.
The proxy image holds no whitelist.

An unpinned manifest lets `build sandbox` install the agent's
release current at build time (it never reuses an older build's layer
for that), and prints the version each CLI got. To pin one, say so where
you build and where you run: `openrua build sandbox --agent
claude-code@2.1.301` and `agent.version: "2.1.301"` in the config;
`doctor` then suggests the pinned rebuild. A manifest may also name the
oldest release that takes every flag its adapter passes (`min_version`;
Claude Code's is 2.1.251): a build that produces an older one fails, and
a session or trial on a sandbox with an older one is refused before the
agent starts, with the rebuild command. Every trial records the version
it saw.

A robot whose agent needs interface packages beyond ROS's own (a vendor's
message definitions, the ros2 control CLI) declares the install in its
config (`backend.sandbox_install`) and gets a sandbox image of its own:
`openrua build sandbox --robot ./my-robot.yaml`. A real robot whose
launch command runs its driver in images of its own names them with the
directories they are built from (`backend.build`); `openrua build driver
--robot ./my-robot.yaml` builds them. **Save project and start** builds both
when they are missing. See [your-own-robot.md](your-own-robot.md).

In the sandbox, `$HOME` is `/workspace`, so whatever the agent writes
under `~` stays in the session's record. The logs and caches tools write
on their own (ROS logs, the pip and npm caches) go to `/home/robot`
through `ROS_HOME`, `XDG_CACHE_HOME` and `npm_config_cache`.

## What the agent can reach

Three limits say what the agent may reach beyond the robot:

| limit | values | what it governs |
|---|---|---|
| `internet` | `open` \| `model-only` | the whole internet, or only the agent's model service and login, through a whitelist proxy |
| `web_search` | `true` \| `false` | the agent CLI's own web search and web fetch tools |
| `account_connectors` | `true` \| `false` | what your account brings beyond the CLI: its connectors or apps, and the skills and plugins it syncs |

A session you start yourself (`openrua`, `up`, `serve`) sets none
of them: the agent CLI behaves as you have it set up, no proxy starts,
and the sandbox reaches the internet directly (a simulated robot's
sandbox also joins `openrua-internet`, a network only OpenRUA's
sandboxes are on, so no other container shares its ROS 2 traffic; a real
robot's sandbox is on the host network). Turn one on for a session with
`--internet model-only`, `--no-web-search` or `--no-account-connectors`.

A recorded run (`openrua exec`, or a benchmark trial under `openrua bench`)
applies its config's `protocol.limits`, by default all three: the agent
reaches its model service and nothing else. A benchmark config may change
them, and the same flags on `openrua exec` or `openrua bench` override them
for a run (`--internet open`, `--web-search`, `--account-connectors`). The run's `config.json` and each
trial's record say which applied; a session's `provenance.json` records
its limits and the agent CLI version the sandbox reported.

With `internet: model-only`, OpenRUA starts one standing proxy per
whitelist, named after the hash of its hosts (`openrua-proxy-<hash>`),
with the hosts the agent's manifest lists. A different agent gets its own
proxy; no rebuild is needed and a proxy in use is never replaced.

## Explicit API authentication

Claude Code and Codex support separate API profiles for ordinary terminal chat,
native terminal sessions, and benchmark trials.
In `openrua --setup`, choose **Authentication → API key file**, then enter the
private key file's path. **Native CLI login** remains the default and can be
selected again in the same menu. The CLI alternative is below.
Create a private file outside your workspace and repository containing only
your Anthropic or OpenAI API key, then select it explicitly:

```bash
# Claude Code
chmod 600 /path/to/anthropic.key
openrua config set --user --agent claude-code --auth api --api-key-file /path/to/anthropic.key
openrua doctor --agent claude-code
openrua

# Or Codex
chmod 600 /path/to/openai.key
openrua config set --user --agent codex --auth api --api-key-file /path/to/openai.key
openrua doctor --agent codex
openrua
```

This selection uses API billing for new sessions and trials of the selected agent.
It does not change an already running session or your normal native CLI login. The default is
`native`, which reuses the login you selected in the native CLI; merely saving
a key file never enables API use. To return to that login:

```bash
openrua config set --user --agent claude-code --auth native
# Or: openrua config set --user --agent codex --auth native
```

The equivalent configuration is `agents.claude-code.auth: {mode: api, key_file:
/absolute/path/to/anthropic.key}` (or `agents.codex.auth` for Codex).
Configuration stores the path, not the key.
API mode creates a private, session-local native profile without mounting the
subscription credentials. Live session profiles are retained with the other
session files until explicit deletion; do not publish them. Trial profiles
are removed after finalization, and their key is supplied to archive redaction.
An invalid key fails through the native CLI, without falling back to subscription
login. Local checks establish readability and format, not account validity or quota.

Claude Code uses its native `apiKeyHelper` setting to read the private key file;
Codex uses its native API login profile. See [Claude Code authentication](https://code.claude.com/docs/en/authentication)
and [Codex authentication](https://developers.openai.com/codex/auth/). This configures
direct provider API access, not a third-party gateway. Unsupported plugins reject
`--auth api` before changing defaults. Bundled preview integrations for
[Kimi Code](agents/kimi.md) and
[ZCode](agents/zcode.md) document their own explicit API
configuration and validation scope; both are available in setup.

`openrua doctor` prints the command for whichever agent it finds
logged out. The other route is a token by file: `openrua bench
--token-file <path>` names a one-line `KEY=value` file (for Claude Code
a `claude setup-token` value as `CLAUDE_CODE_OAUTH_TOKEN`, for Codex an
`OPENAI_API_KEY`) that docker hands to the agent process only. The
token is scrubbed from the trial record like every other secret.
Do not combine `--token-file` or account overrides with `auth.mode: api`;
choose one authentication source.

The readiness check verifies daemon access and that the selected credential
file is readable, nonempty, and a regular file. It does not contact a model API,
validate a token, or check quota. Authentication errors still come from the native
agent; repeat its login command when it reports an expired or invalid login.
A Docker CLI on PATH is insufficient if its daemon is stopped or inaccessible.
`doctor` retains the daemon error and reports the repair step.

## Project settings and personal preferences

Use `openrua --setup` or `openrua config set` in your project to save its
robot, simulator, benchmark, agent and model selection. The setup screen
and CLI write `.openrua/settings.json` in that project. For example:

```sh
openrua config set --robot panda --sim robosuite --agent codex
openrua config show
```

Selected simulation declarations and build files are copied into project
`.openrua/configs/`. Project references are relative to its root. A normal launch's
flags apply only to that run. `config set --bench null` explicitly saves a
choice of the simulator's native scene. See [projects](projects.md).

`openrua config set --user` edits personal agent preferences and authentication
in `~/.openrua/config.yaml`. The user file also accepts sandbox options such as
rootless Podman's `--userns=keep-id`. Robot, simulator and benchmark defaults
belong to the project and are not accepted in this file.
`openrua config schema --user` describes the personal settings, including when
the current file is invalid. Fix the named file before restarting; the accepted
top-level keys are `agent`, `agents` and `sandbox`.

```sh
openrua config set --user --agent codex --model MODEL
openrua config show --user
```

Personal model, CLI version and login-directory facts live under
`agents.<name>`. A benchmark's authored agent facts override personal defaults
for that agent; explicit project model and command-line choices are applied
by the project entry point. See [config.md](config.md#userconfig).

## The user directory

`~/.openrua` (`$OPENRUA_HOME`, `--home`) holds your defaults in
`config.yaml`, logins in `credentials/`, and session files in
`sandboxes/<name>/` and `launches/<name>/`. Interrupting `up`, or calling `down`,
stops the session's resources but retains its workspace, native agent
profile (including any conversation records written by the agent), logs
and state. Project files remain in your chosen directory. A shared service
started by `openrua` runs independently of the client: end it with `/end` in
the terminal, **End session** in the browser, the end choice when you leave
the terminal, or `openrua session --name NAME end`.

Startup output is retained in `launches/<name>/service.log`, including errors
before a session becomes ready. These startup diagnostics are separate from
native agent logs under `sandboxes/<name>/`. The current project has its own
environment choices and per-run robot logs; see [robot projects](projects.md).

Run `openrua` for a new automatically named session while retaining the earlier
one. Explicit `--name` values must not overwrite retained work. An
existing session directory is never overwritten. The default workspace is
the current directory itself; `--workspace <directory>` selects another
persistent project. Its existing files remain intact. First use asks whether
you trust the directory. `--resume` lists the current project's conversations;
`--resume --all` includes every project and older records. Confirmed ended
project conversations can reconnect to fresh resources without replaying work.

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
whether its label still matches the manifests (the sandbox) or the
declaration it was rendered from (the simulator image), a real robot's
driver images and each of its startup checks (`backend.startup_checks`, a
failed one an error with its fix), the agent login, the user directory. A
missing proxy image
is a warning, not an error: only runs limited to the model service use it.
Error and warning rows include repair guidance when available; the exit
code is 1 only when an error is present, and on a pipe the report is JSON.

If a proxy cannot join the session's network, startup reports the Docker
error instead of returning an unreachable address. Restore the Docker
network or daemon access described in the error, then retry; the failure
does not stop existing sessions.

## Update or remove

`openrua` checks PyPI for a newer OpenRUA at most once a day and, when
there is one, asks before the session starts: update now, skip, or skip
until the next version. Updating runs the upgrade your installer uses
(`pip install -U openrua`, `uv tool upgrade openrua` or `pipx upgrade
openrua`); run `openrua` again to start the new version. `openrua up`
and `openrua exec` print one line instead of asking. An install from a
source checkout is never offered one: `git pull` updates it.

Agent updates change only the selected CLI in an existing image. Compatible
runtime dependencies, robot tools and other agents remain in place; a missing
prerequisite is prepared through that plugin's declared check and command.
The candidate must pass its native version check before replacing the image tag.
Running sessions retain their current environment.

The fullscreen update page offers **Update now**, **Continue with current
version**, and **Skip this version**. Progress is compact; **L** reveals live
logs, and **View update log** opens the retained output after completion or
failure. Retrying requires an explicit selection. Optional failures allow the
current version; an unsupported CLI still requires an update or exit. Browser launches retain text update prompts in the invoking terminal. Noninteractive commands print a
notice and never update automatically. Pinned plugins are not offered a newer,
unvalidated release.

For an explicit component update, select its release and existing tool image:

```sh
openrua update --agent codex@0.164.0 --image openrua-sandbox-jazzy
```

The same operation accepts a project manifest with `@version`. This does not
rebuild simulator or driver images. See [plugin preparation](agents.md#installing-and-updating-a-component).

```bash
pip install -U openrua                   # next launch prepares the selected project when needed
pip uninstall openrua
rm -r ~/.openrua                         # personal preferences, logins and retained session state
docker image rm $(docker image ls -q 'openrua-*')
```

## Keyboard terminal

The ordinary install includes the Pi terminal and its runtime:
`pip install -U openrua`, then `openrua`. No separate Node/npm install is needed.
`[pi]` and `[tui]` remain empty compatibility extras. Textual has been removed.
See the [keyboard guide](terminal.md). Linux x86_64 is the tested distribution path.
