---
summary: The units of OpenRUA, what each owns, and who may import whom
read_when:
  - You are changing how the units fit together or adding one
  - You need to know where a fact lives (paths, schema, errors, agents)
  - A layering test or import-linter contract failed
---

# Architecture

OpenRUA gives a native coding agent a workspace and access to the robot's
ROS 2 interfaces. The product manages resources and user interaction; the agent
chooses the robot observations, programs, and actions.

Three entry paths share these foundations:

| Path | Composition | Lifetime |
|---|---|---|
| Shared chat | default startup → background `serve` → `runner.managed` → live resources, native conversation, session execution; TUI, browser, and plain CLI use its HTTP API | The service owns execution; closing a client only detaches |
| Native terminal | `run`, or `up` then `agent`, use `runner.live` and the selected agent's native terminal | `run` ends its resources when the agent exits; `up` retains them until shutdown |
| Benchmark | `bench` → trial runner → fresh workspace, operator, preflight, scoring, records | The trial protocol controls budgets and teardown |

Shared chat does not replace the native agent loop or the benchmark protocol.
Its queue orders user instructions, not individual robot movements.

## Units

| Unit | Role | Front door |
|---|---|---|
| `openrua/robot/` | the machine, provided by a backend (`up()` dispatches on `machine.backend.kind`). `sim/`: image build, container up/down, the host-side client, and `bridge/`, the simulated robot's own software (`environments/`, `engines/`, `ros/`, `rpc.py`, `main.py`). `real/`: an optional launch command (on the host or in a driver image), a handle that waits for the graph, and `probe.py` (a profile draft from the graph). | `python -m openrua.robot.sim.build` |
| `openrua/sandbox/` | the agent's terminal: an Ubuntu + ROS 2 container with the agent installed and `workspace/` seeded (README, `machine.yaml`, four docs, a few tools). | `python -m openrua.sandbox` |
| `openrua/proxy/` | a whitelist HTTP proxy, the sandbox's only route out. | `python -m openrua.proxy` |
| `openrua/agents/` | `base.Agent` (the contract), the registry (manifests under `configs/agents/`, the module each names under `entry_point`), the launcher, credentials staging, the prompts. | `python -m openrua.agents launch` |
| `openrua/runner/` | composition and ownership: `live.py` opens robot resources, `live_state.py` retains their state, `managed.py` adds a native conversation and shared execution owner. Trial execution stays separate: `main.py` (`openrua bench`), `bringup.py` (one resolved config to sandbox + robot), `trial.py`, `operators.py`, `session.py` (the agent operator across segments), `preflight.py` (every promise the workspace docs make, checked before the agent starts), `record.py` (the only writer under `runs/`), `lock.py`. | `openrua serve`, `run`, `up`, `bench` |
| `openrua/demo/` | a video from a recorded trial's files (`frames/`, `ops.jsonl`): `compose.py` renders the terminal beside the cameras. Reads files, imports `errors` only; its libraries are the `demo` extra. | `openrua demo` |
| `openrua/sessions/` | shared user-message queue, durable events and native connection ownership through injected storage/transport contracts; independent of robot task planning. See [sessions.md](sessions.md). | `serve`, `chat`, `session` (experimental) |
| `openrua/tui/` | setup form with injected configuration/readiness/launch callbacks; Textual chat client, expandable tool output and queue controls. Imports the common session client, not execution or vendor implementations. | `openrua chat --tui` |
| `openrua/web/` | bundled browser assets, supplied to the HTTP server by its caller. The browser consumes the session API. | `openrua session --name NAME web` |
| `openrua/artifacts.py` | bounded, read-only workspace file access; the owner supplies the root and reader to the HTTP server. | file operations in the session API |
| `openrua/terminal/` and `ui/terminal/` | Pi launcher and frontend. The launcher receives configuration/history operations as callbacks; chat consumes the same local session API. Private temporary files carry screen input/output. Prepared JS resources ship in the wheel; the optional runtime is a pip dependency. | `openrua --tui pi` |
| `openrua/cli/` | the command line: one module per verb under `commands/` including discovery, builds, native sessions, shared chat, trials, and cleanup. `output.py` formats output; `state.py` retains compatibility imports for resource state. | `openrua` |
| `openrua/doctor/` | is this machine ready: `checks.py` (docker, images and their labels against the selected agents' manifests, simulator, login, the user directory), `report.py`. | `openrua doctor` |

Shared support modules, used only by the layers permitted by the import
contracts; none is imported by the simulator bridge:

| Leaf | Owns |
|---|---|
| `openrua/config/paths.py` | where things live: the bundled data (`openrua/configs/`, `openrua/plugins/`), how a name resolves (a bundled name, else a path), how an `entry_point` resolves (a bundled module, else a file next to the yaml), the user directory (`~/.openrua`) and what the tool keeps there. |
| `openrua/config/schema.py` | the schema (pydantic): robot type and instance, simulator, benchmark, user config, the assembled per-trial config; defaults and a description per key; unknown keys are errors. `loader.compose` folds robot, simulator and benchmark (benchmark -> simulator -> robot, one direction) into the one `machine:` dict every unit reads. |
| `openrua/errors.py` | the error family: message, hint, sysexits code. The CLI entry point is the one place an error becomes text. |
| `openrua/testing.py` | `check_manifest` and `check_agent`, the conformance tests third parties run. |

Resolved configuration and explicit contracts connect the units. Robot and
simulator components receive validated data; the sandbox seeds its workspace
from that data. Trial preflight derives its checks from the same configuration.
Native agents communicate through their plugin transports, while the robot
exposes ROS 2 interfaces. Shared clients use the local HTTP API and durable
session events; benchmark artifacts remain under `runs/`.

## Where things live

```
openrua/configs/{robots,simulators,benchmarks,agents}/   bundled declarations, ship in the wheel
openrua/plugins/agents/, openrua/robot/sim/bridge/{environments,engines}/   the code bundled entry_points name
openrua-sim-<name>, openrua-sandbox-<distro>, openrua-proxy   the images (openrua build): a simulator's whole environment, the agent's terminal, the proxy
~/.openrua/                                             the user directory ($OPENRUA_HOME, --home); written by the tool, not by hand
  config.yaml                                            your defaults (openrua config set)
  credentials/<agent>/                                   login profiles
  sandboxes/<name>/                                      session workspace, profile copy and state; retained until explicitly deleted with openrua clean
./runs/                                                 trial data (--runs-root)
```

A file of your own (a robot, a simulator, a benchmark, an agent
manifest) is passed as a path where a name is expected; code it needs
is named under `entry_point` relative to it. The repository is where
shared ones go.

## The layering contract

- Nothing host-side imports `openrua.robot.sim.bridge` (it lives in the
  container, with rclpy).
- Inside the bridge, `environments/`, `engines/`, `ros/` and `rpc.py`
  never import each other; `main.py` wires them with parameters. A
  physics engine is named in `engines/` and nowhere else: the ROS side
  and the control line read joints, poses and cameras and assemble
  actions through the bound engine (`ENGINE_INTERFACE`), the way the
  monitor scores through the loader (`LOADER_INTERFACE`).
- The bridge imports nothing from openrua outside itself, the shared
  leaves included: it can be installed on its own inside the container
  and reads absolute paths and validated dicts from the resolved config
  file.
- The robot's host side consumes data only: no shared leaf, never the
  bridge.
- `sandbox` imports no other layer: what the agent experiences knows
  nothing about scoring.
- `preflight` and `record` are leaves; handles and paths are handed in.
- `agents` and `proxy` are leaves (they may use `paths` and `errors`).
- The schema (`config`) is read by the runner, the cli and the agents
  registry; the other units read validated dicts and never import it.
- Only the runner composes robot startup: `bringup.py` is shared by trials
  and `live.py`; `managed.py` adds shared conversation ownership.
- `sessions` depends on agent contracts and injected storage/transport, never
  robot resources, vendor plugins, configuration, or CLI code. Agent plugins
  do not import session coordination.
- `tui` consumes `sessions.client`; it does not import execution internals.
  `web` supplies static assets and is not imported by session coordination.
- Workspace artifact readers are independent leaves. Their root and limits
  are passed in by the resource owner, not discovered through global state.
- `demo` reads a trial directory and imports `errors` only; recording
  itself is the bridge's (a `Recording` handed to its monitor) and is
  switched on by `openrua bench --record`, never by the demo unit.
- `cli`, `doctor` and `testing` sit above the units; no unit imports them.

The contract is stated in `pyproject.toml` (import-linter) and again in
`tests/architecture/test_layering.py` (AST-checked, no dependency). CI
runs both. A third test, `tests/agents/test_agent_boundary.py`, keeps
every agent-specific token inside the manifests and hooks modules.

## Why a real robot needs no bridge

`up` for a simulated robot (`backend.kind: sim`) starts a container that
runs the bridge: the simulator, a ROS 2 graph over it, and the control
line the runner scores through. For a real robot (`backend.kind: real`)
that graph already exists; `up` runs the profile's launch command if it
has one (on the host, or inside `backend.image` on the host network),
starts the sandbox on the host network pointed at the graph the way
`backend.discovery` says, waits until the sandbox sees a node, and seeds
the workspace from the profile. The handle's rpc answers not applicable,
so a trial records no verdict on hardware. `openrua probe` drafts the
profile from the graph. Both paths expose ROS 2 to the agent; this interface
choice alone does not establish real-hardware reliability. See
[your-own-robot.md](your-own-robot.md) for setup and validation boundaries.

## Interactive resource ownership

`runner.live.open_robot(RobotRequest(...))` creates a live robot and sandbox
through the existing `bring_up` path and returns a `LiveRobot` handle. The
caller retains this handle and calls `power_off()` when it ends the resource
session. `up` and `run` translate their CLI arguments into this same request;
service hosts can use it without importing the CLI. Terminal formatting stays
in the CLI.

`runner.live_state` owns the retained resource facts. The old `cli.state`
imports remain available for callers. Resource state is separate from the
conversation queue in `sessions`: closing a client is neither a robot shutdown
nor a conversation deletion. Shutdown attempts both sandbox and machine and
marks the resource session stopped only if both calls succeed. A failure to
save resource facts after startup also triggers resource cleanup.

`runner.managed` combines a live robot with the plugin's native conversation
and the session execution owner. The `serve` CLI supplies a local HTTP front
end; `chat`, the TUI, `session`, and the browser consume that API.
Browser assets live in `openrua.web`, a leaf supplied explicitly to the HTTP
adapter; session coordination never imports it. HTTP connection loss does not close
the execution owner. `down` routes managed sessions through their owner, and
`clean` refuses to delete a directory while its service endpoint remains.
Agent-specific frames and launch commands remain in the existing agent plugins.

For queue semantics, native transport details, and validated failure cases,
see [Shared robot sessions](sessions.md). Client reconnection is supported;
reopening a client does not recover a crashed execution service or restart a
stopped native agent. `serve` remains a foreground host for debugging. The default startup entry
launches that same command in a detached process using `runner.service`, then
checks its session API before opening a client. Failed or uncertain startup
does not automatically replay work. `config.settings` supplies a validated,
atomic update shared by CLI and TUI configuration; direct file edits remain
supported and are read at subsequent starts.

Session history is a read-only discovery adapter in `runner/history.py`, using
the existing conversation journal and endpoint handshake. IDs are generated
for unnamed product launches; titles derive from the first instruction. Legacy
named directories remain valid without migration. The TUI history picker
receives listing and opening callbacks, and never imports storage or execution
modules. Opening retained records cannot restart an owner or issue commands.
