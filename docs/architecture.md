---
summary: The units of OpenRUA, what each owns, and who may import whom
read_when:
  - You are changing how the units fit together or adding one
  - You need to know where a fact lives (paths, schema, errors, agents)
  - A layering test or import-linter contract failed
---

# Architecture

OpenRUA has one job: put a coding agent's terminal on a robot's native
ROS 2 surface. Every unit below exists to set that table; none of them
is visible to the agent.

## Units

| Unit | Role | Front door |
|---|---|---|
| `openrua/robot/` | the machine, provided by a backend (`up()` dispatches on `machine.backend.kind`). `sim/`: image build, container up/down, the host-side client, and `bridge/`, the simulated robot's own software (`environments/`, `ros/`, `rpc.py`, `main.py`). `real/`: an optional launch command (on the host or in a driver image), a handle that waits for the graph, and `probe.py` (a profile draft from the graph). | `python -m openrua.robot.sim.build` |
| `openrua/sandbox/` | the agent's terminal: an Ubuntu + ROS 2 container with the agent installed and `workspace/` seeded (README, `machine.yaml`, four docs, a few tools). | `python -m openrua.sandbox` |
| `openrua/proxy/` | a whitelist HTTP proxy, the sandbox's only route out. | `python -m openrua.proxy` |
| `openrua/agents/` | `base.Agent` (the contract), the registry (manifests under `configs/agents/`, hooks under `plugins/agents/`, bundled then `~/.openrua/`), the launcher, credentials staging, the prompts. | `python -m openrua.agents launch` |
| `openrua/runner/` | running trials: `main.py` (`openrua run`), `bringup.py` (one resolved config to sandbox + robot), `trial.py`, `operators.py`, `session.py` (the agent operator across segments), `preflight.py` (every promise the workspace docs make, checked before the agent starts), `record.py` (the only writer under `runs/`), `lock.py`. | `openrua run` |
| `openrua/demo/` | a video from a recorded trial's files (`frames/`, `ops.jsonl`): `compose.py` renders the terminal beside the cameras. Reads files, imports `errors` only; its libraries are the `demo` extra. | `openrua demo` |
| `openrua/cli/` | the command line: one module per verb under `commands/` (`robots / benchmarks / agents / build / up / agent / down / run / demo / probe / config / doctor`), `output.py`, `state.py`. | `openrua` |
| `openrua/doctor/` | is this machine ready: `checks.py` (docker, images and their labels against the selected agents' manifests, simulator, login, the user directory), `report.py`. | `openrua doctor` |

Shared leaves, importable by every host-side unit and by nothing in
the bridge:

| Leaf | Owns |
|---|---|
| `openrua/config/paths.py` | where things live: the user directory (`~/.openrua`), the bundled data (`openrua/configs/`, `openrua/plugins/`), the lookup order (bundled, then user, then a path), simulator and workspace locations. |
| `openrua/config/schema.py` | the schema (pydantic): robot profile, benchmark config, user config, the assembled per-trial config; defaults and a description per key; unknown keys are errors. |
| `openrua/errors.py` | the error family: message, hint, sysexits code. The CLI entry point is the one place an error becomes text. |
| `openrua/testing.py` | `check_manifest` and `check_agent`, the conformance tests third parties run. |

Data, not code, is what crosses unit boundaries: a robot profile and a
benchmark config are validated and assembled once into one config,
written to disk (`config.yaml`), and read by every party (the sandbox
seeds the workspace from it, the robot's bridge starts from it,
preflight derives its checks from it). At runtime the units talk over DDS, stdio, and
files under `runs/`.

## Where things live

```
openrua/configs/{robots,benchmarks,agents}/, openrua/plugins/agents/   bundled, ship in the wheel
~/.openrua/                                             the user directory ($OPENRUA_HOME, --home)
  config.yaml        your defaults: agent section, default robot
  robots/ benchmarks/ agents/ plugins/agents/            yours, looked up after the bundled ones
  credentials/<agent>/                                   login profiles
  simulators/<name>/.venv-*                              simulator checkouts
  workspaces/<name>/  state/<name>.yaml                  what `openrua up` keeps
./runs/                                                 trial data (--runs-root)
```

## The layering contract

- Nothing host-side imports `openrua.robot.sim.bridge` (it lives in the
  container, with rclpy).
- Inside the bridge, `environments/`, `ros/` and `rpc.py` never import
  each other; `main.py` wires them with parameters.
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
- Only the runner (`runner/bringup.py`, used by `openrua run` and `openrua up`) brings robots up.
- `demo` reads a trial directory and imports `errors` only; recording
  itself is the bridge's (a `Recording` handed to its monitor) and is
  switched on by `openrua run --record`, never by the demo unit.
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
profile from the graph. The agent cannot tell the difference, which is
the point.
