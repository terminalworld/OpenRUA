---
summary: The units of RoboCLI, what each owns, and who may import whom
read_when:
  - You are changing how the units fit together or adding one
  - You need to know where a fact lives (paths, schema, errors, agents)
  - A layering test or import-linter contract failed
---

# Architecture

RoboCLI has one job: put a coding agent's terminal on a robot's native
ROS 2 surface. Every unit below exists to set that table; none of them
is visible to the agent.

## Units

| Unit | Role | Front door |
|---|---|---|
| `robocli/robot/` | the machine. Ground side (`build`, `up`, `down`) runs on the host and is all a real robot needs. `onboard/` is the simulated robot's own software: `environment/` (the simulator), `ros_graph/` (joints, arms, sensors, clock, controllers, MoveIt), `monitor/` (the control line), `boot.py`. | `python -m robocli.robot` |
| `robocli/sandbox/` | the agent's terminal: an Ubuntu + ROS 2 container with the agent installed and `workspace/` seeded (README, `machine.yaml`, four docs, a few tools). | `python -m robocli.sandbox` |
| `robocli/proxy/` | the wall: a whitelist HTTP proxy, the sandbox's only route out. | `python -m robocli.proxy` |
| `robocli/agents/` | `base.Agent` (the contract), the registry (manifests under `configs/agents/`, hooks under `plugins/agents/`, bundled then `~/.robocli/`), the launcher, credentials staging, the prompts. | `python -m robocli.agents` |
| `robocli/bench/` | running a task set: `run.py` conducts trials (and owns config assembly: `load_config`, `compose`, `bring_up`), `precheck.py` verifies every promise the manual makes before the agent boards, `record.py` is the only writer under `runs/`. | `robocli run` |
| `robocli/cli.py` | the front door: `robocli robots / benchmarks / agents / build / up / agent / down / run / config / doctor`. | `robocli` |
| `robocli/doctor.py` | is this machine ready: structured checks over docker, images (their labels against the selected agents), simulator, login, the user directory. | `robocli doctor` |

Shared leaves, importable by every host-side unit and by nothing in
`onboard/`:

| Leaf | Owns |
|---|---|
| `robocli/config/paths.py` | where things live: the user directory (`~/.robocli`), the bundled data (`robocli/configs/`, `robocli/plugins/`), the lookup order (bundled, then user, then a path), simulator and workspace locations. |
| `robocli/config/schema.py` | the schema (pydantic): robot profile, benchmark config, user config, the assembled per-trial config; defaults and a description per key; unknown keys are errors. |
| `robocli/errors.py` | the error family: message, hint, sysexits code. The CLI entry point is the one place an error becomes text. |
| `robocli/testing.py` | `check_manifest` and `check_agent`, the conformance tests third parties run. |

Data, not code, is what crosses unit boundaries: a robot profile and a
benchmark config are validated and assembled once into one config,
written to disk (`config.yaml`), and read by every party (the sandbox
seeds the manual from it, the body boots from it, the precheck derives
its checks from it). At runtime the units talk over DDS, stdio, and
files under `runs/`.

## Where things live

```
robocli/configs/{robots,benchmarks,agents}/, robocli/plugins/agents/   bundled, ship in the wheel
~/.robocli/                                             the user directory ($ROBOCLI_HOME, --home)
  config.yaml        your defaults: agent section, default robot
  robots/ benchmarks/ agents/ plugins/agents/            yours, looked up after the bundled ones
  credentials/<agent>/                                   login profiles
  simulators/<name>/.venv-*                              simulator checkouts
  workspaces/<name>/  state/<name>.yaml                  what `robocli up` keeps
./runs/                                                 trial data (--runs-root)
```

## The layering contract

- Nothing host-side imports `robocli.robot.onboard` (it lives in the
  container, with rclpy).
- Inside `onboard/`, `environment`, `ros_graph` and `monitor` never
  import each other; `boot.py` wires them with parameters.
- `onboard/` imports nothing from robocli outside itself, the shared
  leaves included: it is bake-ready for an image and reads resolved
  absolute paths and validated dicts from the resolved config file.
- The ground verbs consume data only: no shared leaf, never onboard.
- `sandbox` imports no other layer: what the agent experiences knows
  nothing about scoring.
- `precheck` and `record` are leaves; handles and paths are handed in.
- `agents` and `proxy` are leaves (they may use `paths` and `errors`).
- The schema (`config`) belongs to the conductor: units read validated
  dicts and never import it.
- Only the conductor (`bench/run.py`, and `cli.py`) brings bodies up.
- `cli`, `doctor` and `testing` sit above the units; no unit imports them.

The contract is stated in `pyproject.toml` (import-linter) and again in
`tests/test_layering.py` (AST-checked, no dependency). CI runs both.
A third test, `tests/test_agent_boundary.py`, keeps every agent-specific
token inside the manifests and hooks modules.

## Why a real robot needs only the ground verbs

`up` for a simulated robot starts a container that boots the simulator
and publishes a ROS 2 graph. For a real robot that graph already exists;
`up` only has to start the sandbox on the same DDS domain and seed the
manual from the profile. The agent cannot tell the difference, which is
the point.
