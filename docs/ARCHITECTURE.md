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
| `robocli/agents/` | the occupants: one adapter module per coding agent (install, launch, transcript accounting) + the opening prompt. | `python -m robocli.agents` |
| `robocli/bench/` | running a task set: `run.py` conducts trials, `precheck.py` verifies every promise the manual makes before the agent boards, `record.py` is the only writer under `runs/`. | `robocli run` |
| `robocli/cli.py` | composes the verbs above into `robocli up / agent / down / run / build / doctor / robots`. | `robocli` |

Data, not code, is what crosses unit boundaries: a robot profile
(`robots/*.yaml`) and a benchmark config (`benchmarks/*.yaml`) are
assembled once into one config, written to disk, and read by every
party (the sandbox seeds the manual from it, the body boots from it,
the precheck derives its checks from it). At runtime the units talk
over DDS, stdio, and files under `runs/`.

## The layering contract

- Nothing host-side imports `robocli.robot.onboard` (it lives in the
  container, with rclpy).
- Inside `onboard/`, `environment`, `ros_graph` and `monitor` never
  import each other; `boot.py` wires them with parameters.
- `onboard/` imports nothing from robocli outside itself: it is
  bake-ready for an image.
- The ground verbs consume data only: no shared leaf, never onboard.
- `sandbox` imports no other layer: what the agent experiences knows
  nothing about scoring.
- `precheck` and `record` are leaves; handles and paths are handed in.
- `agents` and `proxy` are leaves.
- Only the conductor (`bench/run.py`, and `cli.py`) brings bodies up.

The contract is stated in `pyproject.toml` (import-linter) and again in
`tests/test_layering.py` (AST-checked, no dependency). CI runs both.

## Why a real robot needs only the ground verbs

`up` for a simulated robot starts a container that boots the simulator
and publishes a ROS 2 graph. For a real robot that graph already exists;
`up` only has to start the sandbox on the same DDS domain and seed the
manual from the profile. The agent cannot tell the difference, which is
the point.
