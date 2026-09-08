<div align="center">

# RoboCLI

**Let Your Claude Code or Codex Control Any Robot, Real or Simulated**

Through the standard ROS 2 CLI and client library, without relying on any VLA model.<br>
*Start playing with one command :)*

[![CI](https://github.com/terminalworld/RoboCLI/actions/workflows/ci.yml/badge.svg)](https://github.com/terminalworld/RoboCLI/actions/workflows/ci.yml)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue)](pyproject.toml)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-22314E)](docs/architecture.md)
[![License](https://img.shields.io/badge/license-Apache--2.0-green)](LICENSE)

<!-- demo GIF: Claude Code in a terminal — `ros2 topic list`, a camera
     snapshot, a trajectory goal, the arm picking the object up -->

</div>

A coding agent already knows how to work in a terminal: list what is
there, read the docs, write a script, run it, look at the result. A
ROS 2 robot already *is* a terminal: `ros2 topic`, `ros2 action`,
`rclpy`. RoboCLI puts the two together and adds nothing in between. No
robot API for the agent, no skill library, no planner: the agent gets a
shell on the robot, the robot's own command line and client library,
and a short description of the machine. Everything the agent does from
there is ordinary ROS 2.

## Quick start

```bash
pip install git+https://github.com/terminalworld/RoboCLI
robocli build robot && robocli build sandbox && robocli build proxy   # once
robocli up panda-sim                  # a simulated Franka Panda, ROS 2 graph live
```

In a second terminal:

```bash
robocli agent "pick up the bowl and place it on the plate"
```

Claude Code opens in `/workspace` on the robot's terminal. It reads
`README.md` there, runs `ros2 topic list`, grabs a camera frame, sends a
`FollowJointTrajectory` goal, and checks the result, exactly as it
would on a real machine. `robocli doctor` tells you what is missing
before the first `up` (Docker, the three images, the simulator
checkout, an agent login); the details are in [docs/install.md](docs/install.md).

## How it works

```
 your terminal                       the robot (real or simulated)
 ┌─────────────────────────┐        ┌──────────────────────────────┐
 │ robocli agent           │        │ ROS 2 graph                  │
 │  └─ Claude Code / Codex │  DDS   │  /joint_states  /tf  /camera │
 │      in a sandbox with  │◄──────►│  FollowJointTrajectory       │
 │      ros2 · rclpy · docs│        │  GripperCommand  MoveIt      │
 └─────────────────────────┘        └──────────────────────────────┘
```

- **The interface is the robot's own.** The agent sees the topics,
  actions and services the robot exposes, plus `machine.yaml` (joints,
  limits, frames, ports) and four short docs. It never sees RoboCLI.
- **The sandbox is a plain Ubuntu + ROS 2 container** with the agent
  installed, a workspace mounted, and a whitelist proxy as its only
  way out (the model API; nothing else).
- **A robot is a profile** ([`robocli/configs/robots/`](robocli/configs/robots), or
  your own under `~/.robocli/robots/`): what it is (`machine:`) and how
  it is provided (`machine.backend`: a simulator image and scene, or a
  real robot's launch command and how to reach its graph).
- **A benchmark is a task set** ([`robocli/configs/benchmarks/`](robocli/configs/benchmarks)):
  which suites and init states to load, how a trial runs and stops.
  `robocli run` runs trials, checks every promise the workspace docs
  make before the agent starts, and records each trial with full
  provenance.
- **Everything is checked against one schema** (`robocli config
  schema`): a misspelled key in any file is an error, never a silent
  no-op. Your defaults live in `~/.robocli/config.yaml`.

## Supported robots

| Profile | Robot | Backend | Scenes |
|---|---|---|---|
| `panda-sim` | Franka Emika Panda | simulated (robosuite / MuJoCo, ROS 2 Jazzy) | LIBERO-PRO |
| `panda-sim-humble` | Franka Emika Panda | simulated (robosuite / MuJoCo, ROS 2 Humble) | CaP-Bench |
| `panda-omron-sim` | Panda on an Omron mobile base | simulated (RoboCasa, ROS 2 Humble) | RoboCasa365 |
| *your robot* | any ROS 2 arm or mobile manipulator | real | see [docs/your-own-robot.md](docs/your-own-robot.md) |

`robocli robots` prints this list from the profiles on disk, yours included.

## Supported agents

| Agent | Status |
|---|---|
| [Claude Code](https://claude.com/claude-code) | supported (`--agent claude-code`) |
| [Codex](https://github.com/openai/codex) | supported (`--agent codex`) |

Bring your own agent. An agent is a manifest (how to install its CLI in the sandbox, which
hosts it talks to, how it logs in) and a small hooks class (how to
launch it); everything else is optional. Drop yours in
`~/.robocli/agents/` and `~/.robocli/plugins/agents/` or send a pull
request;
`robocli agents` lists what is available and what each can do. See
[docs/agents.md](docs/agents.md).

## Use your own robot

Draft a profile from the robot's live graph, finish the `TODO` lines,
and point `up` at it:

```bash
robocli probe --host > my-ur5.yaml   # joints, limits, frames, ports, cameras from the graph
robocli up ./my-ur5.yaml --task "..." # or copy it to ~/.robocli/robots/ and: robocli up my-ur5
```

The profile's `machine:` section is what the agent's `machine.yaml` is
generated from: model, joint names and limits, frames,
gripper, and the ports (`trajectory`, `gripper`, `twist`, `wrench`) the
robot serves. Details in [docs/your-own-robot.md](docs/your-own-robot.md).

## Simulation and benchmarks

The simulated robots run the community benchmark scenes unchanged;
their original success predicates score the trial in place.

```bash
robocli run --config libero_pro --run-id demo \
            --task-suite libero_goal_task --task-ids 0,1 --seeds 0 --operator agent
```

Every trial writes `result.json` (verdict, preflight, termination,
token accounting), `provenance.json` (code and simulator commits, image
digests, config and prompt hashes), the agent's full transcript, and
the workspace it left behind; the run directory keeps a `SUMMARY.md`
regenerated from those files after every trial. Building the simulator checkouts:
[docs/simulation.md](docs/simulation.md).

A trial replays from its own `commands.sh`, and a replay with
`--record` renders as a video, terminal on the left, cameras on the
right (`robocli demo <trial>`; see
[running-experiments.md](docs/running-experiments.md#making-a-demo-video)).

## Architecture

Six units, one direction of dependency: `robot/` (the machine, simulated
or real), `sandbox/` (the agent's terminal and workspace), `proxy/`
(the only route out), `agents/` (the agent contract, registry and
launcher), `runner/` (bring-up, preflight, operator, verdict, record),
`cli/`; beside them `demo/` renders a recorded trial's files into a
video. Who may import whom is enforced by CI (import-linter and
`tests/architecture/`). The prose is [docs/architecture.md](docs/architecture.md).

## Documentation

| page | read when |
|---|---|
| [docs/install.md](docs/install.md) | setting a machine up: images, logins, doctor |
| [examples/first-task.md](examples/first-task.md) | your first task on the simulated Panda |
| [docs/your-own-robot.md](docs/your-own-robot.md) | describing your robot in one profile |
| [examples/real-robot.md](examples/real-robot.md) | the same flow on a real ROS 2 arm |
| [docs/simulation.md](docs/simulation.md) | the simulator checkouts and GPU rendering |
| [docs/podman.md](docs/podman.md) | machines without Docker |
| [docs/running-experiments.md](docs/running-experiments.md) | `robocli run`, the `runs/` layout, every record field, replays and demo videos |
| [docs/cli.md](docs/cli.md) | every verb and flag, exit codes (generated) |
| [docs/config.md](docs/config.md) | every config key (generated) |
| [docs/agents.md](docs/agents.md) | adding a coding agent |
| [docs/architecture.md](docs/architecture.md) | the units and the layering contract |
| [CONTRIBUTING.md](CONTRIBUTING.md) | conventions for code, names and docs |

## Paper

*RoboCLI: Operating Robots Through Their Native Command-Line Interface*
(arXiv link to follow). If you use RoboCLI, please cite it; see
[CITATION.cff](CITATION.cff).

## License

Apache-2.0
