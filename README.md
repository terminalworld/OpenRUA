<div align="center">

# RoboCLI

**Connect Your Claude Code and Codex Directly to Any Robot, 100% Natively**

Real or simulated, on the robot's own ROS 2 command line.

[![CI](https://github.com/terminalworld/RoboCLI/actions/workflows/ci.yml/badge.svg)](https://github.com/terminalworld/RoboCLI/actions/workflows/ci.yml)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue)](pyproject.toml)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-22314E)](docs/ARCHITECTURE.md)
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
pip install robocli
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
checkout, an agent login).

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
| [Codex](https://github.com/openai/codex) | planned |

An agent is a manifest (how to install its CLI in the sandbox, which
hosts it talks to, how it logs in) and a small hooks class (how to
launch it); everything else is optional. Drop yours in
`~/.robocli/agents/` and `~/.robocli/plugins/agents/` or send a pull
request;
`robocli agents` lists what is available and what each can do. See
[docs/agents.md](docs/agents.md).

## Use your own robot

Write a profile with your robot's facts and point `up` at it:

```bash
robocli up ./my-ur5.yaml          # or copy it to ~/.robocli/robots/ and: robocli up my-ur5
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
the workspace it left behind. Building the simulator checkouts:
[docs/simulation.md](docs/simulation.md).

## Architecture

```
robocli/
  cli/          robocli robots | benchmarks | agents | build | up | agent | down | run | config | doctor
  doctor/       structured checks: docker, images, simulator, login
  config/       the schema every file is checked against, the loader, where things live
  configs/      bundled robots/, benchmarks/, agents/ (manifests), config.yaml (defaults)
  plugins/      agents/: the hooks module behind each agent manifest
  errors.py     errors with a fix and an exit code
  testing.py    the agent conformance test
  robot/        the machine: sim/ (container + bridge/, the robot's own software), real/
  sandbox/      the agent's terminal + workspace/ (the docs and tools the agent sees)
  agents/       the agent contract (base.py), the registry, the launcher, the prompts
  proxy/        the whitelist proxy
  runner/       run: bring-up, preflight, the operator, the verdict, the record
tests/          one directory per unit + architecture/ (the layering contract)
```

Who may import whom is written down twice and enforced by CI:
[`pyproject.toml`](pyproject.toml) (import-linter) and
[`tests/architecture/test_layering.py`](tests/architecture/test_layering.py). The prose version
is [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md); how to add a robot,
an agent or a config key is in [CONTRIBUTING.md](CONTRIBUTING.md).

## Paper

*RoboCLI: Operating Robots Through Their Native Command-Line Interface*
(arXiv link to follow). If you use RoboCLI, please cite it; see
[CITATION.cff](CITATION.cff).

## License

Apache-2.0
