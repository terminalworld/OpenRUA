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
and a short manual. Everything the agent does from there is ordinary
ROS 2.

## Quick start

```bash
pip install robocli
robocli build robot && robocli build sandbox && robocli build proxy   # once
robocli up --robot panda-sim          # a simulated Franka Panda, ROS 2 graph live
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
substrate, an agent login).

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
- **A robot is a profile** under [`robots/`](robots): what it is
  (`machine:`) and, for simulated ones, which body image and scene to
  boot. A real robot needs only the `machine:` facts and a reachable
  ROS 2 graph.
- **A benchmark is a task set** under [`benchmarks/`](benchmarks):
  which suites and init states to load, how a trial runs and stops.
  `robocli run` conducts trials, checks the manual's promises before
  the agent boards, and records every trial with full provenance.

## Supported robots

| Profile | Robot | Body | Scenes |
|---|---|---|---|
| `panda-sim` | Franka Emika Panda | simulated (robosuite / MuJoCo, ROS 2 Jazzy) | LIBERO-PRO |
| `panda-sim-humble` | Franka Emika Panda | simulated (robosuite / MuJoCo, ROS 2 Humble) | CaP-Bench |
| `panda-omron-sim` | Panda on an Omron mobile base | simulated (RoboCasa, ROS 2 Humble) | RoboCasa365 |
| *your robot* | any ROS 2 arm or mobile manipulator | real | see [docs/your-own-robot.md](docs/your-own-robot.md) |

`robocli robots` prints this list from the profiles on disk.

## Supported agents

| Agent | Status |
|---|---|
| [Claude Code](https://claude.com/claude-code) | supported (`--cli claude-code`) |
| [Codex](https://github.com/openai/codex) | planned |

An agent is one file under [`robocli/agents/`](robocli/agents): how to
install it in the sandbox, how to launch it, what its transcript looks
like. See [docs/agents.md](docs/agents.md).

## Use your own robot

Write a profile with your robot's facts and point `up` at it:

```bash
robocli up --robot ./my-ur5.yaml
```

The profile's `machine:` section is what the agent's `machine.yaml`
manual is generated from: model, joint names and limits, frames,
gripper, and the ports (`trajectory`, `gripper`, `twist`, `wrench`) the
robot serves. Details in [docs/your-own-robot.md](docs/your-own-robot.md).

## Simulation and benchmarks

The simulated bodies run the community benchmark scenes unchanged;
their original success predicates score the trial in place.

```bash
robocli run --config benchmarks/libero_pro.yaml --run-id demo \
            --task-suite libero_goal_task --task-ids 0,1 --seeds 0 --operator agent
```

Every trial writes `result.json` (verdict, precheck, termination,
token accounting), `provenance.json` (code and substrate commits, image
digests, config and prompt hashes), the agent's full transcript, and
the workspace it left behind. Building the simulator substrates:
[docs/simulation.md](docs/simulation.md).

## Architecture

```
robocli/
  cli.py        robocli up | agent | down | run | build | doctor | robots
  robot/        the machine: ground verbs (build/up/down) + onboard/ (sim body software)
  sandbox/      the agent's terminal + workspace/ (the docs and tools the agent sees)
  agents/       one adapter per coding agent
  proxy/        the whitelist wall
  bench/        run · precheck · record: task sets, the gate, the ledger
robots/         one profile per robot
benchmarks/     one config per task set
tests/          unit tests + the layering contract (test_layering.py)
```

Who may import whom is written down twice and enforced by CI:
[`pyproject.toml`](pyproject.toml) (import-linter) and
[`tests/test_layering.py`](tests/test_layering.py). The prose version
is [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Paper

*RoboCLI: Operating Robots Through Their Native Command-Line Interface*
(arXiv link to follow). If you use RoboCLI, please cite it; see
[CITATION.cff](CITATION.cff).

## License

Apache-2.0
