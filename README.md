<div align="center">

# OpenRUA

**Let Your Claude Code or Codex Control Any Robot, Real or Simulated**

*Through the standard ROS&nbsp;2 CLI and client library, without relying on any VLA model.*

[![CI](https://github.com/terminalworld/OpenRUA/actions/workflows/ci.yml/badge.svg)](https://github.com/terminalworld/OpenRUA/actions/workflows/ci.yml)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue)](pyproject.toml)
[![ROS 2](https://img.shields.io/badge/ROS%202-22314E?logo=ros&logoColor=white)](docs/architecture.md)
[![License](https://img.shields.io/badge/license-Apache--2.0-green)](LICENSE)<br>
[![Stars](https://img.shields.io/github/stars/terminalworld/OpenRUA)](https://github.com/terminalworld/OpenRUA/stargazers)
[![Forks](https://img.shields.io/github/forks/terminalworld/OpenRUA)](https://github.com/terminalworld/OpenRUA/forks)
[![Watchers](https://img.shields.io/github/watchers/terminalworld/OpenRUA)](https://github.com/terminalworld/OpenRUA/watchers)

<!-- demo GIF: Claude Code in a terminal — `ros2 topic list`, a camera
     snapshot, a trajectory goal, the arm picking the object up -->

</div>

A [robot-use agent](https://web.mit.edu/phillipi/www/writing/robot-use-agents.html)
uses a robot just as a computer-use agent uses a computer. OpenRUA is
the open harness for one: type `openrua run "pick up the red cube"` and
Claude Code or Codex opens in a terminal on the robot's ROS&nbsp;2 graph,
lists the topics, reads the docs in its workspace, writes a script with
`rclpy`, runs it, and checks the camera.

OpenRUA gives you one command for three things:

- **Play in simulation.** `openrua run` brings up a Franka Panda in
  MuJoCo; the agent drives it the same way it would a real one.
- **Put an agent on your robot.** Draft its file from the robot's live
  graph, finish the `TODO` lines, `openrua run <name>`. See
  [docs/your-own-robot.md](docs/your-own-robot.md).
- **Run experiments.** `openrua bench` plays a benchmark across tasks and
  seeds with a fresh sandbox per trial and archives every command the
  agent ran. See [docs/running-experiments.md](docs/running-experiments.md).

> OpenRUA turns your robot into a coding project: your agent explores it
> like a live codebase, pulls sensor streams into files for reading, and
> runs commands and programs to move it.
>
> *Start playing with your robot like you code a project :)*

## Quick start

Install, choose a robot and a simulator once, run:

```bash
pip install git+https://github.com/terminalworld/OpenRUA
openrua config set --robot panda --sim robosuite --agent claude-code   # your defaults
openrua build                                       # the three images, once
openrua install --sim robosuite                     # the simulator's checkout and venv, once
openrua run "pick up the red cube"
```

The robot comes up on its ROS&nbsp;2 graph, the agent opens on its
terminal with that sentence, and the robot powers off when you leave;
`run` first prints which robot, scene and agent it picked.

`openrua doctor` tells you what is missing before the first `run`
(Docker, the three images, the simulator install, an agent login); the
details are in [docs/install.md](docs/install.md).

## Choosing what to run

- **The world.** Without `--bench` the scene is the simulator's own (robosuite: a
  table and a cube). `--bench libero_pro` loads a benchmark's world instead, a
  LIBERO kitchen with a bowl, a plate, a wine bottle, a drawer and a stove;
  `--task-suite` and `--task-id` pick a scene inside it.
- **The agent.** `--agent codex` opens Codex; `--model` picks the model.
- **Once or every time.** Whatever `config set` stored can also be given on the
  command line: `openrua run panda --sim robosuite --bench libero_pro "pick up the bowl"`.
  `openrua robots`, `openrua simulators`, `openrua benchmarks` and
  `openrua agents` list the choices; `openrua benchmarks libero_pro`
  lists one benchmark's suites and tasks.
- **A robot that stays up.** The same three steps as separate commands:
  `openrua up`, then `openrua agent "..."` in a second terminal, then
  `openrua down`.

## How it works

```
 your terminal                       the robot (real or simulated)
 ┌─────────────────────────┐        ┌──────────────────────────────┐
 │ openrua run             │        │ ROS 2 graph                  │
 │  └─ Claude Code / Codex │  DDS   │  /joint_states  /tf  /camera │
 │      in a sandbox with  │◄──────►│  FollowJointTrajectory       │
 │      ros2 · rclpy · docs│        │  GripperCommand  MoveIt      │
 └─────────────────────────┘        └──────────────────────────────┘
```

- **The interface is the robot's own.** The agent sees the topics,
  actions and services the robot exposes, plus `machine.yaml` (joints,
  limits, frames, ports) and four short docs. It never sees OpenRUA.
- **The sandbox is a plain Ubuntu + ROS&nbsp;2 container** with the agent
  installed, a workspace mounted, and a whitelist proxy as its only
  way out (the model API; nothing else).
- **A robot is a profile** ([`openrua/configs/robots/`](openrua/configs/robots), or
  a file of your own passed by path): what it is (`machine:`) and how
  it is provided (`machine.backend`: a simulator image and scene, or a
  real robot's launch command and how to reach its graph).
- **A benchmark is a task set** ([`openrua/configs/benchmarks/`](openrua/configs/benchmarks)):
  which suites and init states to load, how a trial runs and stops.
  `openrua bench` runs trials, checks every promise the workspace docs
  make before the agent starts, and records each trial with full
  provenance.
- **Everything is checked against one schema** (`openrua config
  schema`): a misspelled key in any file is an error, never a silent
  no-op. Your defaults live in `~/.openrua/config.yaml`.

## Supported robots

A robot is what is true of it wherever it runs: joints, limits, frames,
gripper, ports, planner. Which simulator embodies it, and what surrounds
it, come from the other two kinds of file.

| Robot | Model | Embodied by |
|---|---|---|
| `panda` | Franka Emika Panda | `robosuite`, `maniskill` |
| `panda-omron` | Panda on an Omron mobile base | `robosuite` through `robocasa` / `robocasa365`'s assets |
| `widowx` | Trossen WidowX 250S | `maniskill` (the Bridge dataset's arm, through `simpler`) |
| `aloha-agilex` | AgileX Cobot Magic with two ARX X5 arms | `robotwin` |
| *your robot* | any ROS&nbsp;2 arm or mobile manipulator, real | its own file; see [docs/your-own-robot.md](docs/your-own-robot.md) |

`openrua robots` prints this list from the files on disk, yours included.

## Supported simulators

| Simulator | Engine | Robots | Native scene |
|---|---|---|---|
| `robosuite` | [robosuite](https://robosuite.ai) 1.5 on MuJoCo | `panda` | `Lift`: a table and a cube |
| `maniskill` | [ManiSkill](https://maniskill.ai) 3 on SAPIEN 3 (PhysX, CPU) | `panda`, `widowx` | `PickCube-v1`: a table, a cube and a goal marker |
| `robotwin` | [RoboTwin 2.0](https://robotwin-platform.github.io)'s harness on SAPIEN 3 (PhysX, CPU) | `aloha-agilex` | none: name a benchmark |

A simulator file knows the engine and how it drives each robot it
embodies; it knows no benchmark. Its install (a venv under
`~/.openrua/simulators/`) and the benchmarks' own are described in
[docs/simulation.md](docs/simulation.md).

## Supported benchmarks

| Benchmark | Robot | Simulator | Brings |
|---|---|---|---|
| [LIBERO](https://github.com/Lifelong-Robot-Learning/LIBERO) (`libero`) | `panda` | `robosuite` | the four standard suites and LIBERO-90, on LIBERO's robosuite 1.4 fork, ROS&nbsp;2 Jazzy |
| [LIBERO-PRO](https://github.com/Zxy-MLlab/LIBERO-PRO) (`libero_pro`) | `panda` | `robosuite` | LIBERO's scenes under five perturbation axes, same fork and venv as `libero` |
| [LIBERO-Plus](https://github.com/sylvestf/LIBERO-plus) (`libero_plus`) | `panda` | `robosuite` | ~10,000 perturbed variants of the four suites, its own fork and assets |
| [LIBERO-Mem](https://github.com/libero-mem/libero-mem) (`libero_mem`) | `panda` | `robosuite` | ten non-Markovian tasks with subgoal sequences, its own fork |
| [RoboCerebra](https://github.com/qiuboxiang/RoboCerebra) (`robocerebra`) | `panda` | `robosuite` | long-horizon tabletop cases on its LIBERO fork, the `Ideal` protocol |
| [CaP-Bench](https://github.com/capgym/cap-x) (`capbench`) | `panda` | `robosuite` | CaP-X's tabletop scenes on robosuite 1.5, ROS&nbsp;2 Humble |
| [RoboCasa](https://robocasa.ai) (`robocasa`) | `panda-omron` | `robosuite` | the original release's 24 atomic kitchen tasks (v0.2 on robosuite 1.5.0), ROS&nbsp;2 Humble |
| [RoboCasa365](https://robocasa.ai) (`robocasa365`) | `panda-omron` | `robosuite` | the 365-task release's kitchens and the Panda-Omron body, ROS&nbsp;2 Humble |
| [ManiSkill](https://maniskill.ai) (`maniskill`) | `panda` | `maniskill` | the eleven table-top Panda tasks that ship with ManiSkill 3, seeded resets, ROS&nbsp;2 Jazzy |
| [SimplerEnv](https://simpler-env.github.io) (`simpler`) | `widowx` | `maniskill` | the four WidowX Bridge tasks as their authors ported them to ManiSkill 3 (the SAPIEN 2 original needs a GPU; its Google Robot tasks are not ported), the visual-matching placement grid |
| [MIKASA-Robo](https://github.com/CognitiveAISystems/MIKASA-Robo) (`mikasa`) | `panda` | `maniskill` | the 90 language-conditioned memory tasks (remember, shell game, intercept, ...), its own venv on ManiSkill 3.0.1 |
| [RoboTwin 2.0](https://robotwin-platform.github.io) (`robotwin`) | `aloha-agilex` | `robotwin` | the fifty dual-arm tasks under the Easy protocol (`demo_clean`); the Hard protocol needs its 11 GB textures and is not declared |

A benchmark names its robot and simulator and brings its own world:
`install:` (its venv and ROS distro) and `scenes:` (scene cameras, and
robot embodiments its assets add). `openrua benchmarks` prints this
list; `openrua bench --config <name>` runs one.

## Supported agents

| Agent | Status |
|---|---|
| [Claude Code](https://claude.com/claude-code) | supported (`--agent claude-code`) |
| [Codex](https://github.com/openai/codex) | supported (`--agent codex`) |

Bring your own agent. An agent is a manifest (how to install its CLI in the sandbox, which
hosts it talks to, how it logs in) and a small hooks class (how to
launch it); everything else is optional. Pass yours as a path
(`--agent ./my-agent.yaml`) or send a pull request;
`openrua agents` lists what is available and what each can do. See
[docs/agents.md](docs/agents.md).

## Use your own robot

Draft a profile from the robot's live graph, finish the `TODO` lines,
and point `up` at it:

```bash
openrua probe --host > my-ur5.yaml   # joints, limits, frames, ports, cameras from the graph
openrua run ./my-ur5.yaml "..."      # or: openrua config set --robot ./my-ur5.yaml
```

The profile's `machine:` section is what the agent's `machine.yaml` is
generated from: model, joint names and limits, frames,
gripper, and the ports (`trajectory`, `gripper`, `twist`, `wrench`) the
robot serves. Details in [docs/your-own-robot.md](docs/your-own-robot.md).

## Simulation and benchmarks

The simulated robots run the community benchmark scenes unchanged;
their original success predicates score the trial in place.

```bash
openrua bench --config libero_pro --run-id demo \
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
right (`openrua demo <trial>`; see
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
| [examples/real-robot.md](examples/real-robot.md) | the same flow on a real ROS&nbsp;2 arm |
| [docs/simulation.md](docs/simulation.md) | the simulator checkouts and GPU rendering |
| [docs/podman.md](docs/podman.md) | machines without Docker |
| [docs/running-experiments.md](docs/running-experiments.md) | `openrua bench`, the `runs/` layout, every record field, replays and demo videos |
| [docs/cli.md](docs/cli.md) | every verb and flag, exit codes (generated) |
| [docs/config.md](docs/config.md) | every config key (generated) |
| [docs/agents.md](docs/agents.md) | adding a coding agent |
| [docs/architecture.md](docs/architecture.md) | the units and the layering contract |
| [CONTRIBUTING.md](CONTRIBUTING.md) | conventions for code, names and docs |

## License

Apache-2.0
