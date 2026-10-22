<div align="center">

# OpenRUA

**Let Your Claude Code or Codex Control Any Robot, Real or Simulated**

*Through the standard ROS&nbsp;2 CLI and client library, without relying on any VLA model.*

[![CI](https://github.com/terminalworld/OpenRUA/actions/workflows/ci.yml/badge.svg)](https://github.com/terminalworld/OpenRUA/actions/workflows/ci.yml)
[![arXiv](https://img.shields.io/badge/arXiv-2610.02459-b31b1b.svg)](https://arxiv.org/abs/2610.02459)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue)](pyproject.toml)
[![ROS 2](https://img.shields.io/badge/ROS%202-22314E?logo=ros&logoColor=white)](docs/architecture.md)
[![License](https://img.shields.io/badge/license-Apache--2.0-green)](LICENSE)<br>
[![Stars](https://img.shields.io/github/stars/terminalworld/OpenRUA)](https://github.com/terminalworld/OpenRUA/stargazers)
[![Forks](https://img.shields.io/github/forks/terminalworld/OpenRUA)](https://github.com/terminalworld/OpenRUA/forks)
[![Watchers](https://img.shields.io/github/watchers/terminalworld/OpenRUA)](https://github.com/terminalworld/OpenRUA/watchers)

https://github.com/user-attachments/assets/3b134c51-a949-44dd-9474-5249c3879aa0

*Codex (GPT-6 Astra) on LIBERO-10, "put the yellow and white mug on the left plate and put the white mug on the right plate": the commands it typed on the left, the robot's cameras on the right, task success. More in [docs/demos.md](docs/demos.md).*

</div>

A [robot-use agent](https://web.mit.edu/phillipi/www/writing/robot-use-agents.html) (RUA)
uses a robot just as a computer-use agent uses a computer.

OpenRUA connects off-the-shelf coding agents to robots through their native
ROS&nbsp;2 interfaces. The agent works in a workspace containing robot
documentation and starter tools, writes perception and control programs, and
uses execution feedback to continue the task.

You can chat with the agent through **OpenRUA's terminal UI or browser**, use
its **original terminal**, have it **do one task on its own, recorded**, or run
**recorded benchmark experiments**. Shared chat lets you send follow-up
instructions, inspect tool activity, manage the queue, and revisit saved
observations without starting a new robot each turn.

- [Start a shared session](examples/shared-session.md): OpenRUA TUI, browser, or plain CLI.
- [Run one task, or use the agent's original terminal](examples/first-task.md): `openrua exec` records the agent doing a task on its own; `openrua up` and `openrua agent` open its own terminal on the robot.
- [Connect a real robot](docs/connect.md): `openrua connect`, your coding agent setting it up with you.
- [Describe your robot](docs/your-own-robot.md): its ROS&nbsp;2 interfaces in a profile.
- [Run experiments](docs/running-experiments.md): fresh trials, benchmark scoring, and recorded artifacts.

> OpenRUA turns your robot into a coding project: your agent explores it
> like a live codebase, pulls sensor streams into files for reading, and
> runs commands and programs to move it.
>
> *Start playing with your robot like you code a project :)*

## 🔥 News

- **[2026-10-22]** 🧰 **Bring OpenRUA to your own robot, starting from a bare computer.** Missing Docker or an agent login? `openrua` sets them up with you, and your coding agent connects the robot step by step. ([v0.18.2](https://github.com/terminalworld/OpenRUA/releases/tag/v0.18.2))
- **[2026-10-22]** 👀 **Watch your robot through its own eyes.** The browser shows its cameras live while the agent works, and every real-robot `openrua exec` run is recorded, ready to become a demo video with `openrua demo`. ([v0.17.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.17.0))
- **[2026-10-21]** 🎯 **Hand your agent a task and let it work on its own.** `openrua exec` runs it in simulation or on a real robot and keeps a full record of the run. ([v0.14.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.14.0))
- **[2026-10-21]** 🦾 **Your coding agent can now set up a real robot for you.** Run [`openrua connect`](docs/connect.md): Claude Code or Codex works out what the robot is, builds its ROS&nbsp;2 drivers and writes its config with you, and OpenRUA checks every step. ([v0.13.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.13.0))
- **[2026-10-21]** ⌨️ **Talk to your robot the way you talk to Claude Code and Codex.** Send a follow-up while the agent works, switch `/permissions` and `/model` on the fly, and press Esc to stop the agent and the arm's motion together. ([v0.13.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.13.0))
- **[2026-10-21]** 🤖 **Real robots, ready to go.** Each robot describes itself in its own config, and OpenRUA builds its drivers and its own sandbox the first time you start it. ([v0.11.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.11.0), [v0.12.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.12.0))
- **[2026-10-06]** 💬 **A keyboard terminal for your robot** becomes the default way to chat with it. ([v0.6.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.6.0))
- **[2026-10-03]** 👥 **Shared robot conversations:** one session you can join from the terminal, the browser or a script. ([v0.1.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.1.0))
- **[2026-10-01]** 📄 **The paper is out:** [OpenRUA: Robot-Use Agents Are Zero-Shot Visuomotor Policies](https://arxiv.org/abs/2610.02459).
- **[2026-09-14]** 🚀 **OpenRUA is live.** `pip install openrua` and give Claude Code or Codex a robot, simulated or real. ([v0.0.7](https://github.com/terminalworld/OpenRUA/releases/tag/v0.0.7))

## Quick start

**Before you start**, have on a Linux computer:

- **[Docker](docs/install.md#docker)**: the robot, the simulator and your agent's terminal run in containers.
- **[Claude Code or Codex](docs/install.md#your-agents-login)**, signed in with your account: your coding agent does the work.

Missing either one? `openrua` sets it up with you.

```sh
pip install -U openrua
openrua
```

**A simulated robot.** In the setup menu, pick a robot and a scene, then
**Prepare and start**. The first start builds the images, which takes a few
minutes. Then tell the agent what you want, for example:

> Look through the cameras and describe the scene, without moving the robot.

**A real robot.** Choose **Connect a real robot** (or run `openrua connect`).
Your agent sets the robot up with you, step by step, and OpenRUA checks each
step, up to a first small motion with the emergency stop at hand
([Connect a real robot](docs/connect.md)).

**While it works.** Enter sends, Esc stops the agent and the arm's motion,
`/watch` shows a camera live and `/help` lists the rest
([terminal guide](docs/terminal.md)). Ctrl+D leaves and lets the session keep
running; `openrua --resume` brings you back.

### Next

- **One task, on its own.** `openrua exec` hands the agent a task and records
  the run, as `claude -p` or `codex exec` do ([first task](examples/first-task.md)).
- **Benchmarks and demo videos.** Run a benchmark's tasks, scored by its own
  success predicates, and render a trial as a video
  ([running experiments](docs/running-experiments.md)).
- **Your robot, described by hand.** Draft its profile from the live graph
  instead of connecting it with your agent ([your own robot](docs/your-own-robot.md)).

## Choose your interface

| Interface | Entry | When you leave |
|---|---|---|
| OpenRUA TUI | `openrua --name chat-demo` | Ctrl+D asks: keep the shared session running (Enter), or end it |
| Browser | `openrua --gui --name chat-demo` | Closing the page leaves the shared session running |
| Plain CLI | `openrua --cli --name chat-demo` | Leaving the client keeps the shared session running |
| Native agent terminal | `openrua up panda --sim robosuite --bench capbench --agent codex --name native-demo`, then `openrua agent --name native-demo` in a second terminal | Exiting the agent leaves the robot up; Ctrl-C in `up` or `openrua down --name native-demo` powers it off |
| One task, on its own | `openrua exec panda "lift the cube" --sim robosuite` | Nothing to leave: the robot powers off when the agent is done, and the run stays under `runs/` |

The first three share one conversation and its queue: join from any of them,
leave, and come back ([shared sessions](docs/sessions.md)). The
[terminal guide](docs/terminal.md) has the keys and commands.

## How it works

**Treat the robot as an interactive software project.** Give an off-the-shelf
coding agent a task, terminal access to the robot's native ROS&nbsp;2 interface,
and a workspace. The agent explores the machine, writes and tests programs,
and uses execution feedback to refine its actions.

```text
your terminal                                 the robot (real or simulated)
┌──────────────────────────────────┐          ┌────────────────────────────────┐
│ openrua                          │          │ ROS 2 graph                    │
│  └─ Claude Code / Codex          │   DDS    │ /joint_states · /tf · /camera  │
│      in a sandbox workspace      │◄────────►│ FollowJointTrajectory          │
│      ros2 · rclpy · docs         │          │ GripperCommand · MoveIt        │
│      starter tools · saved files │          │ Velocity control               │
└──────────────────────────────────┘          └────────────────────────────────┘
```

- **Workspace as harness.** Documentation and readable starter tools give the
  agent a starting point. Files preserve observations, measurements, programs,
  and notes as the task progresses. OpenRUA prescribes no robot-task workflow.
- **Perception as file I/O.** The agent pulls sensor data on demand, inspects
  saved images, and can write code to process images and compute measurements.
- **Manipulation as coding.** The agent writes and executes commands or programs
  against native ROS&nbsp;2 interfaces, incorporating sensor feedback when needed.
  It decides when to observe, how to act, and how to verify completion.

Task-specific perception and control strategies emerge from the agent's own
coding during execution. The [paper](https://arxiv.org/abs/2610.02459) analyzes
these behaviors, including image processing, metric measurement, and feedback
controllers, without a learned robot policy or a task-specific primitive library.

For the software modules, plugin boundaries, and shared-session implementation,
see [Architecture](docs/architecture.md).

## Choosing what to run

- **Robot and scene.** `--sim` selects the simulator; `--bench`, `--task-suite`,
  and `--task-id` select a benchmark scene. Without a benchmark selection or saved
  benchmark default, robosuite uses its native `Lift` scene.
- **Agent and model.** Choose `--agent` and optionally `--model` when starting
  the session. `/model <name>` changes the model from the next request; the
  agent cannot be switched inside a running conversation.
- **What the agent can reach.** A session you start sets no limits: the agent
  reaches the internet as on your machine. A recorded run (`openrua exec`,
  `openrua bench`) limits it to its model service by default. `--internet`,
  `--[no-]web-search` and `--[no-]account-connectors` set either
  ([what the agent can reach](docs/install.md#what-the-agent-can-reach)).
- **Custom profiles.** Pass a file with `--robot`, `--sim`, or `--bench` to use
  your own environment in the TUI. It is validated and prepared without being
  replaced by a guided choice. See [your own robot](docs/your-own-robot.md).
- **Defaults.** Save common choices with `openrua config set`; explicit command
  arguments override them. [Configuration](docs/config.md) documents the fields.
- **Available extensions.** `openrua robots`, `openrua simulators`,
  `openrua benchmarks`, and `openrua agents` list the bundled and local choices.
  `openrua benchmarks libero_pro` lists its suites and tasks.

## Supported robots

A robot is what is true of it wherever it runs: joints, limits, frames,
gripper, ports, planner. Which simulator embodies it, and what surrounds
it, come from the other two kinds of file.

| Robot | Model | Embodied by |
|---|---|---|
| `panda` | Franka Emika Panda | `robosuite`, `maniskill`, `calvin`, `vlabench` |
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
| `calvin` | [calvin_env](https://github.com/mees/calvin_env) on PyBullet (TinyRenderer, CPU) | `panda` | none: name the benchmark |
| `vlabench` | [VLABench](https://github.com/OpenMOSS/VLABench)'s dm_control environments on MuJoCo 3.2 | `panda` | none: name the benchmark |

A simulator file knows the engine and how it drives each robot it
embodies; it knows no benchmark. Its install, and the benchmarks' own
over it, is what `openrua build` renders into one image per declaration
(`openrua-sim-<name>`: ROS 2, the checkouts, the assets, the Python
environment); [docs/simulation.md](docs/simulation.md) describes them.

## Supported benchmarks

| Benchmark | Robot | Simulator | Brings |
|---|---|---|---|
| [LIBERO](https://github.com/Lifelong-Robot-Learning/LIBERO) (`libero`) | `panda` | `robosuite` | the four standard suites and LIBERO-90, on LIBERO's robosuite 1.4 fork, ROS&nbsp;2 Jazzy |
| [LIBERO-PRO](https://github.com/Zxy-MLlab/LIBERO-PRO) (`libero_pro`) | `panda` | `robosuite` | LIBERO's scenes under five perturbation axes, same fork as `libero` |
| [LIBERO-Plus](https://github.com/sylvestf/LIBERO-plus) (`libero_plus`) | `panda` | `robosuite` | ~10,000 perturbed variants of the four suites, its own fork and assets, ROS&nbsp;2 Jazzy |
| [LIBERO-Mem](https://github.com/libero-mem/libero-mem) (`libero_mem`) | `panda` | `robosuite` | ten non-Markovian tasks with subgoal sequences, its own fork, ROS&nbsp;2 Jazzy |
| [RoboCerebra](https://github.com/qiuboxiang/RoboCerebra) (`robocerebra`) | `panda` | `robosuite` | long-horizon tabletop cases on its LIBERO fork, the `Ideal` protocol, ROS&nbsp;2 Jazzy |
| [CaP-Bench](https://github.com/capgym/cap-x) (`capbench`) | `panda` | `robosuite` | CaP-X's tabletop scenes on robosuite 1.5, ROS&nbsp;2 Humble |
| [RoboCasa](https://robocasa.ai) (`robocasa`) | `panda-omron` | `robosuite` | the original release's 24 atomic kitchen tasks (v0.2 on robosuite 1.5.0), ROS&nbsp;2 Humble |
| [RoboCasa365](https://robocasa.ai) (`robocasa365`) | `panda-omron` | `robosuite` | the 365-task release's kitchens and the Panda-Omron body, ROS&nbsp;2 Humble |
| [ManiSkill](https://maniskill.ai) (`maniskill`) | `panda` | `maniskill` | the eleven table-top Panda tasks that ship with ManiSkill 3, seeded resets, ROS&nbsp;2 Jazzy |
| [SimplerEnv](https://simpler-env.github.io) (`simpler`) | `widowx` | `maniskill` | the four WidowX Bridge tasks as their authors ported them to ManiSkill 3 (the SAPIEN 2 original needs a GPU; its Google Robot tasks are not ported), the visual-matching placement grid, ROS&nbsp;2 Jazzy |
| [MIKASA-Robo](https://github.com/CognitiveAISystems/MIKASA-Robo) (`mikasa`) | `panda` | `maniskill` | the 90 language-conditioned memory tasks (remember, shell game, intercept, ...), its own image on ManiSkill 3.0.1, ROS&nbsp;2 Jazzy |
| [RoboTwin 2.0](https://robotwin-platform.github.io) (`robotwin`) | `aloha-agilex` | `robotwin` | the fifty dual-arm tasks under the Easy protocol (`demo_clean`); the Hard protocol needs its 11 GB textures and is not declared; ROS&nbsp;2 Jazzy |
| [CALVIN](https://github.com/mees/calvin) (`calvin`) | `panda` | `calvin` | the 1000 five-subtask chains of the long-horizon evaluation on play table D, each with its fixed initial condition and the benchmark's task oracle, ROS&nbsp;2 Humble |
| [VLABench](https://github.com/OpenMOSS/VLABench) (`vlabench`) | `panda` | `vlabench` | every task registered in the pinned checkout (5 GB of objects and scenes), seeded resets, the task's own termination as success, ROS&nbsp;2 Humble |

A benchmark names its robot and simulator and brings its own world:
`install:` (its image contents and ROS distro) and `scenes:` (scene
cameras, and robot embodiments its assets add). `openrua benchmarks` prints this
list; `openrua bench --config <name>` runs one.

## Supported agents

| Agent | Status |
|---|---|
| [Claude Code](https://claude.com/claude-code) | supported (`--agent claude-code`) |
| [Codex](https://github.com/openai/codex) | supported (`--agent codex`) |

Experimental external plugins are available for
[Kimi Code](examples/plugins/kimi/README.md) and
[ZCode](examples/plugins/zcode/README.md). Both have passed small real-provider
file tasks and two-turn shared-chat checks. Their API paths have real-provider checks; native Kimi OAuth and ZCode
Coding Plan key profile staging have offline checks only. ZCode OAuth reuse
remains unsupported, and robot tasks remain unvalidated. They are not default setup choices.

Bring your own agent. An agent is a manifest (how to install its CLI in the sandbox, which
hosts it talks to, how it logs in) and a small hooks class (how to
launch it); everything else is optional. Pass yours as a path
(`--agent ./my-agent.yaml`) or send a pull request;
`openrua agents` lists what is available and what each can do. See
[docs/agents.md](docs/agents.md).

## Results

Results below are from our [paper](https://arxiv.org/abs/2610.02459), evaluated
zero-shot in simulation without privileged simulator state or access to the
benchmark's success predicate during execution.

Main benchmark results use Claude Code with Claude Opus 5 at high reasoning effort:

| Benchmark | Evaluation scope | Success |
|---|---|---|
| CaP-Bench | Seven manipulation tasks | 99.0% |
| LIBERO-PRO | All four suites, both perturbation types | 87.0% |

The model comparison uses the **LIBERO-10 suite of LIBERO-PRO**, with both
perturbation types and 200 trials per configuration:

| Agent | Model (reasoning effort) | Success |
|---|---|---|
| Claude Code | Claude Sonnet 5 (high) | 24.0% |
| Claude Code | Claude Opus 5 (high) | 72.5% |
| Claude Code | Claude Fable 5.1 (high) | 75.0% |
| Codex | GPT-6 Astra (medium) | 62.5% |
| Codex | GPT-6 Astra (high) | 69.0% |

See the paper for baselines, evaluation protocols and detailed analysis.

## Use your own robot

The quickest way is `openrua connect my-arm`: your coding agent finds out
what the robot is, builds and starts its ROS&nbsp;2 drivers and writes
`robot.yaml` with you, and OpenRUA checks each step. When every check has
passed, the same conversation goes on in the robot's own terminal. See
[docs/connect.md](docs/connect.md).

To write the profile yourself, draft it from the robot's live graph, finish
the `TODO` lines, and pass it where a robot is named:

```bash
openrua probe --host > my-ur5.yaml   # joints, limits, frames, ports, cameras from the graph
openrua --robot ./my-ur5.yaml        # chat with the agent on it; or: openrua config set --robot ./my-ur5.yaml
openrua exec ./my-ur5.yaml "put the cup on the plate"   # one task on its own, recorded
```

The profile says what the machine is and how its drivers start and stop
([your own robot](docs/your-own-robot.md)).

## Simulation and benchmarks

The simulated robots run the community benchmark scenes unchanged;
their original success predicates score the trial in place. `openrua exec`
runs one trial; `openrua bench` runs a benchmark's task set, one trial per
task and seed:

```bash
openrua bench --config libero_pro --run-id demo \
            --task-suite libero_goal_task --task-ids 0,1 --seeds 0 --operator agent
```

Every trial keeps its verdict, provenance, the agent's transcript and the
workspace it left, and replays from its own commands; a replay renders as a
video, terminal on the left and cameras on the right
([running experiments](docs/running-experiments.md)).

## Architecture

The implementation separates robot resources, native agent adapters, shared
conversations, and presentation. `runner/` composes resources; `sessions/`
coordinates messages and events; `terminal/`, `ui/terminal/` and `web/` provide client interfaces.
Robot, sandbox, proxy, and agent knowledge stay in their respective modules.
Benchmark execution and recorded demos have separate entry points.

Import-linter and architecture tests enforce these boundaries. See the
[module map](docs/architecture.md#units), [agent extension guide](docs/agents.md),
and [contribution guide](CONTRIBUTING.md).

## Documentation

| page | read when |
|---|---|
| [docs/install.md](docs/install.md) | setting a machine up: Docker, your agent's login, images, doctor |
| [examples/first-task.md](examples/first-task.md) | your first task on the simulated Panda: one recorded run with `exec`, and the agent's original terminal with `up` and `agent` |
| [examples/shared-session.md](examples/shared-session.md) | TUI, browser and CLI chat, queue checks, reconnection, saved images and shutdown |
| [docs/terminal.md](docs/terminal.md) | TUI shortcuts, expandable panels, and current limitations |
| [docs/sessions.md](docs/sessions.md) | shared-session operations, API, architecture and validation scope |
| [docs/connect.md](docs/connect.md) | connecting a real robot with your coding agent, and what each step checks |
| [docs/your-own-robot.md](docs/your-own-robot.md) | describing your robot in one profile |
| [examples/real-robot.md](examples/real-robot.md) | the same flow on a real ROS&nbsp;2 arm |
| [docs/setup-validation.md](docs/setup-validation.md) | which combinations the setup menu offers, and the checks behind them |
| [docs/simulation.md](docs/simulation.md) | the simulator checkouts and GPU rendering |
| [docs/podman.md](docs/podman.md) | machines without Docker |
| [docs/demos.md](docs/demos.md) | recorded trials rendered as videos, one per task type |
| [docs/running-experiments.md](docs/running-experiments.md) | `openrua exec` and `openrua bench`, the `runs/` layout, every record field, replays and demo videos |
| [docs/cli.md](docs/cli.md) | every verb and flag, exit codes (generated) |
| [docs/config.md](docs/config.md) | every config key (generated) |
| [docs/agents.md](docs/agents.md) | adding a coding agent |
| [docs/architecture.md](docs/architecture.md) | the units and the layering contract |
| [CONTRIBUTING.md](CONTRIBUTING.md) | conventions for code, names and docs |
| [CHANGELOG.md](CHANGELOG.md) | release changes, upgrade notes, and how versions are numbered |

## Citation

If you use OpenRUA in your research, please cite our [paper](https://arxiv.org/abs/2610.02459):

```bibtex
@misc{chu2026openrua,
  title         = {{OpenRUA}: Robot-Use Agents Are Zero-Shot Visuomotor Policies},
  author        = {Zhaoyang Chu and Earl T. Barr and Claire Le Goues and Peter O'Hearn and Mark Harman and Federica Sarro and He Ye},
  year          = {2026},
  eprint        = {2610.02459},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/2610.02459}
}
```

## License

Apache-2.0

## Feedback

Try OpenRUA with your coding agent and robot, and tell us how it goes.
Bug reports, confusing steps, and ideas for improving the experience are welcome
in [GitHub Issues](https://github.com/terminalworld/OpenRUA/issues).
If something fails, include your OpenRUA version, agent, environment, and the
command or steps that led to it. Please remove credentials and private data
from any logs you share.
