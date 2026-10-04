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

https://github.com/user-attachments/assets/3b134c51-a949-44dd-9474-5249c3879aa0

*Codex (GPT-6 Astra) on LIBERO-10, "put the yellow and white mug on the left plate and put the white mug on the right plate": the commands it typed on the left, the robot's cameras on the right, task success. More in [docs/demos.md](docs/demos.md).*

</div>

OpenRUA connects off-the-shelf coding agents to robots through their native
ROS&nbsp;2 interfaces. The agent works in a workspace containing robot
documentation and starter tools, writes perception and control programs, and
uses execution feedback to continue the task.

You can chat with the agent through **OpenRUA's terminal UI or browser**, use
its **original terminal**, or run **recorded benchmark experiments**. Shared
chat lets you send follow-up instructions, inspect tool activity, manage the
queue, and revisit saved observations without starting a new robot each turn.

- [Start a shared session](examples/shared-session.md): OpenRUA TUI, browser, or plain CLI.
- [Use the agent's original terminal](examples/first-task.md): launch a task with `openrua run`.
- [Connect your robot](docs/your-own-robot.md): describe its ROS&nbsp;2 interfaces in a profile.
- [Run experiments](docs/running-experiments.md): fresh trials, benchmark scoring, and recorded artifacts.

## Quick start

Use a Linux host with Docker or supported Podman setup. The terminal UI is
included in the default installation. Automatic session IDs and history selection
are available from version 0.3.0:

```sh
pip install -U 'openrua>=0.3.0'
openrua
```

Each ordinary launch creates a new session with an automatic ID; no name is
required. The TUI guides you through selecting a robot, simulator,
optional benchmark, and coding agent. Names support completion; you can also
enter paths to your own profiles. **Save & check** writes your choices to
`~/.openrua/config.yaml` and shows the preparation still needed. It does not
build images or log in automatically. Follow the commands shown by the check
and the [installation guide](docs/install.md), then choose **Save & start**.

OpenRUA starts the existing session service in the background and opens chat.
Type an instruction and press **Ctrl+S**, for example:

> Inspect the workspace documentation and describe the scene without moving the robot.

**Ctrl+Q** leaves the interface while the session keeps running. Use `/resume`
in the startup form or chat to search previous conversations by title or ID.
You can also run `openrua --resume`, or `openrua --gui --resume ID` to open a
running conversation and its saved workspace images in the browser. To edit defaults later, use `openrua --setup`,
`openrua config set`, or edit the same `config.yaml`; changes apply to new
sessions, not the one already running.

Choose **End session** when finished. Records and workspace files are retained;
run `openrua` again for a new session. Ended or unavailable conversations open
read-only from history; selecting them never restarts robot execution. The
[full walkthrough](examples/shared-session.md) gives a concrete simulated Panda
example with image preparation, queue operations, SSH access, and shutdown.

Shared chat remains experimental. Codex has live shared-session checks;
successful Claude Code turns through the shared adapter still need validation.
See [validation scope](docs/sessions.md#validation-scope).

## Choose your interface

| Interface | Entry | When you leave |
|---|---|---|
| OpenRUA TUI | `openrua --name chat-demo` | Ctrl+Q detaches; the shared session continues |
| Browser | `openrua --gui --name chat-demo` | Closing the page leaves the shared session running |
| Plain CLI | `openrua --cli --name chat-demo` | Leaving the client keeps the shared session running |
| Native agent terminal | `openrua run panda --sim robosuite --bench capbench --agent codex --name native-demo` | Exiting the agent stops the resources owned by `run` |

The first three use one shared queue per session. These examples explicitly
name a session; omitting `--name` creates a new one with an automatic ID.
To reconnect to a running session, pass only its name and interface choice;
startup options cannot change an active conversation. Existing `chat` and
`session` commands remain available for explicit client operations.
Native `run` starts a separate session using the agent's original TUI; it cannot
yet take over that shared conversation. For a native terminal with a robot that
stays up between agent visits, use `up`, `agent`, and `down` as described in
[First task in simulation](examples/first-task.md).

In shared chat, new instructions queue behind the active turn. Interrupting
pauses the queue for review; **Resume** continues it. Ending the session stops
its resources while preserving files; deleting them is a separate operation.
For development, `openrua serve` still runs the service in the foreground.
Stopping that process ends its session, unlike closing one of its clients.

## How it works

```text
OpenRUA TUI / browser / plain CLI
                |
       shared session service
       queue, events, resource ownership
                |
       native coding agent in a workspace
       ROS 2 docs, starter tools, saved files
                |
         ros2 commands / rclpy programs
                |
       robot's native ROS 2 interfaces
       sensors, trajectories, gripper, velocities
```

The service coordinates user messages and resources. The coding agent decides
when to observe, what programs to write, and how to complete the robot task.
Native-terminal and benchmark entry points reuse the robot, workspace, and
agent adapters without going through the shared chat queue.

- **The robot provides the control interface.** The agent reads `machine.yaml`
  and workspace documentation, acquires sensor data, and sends ROS&nbsp;2 requests.
- **The workspace is the harness.** A ROS&nbsp;2 sandbox contains the agent CLI,
  documentation, and readable starter-tool source. The agent can inspect, adapt,
  or replace the tools and save observations and programs for later use.
- **Extensions have their own boundaries.** Agent manifests and hooks own
  agent integration; robot profiles own hardware facts; simulator engines and
  benchmark loaders own their environments. Frontends use the common session API.
- **Experiments retain their own protocol.** `openrua bench` creates fresh trials,
  checks workspace interfaces, and records scoring and provenance separately
  from interactive conversations.

See [Architecture](docs/architecture.md) for the module boundaries and contracts.

## Choosing what to run

- **Robot and scene.** `--sim` selects the simulator; `--bench`, `--task-suite`,
  and `--task-id` select a benchmark scene. Without a benchmark selection or saved
  benchmark default, robosuite uses its native `Lift` scene.
- **Agent and model.** Choose `--agent` and optionally `--model` when starting
  the session. Switching them inside a running conversation is not implemented.
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

Bring your own agent. An agent is a manifest (how to install its CLI in the sandbox, which
hosts it talks to, how it logs in) and a small hooks class (how to
launch it); everything else is optional. Pass yours as a path
(`--agent ./my-agent.yaml`) or send a pull request;
`openrua agents` lists what is available and what each can do. See
[docs/agents.md](docs/agents.md).

## Results

Detailed results will be released with the paper.

| Benchmark | Agent | Model (reasoning effort) | Success |
|---|---|---|---|
| CaP-Bench | Claude Code | Claude Opus 5 (high) | 99.0% |
| LIBERO-PRO | Claude Code | Claude Opus 5 (high) | 87.0% |
| LIBERO-10 (LIBERO-PRO) | Claude Code | Claude Opus 5 (high) | 72.5% |
| LIBERO-10 (LIBERO-PRO) | Codex | GPT-6 Astra (medium) | 62.5% |

## Use your own robot

Draft a profile from the robot's live graph, finish the `TODO` lines,
and pass it to `run`:

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

The implementation separates robot resources, native agent adapters, shared
conversations, and presentation. `runner/` composes resources; `sessions/`
coordinates messages and events; `tui/` and `web/` provide client interfaces.
Robot, sandbox, proxy, and agent knowledge stay in their respective modules.
Benchmark execution and recorded demos have separate entry points.

Import-linter and architecture tests enforce these boundaries. See the
[module map](docs/architecture.md#units), [agent extension guide](docs/agents.md),
and [contribution guide](CONTRIBUTING.md).

## Documentation

| page | read when |
|---|---|
| [docs/install.md](docs/install.md) | setting a machine up: images, logins, doctor |
| [examples/first-task.md](examples/first-task.md) | your first task on the simulated Panda in the original agent terminal |
| [examples/shared-session.md](examples/shared-session.md) | TUI, browser and CLI chat, queue checks, reconnection, saved images and shutdown |
| [docs/terminal.md](docs/terminal.md) | TUI shortcuts, expandable panels, and current limitations |
| [docs/sessions.md](docs/sessions.md) | shared-session operations, API, architecture and validation scope |
| [docs/your-own-robot.md](docs/your-own-robot.md) | describing your robot in one profile |
| [examples/real-robot.md](examples/real-robot.md) | the same flow on a real ROS&nbsp;2 arm |
| [docs/simulation.md](docs/simulation.md) | the simulator checkouts and GPU rendering |
| [docs/podman.md](docs/podman.md) | machines without Docker |
| [docs/demos.md](docs/demos.md) | recorded trials rendered as videos, one per task type |
| [docs/running-experiments.md](docs/running-experiments.md) | `openrua bench`, the `runs/` layout, every record field, replays and demo videos |
| [docs/cli.md](docs/cli.md) | every verb and flag, exit codes (generated) |
| [docs/config.md](docs/config.md) | every config key (generated) |
| [docs/agents.md](docs/agents.md) | adding a coding agent |
| [docs/architecture.md](docs/architecture.md) | the units and the layering contract |
| [CONTRIBUTING.md](CONTRIBUTING.md) | conventions for code, names and docs |
| [CHANGELOG.md](CHANGELOG.md) | release changes and upgrade notes |

## Citation

The paper will be released soon; until then, cite the software:

```bibtex
@software{openrua,
  author  = {{Terminal World Labs}},
  title   = {OpenRUA},
  year    = {2026},
  url     = {https://github.com/terminalworld/OpenRUA},
  license = {Apache-2.0}
}
```

## License

Apache-2.0
