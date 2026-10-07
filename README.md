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
its **original terminal**, or run **recorded benchmark experiments**. Shared
chat lets you send follow-up instructions, inspect tool activity, manage the
queue, and revisit saved observations without starting a new robot each turn.

- [Start a shared session](examples/shared-session.md): OpenRUA TUI, browser, or plain CLI.
- [Use the agent's original terminal](examples/first-task.md): launch a task with `openrua run`.
- [Connect your robot](docs/your-own-robot.md): describe its ROS&nbsp;2 interfaces in a profile.
- [Run experiments](docs/running-experiments.md): fresh trials, benchmark scoring, and recorded artifacts.

> OpenRUA turns your robot into a coding project: your agent explores it
> like a live codebase, pulls sensor streams into files for reading, and
> runs commands and programs to move it.
>
> *Start playing with your robot like you code a project :)*

## Quick start

### Chat with a robot

Use a Linux host with Docker or a supported Podman setup and an existing
Claude Code or Codex login ([installation](docs/install.md)).

```sh
pip install -U openrua
openrua
```

The keyboard-first TUI guides environment selection and prepares missing images.
Each launch creates a new session; no name is required. Type an instruction and
press **Enter**, for example:

> Inspect the workspace documentation and describe the scene without moving the robot.

Use `/files` to browse saved observations and code, `/resume` to find earlier
sessions, and `/end` to stop execution while keeping your work. **Ctrl+D** on an
empty input detaches while the session continues running.

See the [walkthrough](examples/shared-session.md) for setup and continuous tasks,
[terminal guide](docs/terminal.md) for controls, and
[validation scope](docs/sessions.md#validation-scope) for what has been tested.
Shared chat remains experimental.

Native Claude Code or Codex login is the default. To use an API key, explicitly
select it in `openrua --setup` or through the
[API configuration guide](docs/install.md#explicit-api-authentication).
OpenRUA never switches to API billing after a login error.

### Run a benchmark and make a demo

For researchers, the CLI runs fresh benchmark trials and saves the code,
observations, transcripts, scores, and configuration needed to inspect a run.
The following example runs **one CaP-Bench Lift trial**, records its cameras,
and renders a video with terminal commands beside the robot views.

Install the demo dependencies and prepare the environment once (Docker must
be running; an existing native Claude Code login is reused):

```sh
pip install -U 'openrua[demo]'
claude login  # only if not already signed in
openrua build --bench capbench
openrua build sandbox --distro humble --agent claude-code
openrua build proxy --agent claude-code
openrua doctor panda --sim robosuite --bench capbench --agent claude-code
```

Then run and render:

```sh
openrua bench --config capbench --run-id lift-demo \
  --task-suite capbench_lift --task-ids 0 --seeds 0 \
  --operator agent --record --record-every 4 --wall-clock-min 10
openrua demo runs/capbench/lift-demo/trials/capbench_lift-0/seed0 --speed 4
```

The video is saved as `demo.mp4` in that trial directory. Its `result.json`
reports the benchmark verdict, and `provenance.json` records the resolved
configuration and software versions. This small, recorded run demonstrates the
method; reproducing the paper's aggregate results requires its full task/seed
sets, model settings, and evaluation budgets. Recording also adds rendering cost.
The bundled CaP-Bench configuration selects Claude Opus 5 with high reasoning
effort; `--agent` and `--model` can select another available configuration.

Native login requires the corresponding host CLI. See [installation](docs/install.md)
for prerequisites and credentials. To check the benchmark setup without a model
call, use the same `bench` command with `--operator none` and omit recording;
this checks startup and interfaces, not task success. See
[running experiments](docs/running-experiments.md) for full runs and recorded
replays of existing trials, and the [paper](https://arxiv.org/abs/2610.02459)
for the experimental protocol.

## Choose your interface

| Interface | Entry | When you leave |
|---|---|---|
| OpenRUA TUI | `openrua --name chat-demo` | Ctrl+D detaches; the shared session continues |
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

The default terminal uses **Enter** to send and **Ctrl+J** for a newline.
Use `/tools`, `/files`, and `/queue` to open details on demand. Configuration,
history, and confirmations use arrow keys, Enter, and Escape; no mouse is needed.
`--tui pi` remains an explicit alias. See the [keyboard guide](ui/terminal/README.md).

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
  the session. Switching them inside a running conversation is not implemented.
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
file tasks and two-turn shared-chat checks. They currently require explicitly
configured API access; native subscription reuse and robot tasks remain
unvalidated. They are not default setup choices.

Bring your own agent. An agent is a manifest (how to install its CLI in the sandbox, which
hosts it talks to, how it logs in) and a small hooks class (how to
launch it); everything else is optional. Pass yours as a path
(`--agent ./my-agent.yaml`) or send a pull request;
`openrua agents` lists what is available and what each can do. See
[docs/agents.md](docs/agents.md).

## Results

Detailed results and analysis are available in our [paper](https://arxiv.org/abs/2610.02459).

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
coordinates messages and events; `terminal/`, `ui/terminal/` and `web/` provide client interfaces.
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
