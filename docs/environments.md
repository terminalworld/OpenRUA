---
summary: Bundled robot types, simulators, benchmarks, and environment selection
read_when:
  - You want to check which robot, simulator or benchmark is available
  - You want to choose a scene or provide your own configuration
---

# Robots, simulators and benchmarks

Use this catalog to choose a starting environment for your coding project.
[Robot projects](projects.md) explains where its files live, and
[connecting a real robot](connect.md) covers guided setup for hardware.

## Choosing what to run

- **Robot and scene.** `--sim` selects the simulator; `--bench`, `--task-suite`,
  and `--task-id` select a benchmark scene. Without a benchmark selection or saved
  benchmark default, robosuite uses its native `Lift` scene.
- **Agent and model.** Choose `--agent` and optionally `--model` when starting
  the session. `/model <name>` changes the model from the next request; the
  agent cannot be switched inside a running conversation. Use `/setup` to start
  a fresh conversation with a different agent.
- **What the agent can reach.** A session you start sets no limits: the agent
  reaches the internet as on your machine. A recorded run (`openrua exec`,
  `openrua bench`) limits it to its model service by default. `--internet`,
  `--[no-]web-search` and `--[no-]account-connectors` set either
  ([what the agent can reach](install.md#what-the-agent-can-reach)).
- **Custom profiles.** Pass a file with `--robot`, `--sim`, or `--bench` to use
  your own environment in the TUI. It is validated and prepared without being
  replaced by a guided choice. See [your own robot](your-own-robot.md).
- **Defaults.** Save common choices with `openrua config set`; explicit command
  arguments override them. [Configuration](config.md) documents the fields.
- **Available extensions.** `openrua robots`, `openrua simulators`,
  `openrua benchmarks`, and `openrua agents` list the bundled choices;
  `openrua robots` lists available robot types as project starting points.
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
| *your robot* | any ROS&nbsp;2 arm or mobile manipulator, real | its own file; see [docs/your-own-robot.md](your-own-robot.md) |

`openrua robots` prints this list. Real-robot adaptation files live in your project.

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
environment); [docs/simulation.md](simulation.md) describes them.

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
