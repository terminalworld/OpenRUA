<div align="center">

# OpenRUA

**Turn your robot into a coding project.**

Let your Claude Code or Codex control any robot, real or simulated.

*Through the standard ROS&nbsp;2 CLI and client library, without relying on any VLA model.*

[![CI](https://github.com/terminalworld/OpenRUA/actions/workflows/ci.yml/badge.svg)](https://github.com/terminalworld/OpenRUA/actions/workflows/ci.yml)
[![arXiv](https://img.shields.io/badge/arXiv-2610.02459-b31b1b.svg)](https://arxiv.org/abs/2610.02459)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue)](pyproject.toml)
[![ROS 2](https://img.shields.io/badge/ROS%202-22314E?logo=ros&logoColor=white)](docs/architecture.md)
[![License](https://img.shields.io/badge/license-Apache--2.0-green)](LICENSE)<br>
[![Stars](https://img.shields.io/github/stars/terminalworld/OpenRUA)](https://github.com/terminalworld/OpenRUA/stargazers)
[![Forks](https://img.shields.io/github/forks/terminalworld/OpenRUA)](https://github.com/terminalworld/OpenRUA/forks)
[![Watchers](https://img.shields.io/github/watchers/terminalworld/OpenRUA)](https://github.com/terminalworld/OpenRUA/watchers)

https://github.com/terminalworld/OpenRUA/releases/download/v0.0.7/libero10-mugs.mp4

*Codex (GPT-6 Astra) on LIBERO-10, "put the yellow and white mug on the left plate and put the white mug on the right plate": the commands it typed on the left, the robot's cameras on the right, task success. More in [docs/demos.md](docs/demos.md).*

</div>

A [robot-use agent](https://web.mit.edu/phillipi/www/writing/robot-use-agents.html) (RUA)
uses a robot just as a computer-use agent uses a computer.

OpenRUA connects off-the-shelf coding agents to robots through their native
ROS&nbsp;2 interfaces. The agent works in a workspace containing robot
documentation and starter tools, writes perception and control programs, and
uses execution feedback to continue the task.

> Your agent explores the robot like a live codebase, pulls sensor streams
> into files for reading, and runs commands and programs to move it.
>
> *Start playing with your robot like you code a project :)*

## Why OpenRUA

- **Use your coding agent.** Work with Claude Code or Codex and your existing
  account or API key. The agent uses standard `ros2` commands and client
  libraries such as `rclpy`.
- **Start with a prepared environment.** OpenRUA prepares the agent environment,
  simulators and configured robot drivers in containers, keeping their
  dependencies separate from your host setup.
- **Keep work in your project.** Programs, saved observations, notes and
  reusable methods stay available across conversations, for real robots and
  simulation.
- **Work interactively or run recorded tasks.** Chat from the terminal or
  browser, inspect outputs and watch cameras. Use the same project for a task
  on its own, or run isolated benchmark trials.

## Quick start

Use a **Linux host with Python 3.10 or newer**. You need Docker and a supported
coding-agent account or API key; OpenRUA guides Docker setup and agent sign-in
when they are missing. The agent CLI does not need to be installed on the host.
See [installation](docs/install.md) for requirements and authentication.

```sh
pip install -U openrua
mkdir my-robot-project
cd my-robot-project
openrua
```

1. Trust your project directory, or choose another one.
2. Choose **Simulation** and a simulation scene or benchmark environment,
   then choose your coding agent, model and authentication settings.
3. Review the settings and choose **Save project and start**. The first run
   prepares the required software, which can take a few minutes.

Once the robot is ready, try:

> Look through the cameras and describe the scene, without moving the robot.

For hardware, choose **Real robot → Connect a new robot**, or run
`openrua connect`. Your agent works through setup with you; see
[connect a real robot](docs/connect.md).

### Keep working in the same project

Opening `openrua` again starts a new conversation in this directory. Use
`openrua --resume` or `/resume` to find its previous conversations;
`--resume --all` includes other projects. `/setup` lets you configure a fresh
conversation with another environment or agent.

`/watch` opens live cameras, `/files` browses saved files, `/project` opens
skills and reviews, and `/help` lists commands and keyboard controls. Model
and permission choices come from the selected agent.

In the terminal, `/quit` opens the resource choice. **No**, selected by default,
stops the agent and any simulator or drivers OpenRUA started. **Yes** keeps
them running in the background. Your project files and chat history are kept
either way. See [terminal controls](docs/terminal.md) and
[project history](docs/projects.md#resume-a-conversation) for the full behavior.

## Ways to work

| Use | Entry | Guide |
|---|---|---|
| Shared terminal chat | `openrua` | [Shared session](examples/shared-session.md) |
| Browser or plain text chat | `openrua --gui` or `openrua --cli` | [Sessions and reconnection](docs/sessions.md) |
| The agent's own terminal | `openrua agent` | [First task](examples/first-task.md#1a-the-agents-own-terminal) |
| One recorded task in your project | `openrua exec "<task>"` | [Project tasks](docs/projects.md#run-a-task-without-the-interactive-interface) |
| Isolated benchmark trials | `openrua bench --config <name>` | [Running experiments](docs/running-experiments.md) |

To join an existing shared conversation from another client, use its session ID
with `--name`; the [session guide](docs/sessions.md) covers browser access,
reconnection and resource shutdown. Closing a browser page leaves its session
running.

## Supported agents and environments

| Agent | Status |
|---|---|
| [Claude Code](https://claude.com/claude-code) | Supported (`--agent claude-code`) |
| [Codex](https://github.com/openai/codex) | Supported (`--agent codex`) |
| [Kimi Code](docs/agents/kimi.md) | Preview (`--agent kimi`) |
| [ZCode](docs/agents/zcode.md) | Preview (`--agent zcode`) |

Kimi Code and ZCode have documented authentication and feature limits in their
guides. [Add your own agent](docs/agents.md) through a configuration file and
its hooks; `openrua agents` lists available capabilities.

Connect real robots through their ROS&nbsp;2 interfaces, or start with bundled
robosuite, ManiSkill, RoboTwin, CALVIN and VLABench environments. The
[environment catalog](docs/environments.md) lists every bundled robot,
simulator and benchmark, including LIBERO variants, CaP-Bench and RoboCasa.

## How it works

The agent works inside its sandbox with `ros2`, `rclpy`, robot documentation
and readable starter tools. It saves observations to files, writes programs,
and executes them against the robot's native interfaces. OpenRUA prescribes
no robot-task workflow or task-specific primitive library.

The project preserves the code, measurements and notes used in that work.
Useful methods can become ordinary skills that the native coding agent
reuses in later conversations. See [robot projects](docs/projects.md),
[reusable methods](docs/skills.md) and [architecture](docs/architecture.md).
The [paper](https://arxiv.org/abs/2610.02459) analyzes the resulting perception
and control behavior without relying on a VLA model.

## Paper results

Zero-shot simulation results from the
[paper](https://arxiv.org/abs/2610.02459), using Claude Code with Claude Opus 5
at high reasoning effort, without privileged simulator state or access to the
success predicate during execution:

| Benchmark | Evaluation scope | Success |
|---|---|---|
| CaP-Bench | Seven manipulation tasks | 99.0% |
| LIBERO-PRO | All four suites, both perturbation types | 87.0% |

[Full results and model comparison](docs/results.md) include the separate
LIBERO-10 comparison scope. See [demos](docs/demos.md) for recorded tasks.

## Documentation

Start with [installation](docs/install.md),
[your first shared session](examples/shared-session.md) or
[connecting a real robot](docs/connect.md).

The [documentation index](docs/README.md) links all guides, including native
agent setup, project files, skills, recorded tasks, demos and extension APIs.
[Project news](docs/news.md) preserves the announcements;
[CHANGELOG.md](CHANGELOG.md) records release changes and upgrade notes.

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

[Apache-2.0](LICENSE)

## Feedback

Try OpenRUA with your coding agent and robot, and tell us how it goes.
Bug reports, confusing steps, and ideas for improving the experience are welcome
in [GitHub Issues](https://github.com/terminalworld/OpenRUA/issues).
If something fails, include your OpenRUA version, agent, environment, and the
command or steps that led to it. Please remove credentials and private data
from any logs you share.
