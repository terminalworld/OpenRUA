---
summary: Find the guide for installation, robot projects, shared sessions, recorded tasks or extensions
read_when:
  - You are choosing which OpenRUA guide to read
---

# OpenRUA documentation

Start with [installation](install.md), then follow
[your first shared robot session](../examples/shared-session.md).
For hardware, use [connect a real robot](connect.md).

## User guides

| page | read when |
|---|---|
| [docs/install.md](install.md) | setting a machine up: Docker, your agent's login, images, doctor |
| [examples/first-task.md](../examples/first-task.md) | your first task on the simulated Panda: one recorded run with `exec`, and the agent's original terminal with `openrua agent` |
| [examples/shared-session.md](../examples/shared-session.md) | TUI, browser and CLI chat, queue checks, reconnection, saved images and shutdown |
| [docs/terminal.md](terminal.md) | TUI shortcuts, expandable panels, and current limitations |
| [docs/sessions.md](sessions.md) | shared-session operations, API, architecture and validation scope |
| [docs/projects.md](projects.md) | persistent project files, workspace trust and project-scoped conversations |
| [docs/connect.md](connect.md) | connecting a real robot with your coding agent, and what each step checks |
| [docs/your-own-robot.md](your-own-robot.md) | describing your robot in one profile |
| [examples/real-robot.md](../examples/real-robot.md) | the same flow on a real ROS&nbsp;2 arm |
| [docs/setup-validation.md](setup-validation.md) | which combinations the setup menu offers, and the checks behind them |
| [docs/simulation.md](simulation.md) | the simulator checkouts and GPU rendering |
| [docs/podman.md](podman.md) | machines without Docker |
| [docs/demos.md](demos.md) | recorded trials rendered as videos, one per task type |
| [docs/running-experiments.md](running-experiments.md) | `openrua exec` and `openrua bench`, the `runs/` layout, every record field, replays and demo videos |
| [docs/cli.md](cli.md) | every verb and flag, exit codes (generated) |
| [docs/config.md](config.md) | every config key (generated) |
| [docs/agents.md](agents.md) | adding a coding agent |
| [docs/architecture.md](architecture.md) | the units and the layering contract |
| [CONTRIBUTING.md](../CONTRIBUTING.md) | conventions for code, names and docs |
| [CHANGELOG.md](../CHANGELOG.md) | release changes, upgrade notes, and how versions are numbered |

## Catalogs and project history

- [Robots, simulators and benchmarks](environments.md): supported environments and selection options.
- [Reusable methods](skills.md): project skills, review reports and recurring reviews.
- [Kimi Code](agents/kimi.md) and [ZCode](agents/zcode.md): preview support and authentication limits.
- [Paper results](results.md): benchmark scope and the full model comparison.
- [Project news](news.md): dated announcements.

## Extending OpenRUA

Read [adding an agent](agents.md), [architecture](architecture.md) and
[contributing](../CONTRIBUTING.md). Exact command options and config fields are
in the generated [CLI reference](cli.md) and [configuration reference](config.md).
