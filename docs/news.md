---
summary: Project announcements and release milestones
read_when:
  - You want the history of OpenRUA features and releases
---

# Project news

These announcements describe their releases. For current behavior, use the
[terminal guide](terminal.md) and [shared-session guide](sessions.md).
[CHANGELOG.md](../CHANGELOG.md) records all release changes and upgrade notes.

- **[2026-10-24]** **Robot work belongs to the coding project.** Real-robot setup, simulation environment files and ongoing development stay together in the project directory. ([v0.23.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.23.0))
- **[2026-10-23]** 📂 **Run a task in the same robot project.** `openrua exec "<task>"` reuses your code and notes, with a separate record for each run. `bench` keeps isolated evaluation workspaces. ([v0.20.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.20.0))
- **[2026-10-22]** 📁 **Your robot is a coding project.** Open OpenRUA in a directory, keep programs and notes there, and continue in a new conversation the next day. Resume searches this project's history; `--all` shows every project. ([v0.19.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.19.0))
- **[2026-10-22]** 🧰 **Bring OpenRUA to your own robot, starting from a bare computer.** Missing Docker or an agent login? `openrua` sets them up with you, and your coding agent connects the robot step by step. ([v0.18.2](https://github.com/terminalworld/OpenRUA/releases/tag/v0.18.2))
- **[2026-10-22]** 👀 **Watch your robot through its own eyes.** The browser shows its cameras live while the agent works, and every real-robot `openrua exec` run is recorded, ready to become a demo video with `openrua demo`. ([v0.17.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.17.0))
- **[2026-10-21]** 🎯 **Hand your agent a task and let it work on its own.** `openrua exec` runs it in simulation or on a real robot and keeps a full record of the run. ([v0.14.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.14.0))
- **[2026-10-21]** 🦾 **Your coding agent can now set up a real robot for you.** Run [`openrua connect`](connect.md): Claude Code or Codex works out what the robot is, builds its ROS&nbsp;2 drivers and writes its config with you, and OpenRUA checks every step. ([v0.13.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.13.0))
- **[2026-10-21]** ⌨️ **Talk to your robot the way you talk to Claude Code and Codex.** Send a follow-up while the agent works, switch `/permissions` and `/model` on the fly, and press Esc to stop the agent and the arm's motion together. ([v0.13.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.13.0))
- **[2026-10-21]** 🤖 **Real robots, ready to go.** Each robot describes itself in its own config, and OpenRUA builds its drivers and its own sandbox the first time you start it. ([v0.11.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.11.0), [v0.12.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.12.0))
- **[2026-10-06]** 💬 **A keyboard terminal for your robot** becomes the default way to chat with it. ([v0.6.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.6.0))
- **[2026-10-03]** 👥 **Shared robot conversations:** one session you can join from the terminal, the browser or a script. ([v0.1.0](https://github.com/terminalworld/OpenRUA/releases/tag/v0.1.0))
- **[2026-10-01]** 📄 **The paper is out:** [OpenRUA: Robot-Use Agents Are Zero-Shot Visuomotor Policies](https://arxiv.org/abs/2610.02459).
- **[2026-09-14]** 🚀 **OpenRUA is live.** `pip install openrua` and give Claude Code or Codex a robot, simulated or real. ([v0.0.7](https://github.com/terminalworld/OpenRUA/releases/tag/v0.0.7))
