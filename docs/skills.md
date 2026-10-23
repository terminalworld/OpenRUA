---
summary: Save useful robot methods and reuse them in a new conversation
read_when:
  - You want your agent to remember a reusable method
  - You want to inspect skills or control how a project accumulates them
---

# Reusable methods

Work in your robot project as usual. When you and the agent work out a useful
method, you can say:

> Keep this camera check as a project skill, including the script and how to check its output.

The agent uses ordinary `SKILL.md` files with optional scripts and references.
The next conversation in this directory can discover and reuse them. OpenRUA
guides the agent to capture useful methods during normal work, improve existing
ones when there is evidence, and briefly report changes. There is no quota of
skills to create and no separate model task on every message or session exit.

## Find and use a method

Ask naturally for the work you want. The native coding agent selects relevant
skills from their descriptions. You can also select one explicitly:

- Claude Code: `/skill-name`, optionally followed by instructions.
- Codex: `$skill-name` in your message. In OpenRUA's shared conversation,
  `/skills` lists native skills without starting a model turn; `$skill-name`
  passes an unambiguous skill through Codex's structured skill input. For
  same-name definitions, name resolution stays with the native agent.

The agent's own terminal keeps its native skill interface. Terminal, browser
and plain chat share the same conversation adapter. `exec` uses the same
project and personal skill files, with the agent's own headless invocation.
Simulation and real robots follow the same file rules.

To inspect saved files without starting a robot or signing in:

```sh
openrua skills list
openrua skills list --json
openrua skills show camera-check
```

The file list reports names, descriptions, sources and malformed metadata.
Native discovery decides which skills the active agent can actually use;
`/skills` in a shared Codex conversation queries that agent directly. A saved
method still needs its dependencies and current machine assumptions checked.

## Choose how methods accumulate

You can tell the agent your preference, for example, "From now on, suggest
new skills before saving them." It is guided to update this project's policy.
The same setting is available from the host command line:

```sh
openrua skills policy          # show this project's current policy
openrua skills policy auto     # allow useful in-task additions and improvements
openrua skills policy suggest  # propose changes and wait for agreement
openrua skills policy manual   # save or improve only when explicitly requested
```

The initial policy is `auto`. These commands change the agent's project
instructions, stored in `.openrua/skills.json`. The agent is told to reread
that file before writing skills, including during an existing conversation.
Existing skills remain usable in every accumulation mode. To control implicit
invocation of one skill, use the native agent's skill metadata or settings.
Writing project settings or importing files requires the same workspace trust
as starting work. For unattended use, add `--trust-workspace` explicitly:

```sh
openrua skills -C /path/to/project --trust-workspace policy manual
```

The policy guides the agent; it does not impose a filesystem write barrier.
An explicit editing command can still change an ordinary skill file.

## Project and personal skills

Project methods live in `.agents/skills/<name>/SKILL.md`. OpenRUA connects
Claude Code's `.claude/skills/` to the same source. Existing native directories
are preserved; when both already contain files, missing names are linked on
project startup. Independent definitions with the same name remain separate
and are reported in `/robot/skills.md` for review. Moving, renaming or removing
linked methods should be followed by checking the native directories.

Skills belong in the project's Git repository alongside maintained code and
notes. Track the links as well as their targets. Review changes with `git diff`
and restore earlier content through the project's normal Git workflow.

Claude Code also gets a small `.claude/rules/openrua-skills.md` entry so this
guidance loads when an existing `CLAUDE.md` takes precedence over `AGENTS.md`.
Your `CLAUDE.md` and existing rule text are preserved.

Personal methods are separate, under `~/.openrua/skills/` (or the selected
`--home`). They are mounted into each supported agent's native personal skills
directory for project sessions, independent of its temporary authentication
profile. Same-name project and personal methods follow the native agent's
discovery and selection rules; OpenRUA does not invent a shared precedence.
Promote a method explicitly when it is useful across projects:

```sh
openrua skills add .agents/skills/camera-check --personal
openrua skills list --personal
```

You can import one existing native skill with `openrua skills add <directory>`
or add `--personal`. Imports include its scripts and references and refuse to
overwrite an existing name. Host-wide native skill directories are not copied
wholesale; choose the methods you want available. Credentials and account
connector settings remain separate from local skill files.

## Keep evidence with the method

A useful skill records when to use it, dependencies, machine assumptions,
entry points, steps, checks, and the run or files supporting those checks.
Current images, object poses and one-off logs belong in project observations
or run records. Re-observe the scene before acting. Saving a method does not
verify it, and a format accepted by another agent does not install its tools.

OpenRUA does not move a robot solely to validate a saved skill. Capture the
checks performed during normal work and identify remaining checks explicitly.
Background cross-session review and scheduling are separate capabilities;
this release does not start either automatically. `bench` retains its isolated
workspaces and does not mount your personal daily-use skills.
