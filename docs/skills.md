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

The agent's own terminal keeps its native skill interface. Terminal and browser
chat share the same conversation adapter. `exec` uses the same
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
`bench` retains its isolated workspaces and does not mount your personal
daily-use skills.

## Review completed project work

In a live project conversation, use `/project` to browse saved project and
personal skills, read review reports and diffs, or review completed work once.
The agent's native `/skills` command remains available separately. The project
view lists saved files; it does not claim a method has been loaded or verified.

Before starting a review, check its agent, model, effort, prepared image and worker time
limit. Confirm starts an independent model task using the configured native login
or API. It excludes the active conversation, runs outside the robot network, and
returns proposals. Your conversation remains usable. Task progress appears below
the conversation; `/project` lets you refresh or cancel it. Leaving the terminal
or ending the robot session does not cancel the review or replay it.

After the robot session ends, inspect tasks from its project directory:

```sh
openrua skills tasks
openrua skills tasks TASK_ID --json
openrua skills tasks TASK_ID --cancel
```

These commands do not start a robot. Reports remain available through
`openrua skills reviews`. Cancellation is cooperative: the worker checks the
request and cleans up its own resources. A crashed owner is shown as unknown
and is never automatically restarted.

The report menu shows each proposed method and the full diff and evidence.
For a new skill, read the proposal and confirm **Import new skill**. Its contents
are checked against the preview, file permissions are preserved, and an existing
project method is never overwritten. For changes to an existing skill, select
**Prepare a merge request**. The request is placed in your composer, preserving
an existing draft; review it before sending it to the native agent. Current edits
and conflicts are resolved in the agent's ordinary editing and approval workflow.
Preparing the request does not send it, edit skills or move the robot.

## Periodic reviews

First run and inspect a manual review. When its scope and settings suit the
project, open `/project` and select **Periodic review schedules**. Choose a name,
time, days and timezone, check the agent/model/effort, image and worker limit,
then confirm **Enable schedule**. No periodic plan exists until you create one.
The background task uses your configured native login or API; its execution
time limit is not a monetary cap.

The scheduler belongs to OpenRUA and does not require systemd, cron or an
administrator. Closing the chat terminal keeps it running independently of the
robot conversation. After the computer shuts down, starting OpenRUA normally
restores enabled plans. It does not install an operating system startup service
or run while the computer is off. Several missed times combine into at most one
review per plan; the next time is calculated in its selected IANA timezone.
Daily and weekly calendars follow that timezone's daylight-saving rules.

Each occurrence starts the same isolated, project-scoped review described above.
No new eligible records, or the same completed inputs, are skipped without a
model task. Ordinary failures are recorded and wait for the next time. Login or
quota problems pause the plan. Uncertain execution also pauses it so you can
inspect the existing task rather than replay it. Fix the issue, then resume.
Reports and proposals use the existing explicit adoption workflow.

The project menu lists its plans, next time, current task, latest result and
report. Editing a plan preserves a running review's settings. Pausing or removing
it affects future reviews; an active review continues. To cancel it, use the
**Review tasks** menu or `skills tasks TASK_ID --cancel`.

The same controls work from the project directory without starting a robot:

```sh
openrua skills schedule add --name "Review project methods" \
  --time 21:30 --days mon,tue,wed,thu,fri --timezone Asia/Shanghai \
  --agent codex --effort high --wall-clock-min 5 --enable
openrua skills schedule list
openrua skills schedule show SCHEDULE_ID
openrua skills schedule edit SCHEDULE_ID --time 20:00
openrua skills schedule pause SCHEDULE_ID
openrua skills schedule resume SCHEDULE_ID
openrua skills schedule remove SCHEDULE_ID
```

Omit `--enable` to save a paused plan. For unattended project writes, pass
`--trust-workspace` explicitly. Creation accepts an optional stable `--id` for
retrying an unconfirmed request. Resuming a paused plan starts from its next
future time rather than replaying the period it was paused.

Background status and controls apply to all enabled plans in the selected
OpenRUA user directory:

```sh
openrua skills scheduler status
openrua skills scheduler stop
openrua skills scheduler start
```

Stopping the scheduler prevents new work and lets the current bounded review
finish. An explicit stop is retained across ordinary startup until you start it
again. A project menu cannot restart this deliberately stopped shared owner;
its plan remains saved and the menu shows the stopped background status.
Plans and private scheduler/task state live under the selected OpenRUA user
directory. Reports remain with their project. Keep project paths available;
moving or revoking trust pauses the affected plan rather than choosing a new
folder. This feature does not schedule robot movements.

## Manual review commands

To look back across finished conversations and exec tasks, run a separate
review from the project directory:

```sh
openrua skills review
openrua skills review --since 2026-10-01 --agent codex --wall-clock-min 5
openrua skills reviews
openrua skills reviews REVIEW_ID
```

This is an explicit model task using the selected agent's native login or
configured API authentication. It defaults to the project's saved agent/model,
then user settings. The review image must already contain that agent. Prepare
the default image and model proxy once if needed:

```sh
openrua build sandbox --agent codex
openrua build proxy
```

Use `--image openrua-sandbox-humble` for a prepared Humble image, or name your
own compatible image. There is no robot configuration requirement. Both agents
use their existing headless plugin, so native account limits still apply.

The input includes only closed conversations and completed exec records whose
recorded project path matches this directory exactly, plus selected project
text and existing project skill files. Active conversations and other projects
are excluded. `--records DIR` adds an exec record location while retaining the
same ownership check. In a Git project, ordinary context follows Git's tracked
and unignored files; other directories use a bounded text-file walk. Hidden
project context is omitted except for skill files and their policy.

The default limits select 20 recent records, 32 MiB of input, 2 MiB per file,
2,000 files, and 10,000 recent events per conversation. The manifest lists
captured hashes, omissions and truncation. Files are checked individually
while read; this is not an atomic snapshot of a changing project.

The worker sees a read-only input snapshot and a separate writable staging
directory. It has no robot or host network, source-project mount, host devices or
Docker socket. Model access travels over a Unix socket to the selected agent's
fixed model proxy; web search and account connectors are disabled. The worker
execution time is capped. `--max-turns` requests the native cap when supported;
it is not an enforced Codex turn, token or monetary budget.

Review results live under `.openrua/skill-reviews/REVIEW_ID/`:

| File | Purpose |
|---|---|
| `report.md`, `report.json` | outcome, source record references, proposed changes and output problems |
| `manifest.json` | captured input paths and hashes, limits and omissions |
| `baseline/`, `proposed/`, `changes.diff` | existing skill snapshot, candidate files and their differences |
| `transcript.jsonl`, `stderr.log` | this review's native execution output |

The report shows files that changed in the source project during review.
Snapshots and candidates retain file permissions, including executable scripts;
permission-only changes are included in the report and diff.
Candidates are never applied automatically, even when no concurrent edit is
detected. Inspect the diff and evidence, then edit an existing method through
your normal Git workflow. To import a new candidate without overwriting files:

```sh
openrua skills add .openrua/skill-reviews/REVIEW_ID/proposed/.agents/skills/camera-check
```

The selected record and project input copies remain private under
`~/.openrua/skill-reviews/REVIEW_ID/input/`, outside the project repository.
If you select another `--home`, keep that directory and its review state outside
the project; overlapping storage is refused before input capture.
Read a relative evidence path with:

```sh
openrua skills reviews REVIEW_ID --input manifest.json
openrua skills reviews REVIEW_ID --input records/sessions/SESSION_ID/conversation.json
```

Temporary authentication and working copies are removed after completion,
failure or interruption. Partial candidates and reports remain inspectable;
an owner that exits without finalizing appears as `unknown` and is not replayed.
No eligible records means no credential preparation or model call. Identical
inputs with a completed review skip another model call unless `--force` is
given. Check for changed methods and current machine assumptions before using
any candidate. Periodic scheduling remains a separate step and is not enabled
by running a review or choosing an accumulation policy.
