---
summary: Keep a robot coding project in your directory and continue it across conversations
read_when:
  - You want to continue yesterday's robot programs in a new conversation
  - You want to choose a workspace or resume conversations for one project
---

# Robot projects

Open OpenRUA in the directory where you want to work:

```sh
mkdir cup-sorting
cd cup-sorting
openrua
```

The first time, the terminal shows the full directory path and asks whether
you trust it. Your coding agent can read, change and execute files there;
OpenRUA also stores the project's environment selection and run records.
Choose **Yes** to use it, choose another directory, create a directory, or
quit. Trust is remembered for that exact directory, not its parent or children.
Use `openrua --workspace /path/to/project` to select a different directory.
For a noninteractive launch, explicitly accept it with `--trust-workspace`.

Then select a robot and start as usual. Your directory is mounted directly
at `/workspace` in the agent's terminal. OpenRUA preserves its permissions
and existing files. It adds project guidance to `AGENTS.md` once, retaining
any existing instructions, and keeps generated files under `.openrua/`.

## Continue your work

End the session, return to the same directory later, and run `openrua` again.
This starts a **new conversation** with the same project files. The project's
robot, simulator, benchmark and agent choices are remembered locally; explicit
command arguments override them. Authentication stays in OpenRUA's private
user directory.

The agent is guided to read existing code and notes before working, organize
files, and record findings, dependencies, checks and next steps. Suggested
directories are:

```text
cup-sorting/
  AGENTS.md                project instructions, including OpenRUA guidance
  scripts/                 maintained project programs
  notes/                   findings, progress and program entry points
  observations/            saved sensor data, identified by acquisition time
  .openrua/
    settings.json          this project's environment selection
    runs/<session>/        resolved configuration and robot logs for each run
```

The suggestions do not require restructuring an existing repository. Use Git
for maintained code and notes; choose which observations and generated files
to track according to the project.

The current connection's machine description, documentation and starter tools
are mounted read-only at `/robot`. Read `/robot/README.md` or run a tool by
its full path, for example:

```sh
python3 /robot/tools/perception/cam_snap.py agentview observations/current.png
```

Each connection gets its own generated materials. Reopening a project does
not replace its programs, notes, tools or historical `machine.yaml`. Current
machine facts come from `/robot/machine.yaml`.

Project files outlive containers. Declare the dependencies your programs need
so their environment can be rebuilt; an installation made only inside a
container does not survive its shutdown. Acquire current observations when
continuing: saved images and measurements describe the time they were taken.

## Resume a conversation

```sh
openrua --resume             # conversations in this directory
openrua --resume ID          # a particular conversation in this directory
openrua --resume --all       # conversations across all project directories
openrua --resume ID --all    # explicitly select one from another project
```

In the TUI, `/resume` opens the same project-scoped list; `/resume --all` or
Tab in the picker shows all projects. The list displays each conversation's
project. It matches the exact canonical directory, including when you entered
that directory through a symlink. A repository's subdirectory is a separate
workspace when you launch there.

An active conversation reconnects to its current owner. A confirmed ended
project conversation starts fresh robot resources and continues its native
agent conversation. Startup uses the recorded environment selection and
retained profile; the native conversation's model and permissions are kept.
An unused empty conversation can receive a fresh native thread; it has no
earlier messages to restore.
Past actions are not replayed, and previously queued instructions are withdrawn.
Before the next ordinary user instruction, the agent receives a recorded reminder
to read the current machine description and observe the scene. Uncertain prior
execution still requires reconciliation.

Older sessions without a project identity and sessions with unknown execution
status remain available as read-only history under `--resume --all`. Starting
another conversation in a project always works independently of restoring an
old conversation.

Conversation journals, execution endpoints and native profiles remain under
`~/.openrua/sandboxes/<session>/`; startup logs remain under
`~/.openrua/launches/<session>/`. `openrua clean` can delete those private
session records after shutdown. It retains the external project directory.

`openrua agent` and `openrua up` use the same project-directory convention.
Without an explicit `--name`, `agent` looks for a robot running in the current
project. Its native terminal has its own conversation history; the OpenRUA
resume picker lists shared conversations. Recorded `exec` and benchmark trials
continue to create fresh, independent workspaces under their run directories.
