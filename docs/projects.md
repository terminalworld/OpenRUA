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
Use `openrua -C /path/to/project` (also `--cd` or `--workspace`) to select a different directory.
For a noninteractive launch, explicitly accept it with `--trust-workspace`.

Then select a robot and start as usual. Your directory is mounted directly
at `/workspace` in the agent's terminal. OpenRUA preserves its permissions
and existing files. It adds project guidance to `AGENTS.md` once, retaining
any existing instructions, and keeps generated files under `.openrua/`.

## Save a project environment

Use the setup screen or save explicitly:

```sh
openrua config set --robot panda --sim robosuite --agent codex
openrua build
openrua
```

This copies the selected bundled robot and simulator declarations, including
required build files, into `configs/` in your project. Edit these maintained
files to change your scene or control configuration. Selecting a benchmark
also copies its declaration and dependencies. Existing files are retained.
Project settings store paths relative to this directory; YAML references
resolve relative to the file that declares them. Commit the maintained files
and `.openrua/settings.json` together. `.openrua/requirements.txt` records the
OpenRUA version used when the project was first saved; install it with
`python -m pip install -r .openrua/requirements.txt`. Update that dependency
explicitly when you choose to upgrade the project.

Command-line overrides on ordinary launches and `exec` apply to that run only.
Use `config set` or the setup screen to save choices. `config show` displays
this project's settings. Personal agent preferences and authentication use
`openrua config set --user`; they never select another project's robot.

For a real robot, run `openrua connect` here. Its `robot.yaml`, `driver/` and
`cameras/` are maintained in this project, from the first connecting turn
through later robot work. `--dir robot` selects a subdirectory inside the
project. See [connect](connect.md).

Copying the project preserves maintained inputs. A new machine still needs
the declared software dependencies, authentication and its own workspace
trust. Session history remains local to OpenRUA's user directory.

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

Reusable methods are separate from session handoff notes. See [skills](skills.md)
to save a method, find it in a later conversation, or choose whether the agent
may accumulate methods automatically, suggest them, or wait for your request.
The sandbox's working directory is `/workspace`; its HOME is separate so native
agents can distinguish project skills from personal ones. Save maintained
programs and notes in the project, rather than relying on `~` to name it.

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
If the recorded project directory is missing, its conversation remains
read-only; resume does not create an empty replacement project.

Conversation journals, execution endpoints and native profiles remain under
`~/.openrua/sandboxes/<session>/`; startup logs remain under
`~/.openrua/launches/<session>/`. `openrua clean` can delete those private
session records after shutdown. It retains the external project directory.

`openrua agent` and `openrua up` use the same project-directory convention.
Without an explicit `--name`, `agent` looks for a robot running in the current
project. Its native terminal has its own conversation history; the OpenRUA
resume picker lists shared conversations. Recorded `exec` and benchmark trials
have separate execution records, as described below.

## Run a task without the interactive interface

`exec` uses the same project files and saved environment selection as the
interactive interfaces, for simulated and real robots:

```sh
cd cup-sorting
openrua exec "Read the project notes and continue the sorting program"
openrua exec -C /path/to/project "Inspect the saved program"
```

Each invocation starts a new agent session. It reads existing instructions,
code and notes; it does not automatically continue an earlier conversation.
The current robot materials remain under `/robot`, and the project is
`/workspace`. Files written there remain after the robot and sandbox stop.
Explicit `--robot`, `--sim`, `--bench`, `--agent` and `--model` arguments
override the project's selection. The original `exec <robot> "<task>"`
spelling also works.

The saved model belongs to the project's agent. Selecting another agent uses
that agent's configured model unless `--model` explicitly selects one. Changing
only the version pin of the same agent keeps the saved project model.

For unattended first use, pass `--trust-workspace` explicitly, or trust the
directory interactively beforehand. Without either, a noninteractive launch
fails with instructions before changing the project.

Run records go to `.openrua/runs/<run id>/`, with a suffix if that name is
already taken. `--runs-root DIR` puts records under `DIR/<label>/<run id>/`
while keeping the same project workspace. See [recorded runs](running-experiments.md).

`bench` creates fresh, independent workspaces under its run directories and
keeps its original prompt and starter template. Use it for isolated evaluation,
including a single task and seed. `exec --bench NAME` selects that benchmark's
scene within your project; the project's files remain available to the agent.
An explicit task sentence takes precedence over the scene's default task,
while its success predicate stays the same. A new task keeps the project's
saved scene; `--bench ""` selects the simulator's native scene instead.
