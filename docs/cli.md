---
summary: Every openrua verb, its arguments, and the exit codes
read_when:
  - You want the exact flags of a verb without running --help
  - A script needs to act on openrua's exit code
---

# Command line

Generated from the parsers by `scripts/render_docs.py`; edit the
`add_parser` of a verb, not this page.

```
usage: openrua [-h] [--version] [--home HOME] [--robot ROBOT] [--sim SIM]
               [--bench BENCH] [--task-suite TASK_SUITE] [--task-id TASK_ID]
               [--init-state INIT_STATE] [--task TASK] [--name NAME] [--agent
               AGENT] [-C WORKSPACE] [--trust-workspace] [--ros-domain
               ROS_DOMAIN] [--internet {open,model-only}] [--web-search |
               --no-web-search] [--account-connectors |
               --no-account-connectors] [--permissions MODE] [--model MODEL]
               [--gui | --tui UI] [--port PORT] [--setup] [--resume [ID]]
               [--all] <verb> ...

Open a robot conversation, or use ``openrua <verb> ...``.

positional arguments:
  <verb>

options:
  -h, --help            show this help message and exit
  --version             show program's version number and exit
  --home HOME           personal preferences, logins and session state
                        (default: $OPENRUA_HOME or ~/.openrua)

start or reconnect (without a subcommand):
  --robot ROBOT         robot name or profile path for a new session
  --sim SIM             simulator that embodies the robot (openrua
                        simulators); default: the benchmark's; a real robot's
                        file needs none
  --bench BENCH         benchmark whose world to load (openrua benchmarks);
                        default: the saved benchmark, else the simulator's
                        native scene
  --task-suite TASK_SUITE
                        scene suite (default: the profile's)
  --task-id TASK_ID     scene index (default: the profile's)
  --init-state INIT_STATE
                        episode seed / init state (simulated robots)
  --task TASK           task sentence to show the agent (real robots; a
                        simulated robot's comes from the scene)
  --name NAME           optional session name or ID; omitted for a new
                        automatic ID
  --agent AGENT         agent to open (default: the config's)
  -C WORKSPACE, --cd WORKSPACE, --workspace WORKSPACE
                        persistent robot project directory (default: the
                        current directory)
  --trust-workspace     explicitly trust the workspace without an interactive
                        question
  --ros-domain ROS_DOMAIN
                        ROS_DOMAIN_ID (default: the lowest one no running
                        robot uses, so concurrent robots never share a graph;
                        a real robot: its own domain, 0)
  --internet {open,model-only}
                        open, or only the agent's model service and login,
                        through a whitelist proxy (default: not limited)
  --web-search, --no-web-search
                        the agent CLI's own web search and fetch tools
                        (default: not limited)
  --account-connectors, --no-account-connectors
                        what your account brings beyond the CLI: its
                        connectors or apps, synced skills and plugins
                        (default: not limited)
  --permissions MODE    selected agent's native permission mode; the plugin
                        supplies its default; /permissions lists supported
                        choices
  --model MODEL         model for a new session (default: the agent's
                        configuration)
  --gui                 open the browser instead of terminal chat
  --tui UI              keyboard terminal frontend (default: pi, included)
  --port PORT           local service port for a new session (default: a free
                        port)
  --setup               edit this project in the keyboard setup menu
  --resume [ID]         resume a conversation in this workspace, optionally by
                        ID/name
  --all                 with --resume, show conversations from all project
                        directories
```

| command | does |
|---|---|
| `robots` | list the bundled robots |
| `simulators` | list the bundled simulators |
| `benchmarks` | list the bundled benchmarks (or one benchmark's suites and tasks) |
| `agents` | list the bundled agents |
| `build` | build simulator, sandbox, agent and proxy images |
| `update` | update one coding agent in an existing tool image |
| `exec` | the agent does one task on its own; the run is recorded |
| `up` | bring a robot up with a sandbox terminal on it |
| `connect` | connect a real robot, your coding agent doing the work |
| `serve` | own a robot and shared native conversation for local clients |
| `session` | inspect, queue messages, pause/resume or end a shared session |
| `agent` | your coding agent's own terminal, on a robot |
| `down` | power a robot and its terminal off, retaining session files |
| `bench` | run a benchmark: one trial per task and seed |
| `ps` | list the attempts running under a runs root (--all: stale claims too) |
| `demo` | render a recorded trial as a video (terminal + cameras) |
| `probe` | draft a robot profile from a live ROS 2 graph |
| `config` | view or save project settings; --user for personal preferences |
| `skills` | list reusable methods, import a skill, or set project accumulation |
| `login` | sign your coding agent in on this computer |
| `clean` | explicitly delete retained session files |
| `doctor` | check the install: docker, images, simulator, login |

Without a subcommand, an unnamed launch opens setup with a new session ID. An explicit name reconnects to the named shared session. `--gui` selects the browser, `--resume [ID]` selects history, and `--setup` edits the project settings. With no interface selected, non-interactive invocations print help.

## openrua robots

```
usage: openrua robots [-h] [--json]

Every robot shipped in the package. A file of your own elsewhere is not
listed; pass it as a path where a name is expected.

options:
  -h, --help  show this help message and exit
  --json      machine-readable output
```

## openrua simulators

```
usage: openrua simulators [-h] [--json]

Every simulator shipped in the package. A file of your own elsewhere is not
listed; pass it as a path where a name is expected.

options:
  -h, --help  show this help message and exit
  --json      machine-readable output
```

## openrua benchmarks

```
usage: openrua benchmarks [-h] [--json] [name]

Every benchmark shipped in the package. A file of your own elsewhere is not
listed; pass it as a path where a name is expected. With a name: that
benchmark's suites, each suite's task ids and sentences, and what a seed
means, for choosing --task-suite, --task-ids and --seeds.

positional arguments:
  name        a benchmark (name or path) to show in full

options:
  -h, --help  show this help message and exit
  --json      machine-readable output
```

## openrua agents

```
usage: openrua agents [-h] [--json]

Every agent shipped in the package. A file of your own elsewhere is not
listed; pass it as a path where a name is expected.

options:
  -h, --help  show this help message and exit
  --json      machine-readable output
```

## openrua build

```
usage: openrua build [-h] [--bench NAME] [--sim NAME] [--all] [-C WORKSPACE]
                     [<unit>] ...

Build simulator images from their declarations (--bench / --sim, repeatable;
--all: every bundled benchmark), or one of the agent-side images (sandbox,
agent, proxy), the ROS base, or a real robot's driver images. Bare `openrua
build` makes the sandbox and proxy images for the default agent and the
default benchmark's simulator image. The sandbox takes its install line from
the manifests of the agents named with --agent.

positional arguments:
  [<unit>]

options:
  -h, --help            show this help message and exit
  --bench NAME          benchmark whose simulator image to build (openrua
                        benchmarks); repeatable
  --sim NAME            simulator whose image to build, for its native scene
                        (openrua simulators); repeatable
  --all                 every bundled benchmark's image
  -C WORKSPACE, --cd WORKSPACE, --workspace WORKSPACE
                        project directory (default: the current directory)
```

| command | does |
|---|---|
| `base` | the ROS base image simulator images build on |
| `sandbox` | the agent terminal image |
| `agent` | the CLI image for model discovery and sign-in, without ROS |
| `driver` | the images a real robot's launch command uses, from the directories its config names (backend.build) |
| `proxy` | the whitelist proxy image (runs that limit the agent to its model service start one per agent's hosts) |

### openrua build base

```
usage: openrua build base [-h] [--distro DISTRO] [--tag TAG]

options:
  -h, --help       show this help message and exit
  --distro DISTRO  ROS 2 distro: jazzy | humble
  --tag TAG        image tag (default: openrua-sim-base-<distro>)
```

### openrua build sandbox

```
usage: openrua build sandbox [-h] [--agent NAME[@VERSION]]
                             [--preinstall PREINSTALL] [--distro DISTRO]
                             [--robot-uid ROBOT_UID] [--tag TAG]
                             [--base-image BASE_IMAGE] [--robot NAME|PATH]

options:
  -h, --help            show this help message and exit
  --agent NAME[@VERSION]
                        agent(s) to install, repeatable; @VERSION pins the
                        CLI, otherwise the current release (default: the
                        configured default)
  --preinstall PREINSTALL
                        install line to bake instead of the agents' manifests
  --distro DISTRO       ROS 2 distro: jazzy | humble (default: this project's
                        ros_distro, else jazzy)
  --robot-uid ROBOT_UID
                        container uid (default: the current user)
  --tag TAG             image tag (default: openrua-sandbox-<distro>)
  --base-image BASE_IMAGE
                        retain an existing terminal image and add the selected
                        install layer
  --robot NAME|PATH     a real robot whose config declares
                        backend.sandbox_install: its distro, image name and
                        install chain come from there
```

### openrua build agent

```
usage: openrua build agent [-h] [--agent NAME[@VERSION]] [--tag TAG]
                           [--robot-uid ROBOT_UID]

options:
  -h, --help            show this help message and exit
  --agent NAME[@VERSION]
                        agent name or manifest path, optionally pinned
                        (default: the configured agent)
  --tag TAG             image tag (default: derived from the CLI recipe and
                        user)
  --robot-uid ROBOT_UID
                        container uid (default: the current user)
```

### openrua build driver

```
usage: openrua build driver [-h] [--robot NAME|PATH] [--image IMAGE]

options:
  -h, --help         show this help message and exit
  --robot NAME|PATH  the real robot (default: this project's)
  --image IMAGE      only this one of them
```

### openrua build proxy

```
usage: openrua build proxy [-h] [--tag TAG] [--port PORT]

options:
  -h, --help   show this help message and exit
  --tag TAG
  --port PORT  listen port, baked in and labelled
```

## openrua update

```
usage: openrua update [-h] --agent AGENT --image IMAGE

options:
  -h, --help     show this help message and exit
  --agent AGENT  agent manifest or name with an explicit @version
  --image IMAGE  existing tool image; running containers retain their original
                 image
```

## openrua exec

```
usage: openrua exec [-h] [--robot ROBOT] [-C WORKSPACE] [--trust-workspace]
                    [--sim SIM] [--bench BENCH] [--task-suite TASK_SUITE]
                    [--task-id TASK_ID] [--seed SEED] [--agent AGENT]
                    [--model MODEL] [--run-id RUN_ID] [--runs-root RUNS_ROOT]
                    [--wall-clock-min WALL_CLOCK_MIN]
                    [--ros-domain ROS_DOMAIN] [--record [CAMERAS]]
                    [--no-record] [--internet {open,model-only}]
                    [--web-search | --no-web-search]
                    [--account-connectors | --no-account-connectors]
                    [task_or_robot] [task]

``openrua exec``: the agent does one task on its own, and the run is recorded.
As a coding agent's own non-interactive command runs it without its terminal,
this works in the current project: the robot comes up (simulated or real), the
agent gets the task and works until it is done or a budget ends, the robot
powers off, and records land under ``.openrua/runs/<run id>/`` (transcript,
result, provenance; ``openrua demo`` renders a simulated one; a real robot's
cameras are recorded too, unless --no-record). While the agent works, what it
says and each command it runs show as they happen; Ctrl-C stops it and powers
the robot off. A benchmark scene supplies a default task and judges success;
an explicit sentence is the agent's task, without changing the scene's success
predicate. A real robot's task is the sentence given, and at the end you are
asked whether it was done (your answer is the result; the agent's last words
are kept beside it). Like ``openrua bench``, which runs every task and seed of
a benchmark this way, the agent may reach only its model service unless the
limit flags say otherwise.

positional arguments:
  task_or_robot         task sentence, or a robot name/file followed by its
                        task
  task                  the task, as a sentence: a real robot's, or a
                        simulator's own scene's (default: the scene's task)

options:
  -h, --help            show this help message and exit
  --robot ROBOT         robot name or file (default: the project's, else the
                        benchmark's)
  -C WORKSPACE, --cd WORKSPACE, --workspace WORKSPACE
                        persistent project directory (default: the current
                        directory)
  --trust-workspace     explicitly trust the project without an interactive
                        question
  --sim SIM             simulator for a robot type (default: the project's,
                        else the benchmark's)
  --bench BENCH         benchmark whose scene to load (openrua benchmarks);
                        default: the project's; an empty value selects the
                        native scene
  --task-suite TASK_SUITE
                        scene suite (default: the scene's)
  --task-id TASK_ID     task in the suite (default: the scene's)
  --seed SEED           initial state of a simulated scene
  --agent AGENT         agent (default: the project's or configured one)
  --model MODEL         model (default: the project's or agent's)
  --run-id RUN_ID       the run's name (default: exec-<UTC time>)
  --runs-root RUNS_ROOT
                        record runs under DIR/<label>/ (default: the project's
                        .openrua/runs/)
  --wall-clock-min WALL_CLOCK_MIN
                        active minutes before the run stops (default: the
                        protocol's)
  --ros-domain ROS_DOMAIN
                        ROS_DOMAIN_ID (default: a free one; a real robot: 0)
  --record [CAMERAS]    record the cameras for openrua demo (names, or none
                        for all); a real robot's run is recorded anyway, two
                        frames a second
  --no-record           do not record a real robot's cameras
  --internet {open,model-only}
                        open, or only the agent's model service and login,
                        through a whitelist proxy (default: only the model
                        service, as in a benchmark run)
  --web-search, --no-web-search
                        the agent CLI's own web search and fetch tools
                        (default: only the model service, as in a benchmark
                        run)
  --account-connectors, --no-account-connectors
                        what your account brings beyond the CLI: its
                        connectors or apps, synced skills and plugins
                        (default: only the model service, as in a benchmark
                        run)
```

## openrua up

```
usage: openrua up [-h] [--sim SIM] [--bench BENCH] [--task-suite TASK_SUITE]
                  [--task-id TASK_ID] [--init-state INIT_STATE] [--task TASK]
                  [--name NAME] [--agent AGENT] [-C WORKSPACE]
                  [--trust-workspace] [--ros-domain ROS_DOMAIN]
                  [--internet {open,model-only}]
                  [--web-search | --no-web-search]
                  [--account-connectors | --no-account-connectors]
                  [--model MODEL]
                  [robot]

Bring a robot up (simulated: boot its container; real: join its graph) with a
sandbox terminal on it, then stay in the foreground; Ctrl-C powers it off.
Open a second terminal for `openrua agent`, the coding agent's own terminal on
it.

positional arguments:
  robot                 robot: a type or your robot's file (openrua robots),
                        by name or path; default: this project's selection,
                        else --bench's robot

options:
  -h, --help            show this help message and exit
  --sim SIM             simulator that embodies the robot (openrua
                        simulators); default: the benchmark's; a real robot's
                        file needs none
  --bench BENCH         benchmark whose world to load (openrua benchmarks);
                        default: the saved benchmark, else the simulator's
                        native scene
  --task-suite TASK_SUITE
                        scene suite (default: the profile's)
  --task-id TASK_ID     scene index (default: the profile's)
  --init-state INIT_STATE
                        episode seed / init state (simulated robots)
  --task TASK           task sentence to show the agent (real robots; a
                        simulated robot's comes from the scene)
  --name NAME           session handle (default: openrua)
  --agent AGENT         agent to open (default: the config's)
  -C WORKSPACE, --cd WORKSPACE, --workspace WORKSPACE
                        persistent robot project directory (default: the
                        current directory)
  --trust-workspace     explicitly trust the workspace without an interactive
                        question
  --ros-domain ROS_DOMAIN
                        ROS_DOMAIN_ID (default: the lowest one no running
                        robot uses, so concurrent robots never share a graph;
                        a real robot: its own domain, 0)
  --internet {open,model-only}
                        open, or only the agent's model service and login,
                        through a whitelist proxy (default: not limited)
  --web-search, --no-web-search
                        the agent CLI's own web search and fetch tools
                        (default: not limited)
  --account-connectors, --no-account-connectors
                        what your account brings beyond the CLI: its
                        connectors or apps, synced skills and plugins
                        (default: not limited)
  --model MODEL         model (default: the project's or configured agent's)
```

## openrua connect

```
usage: openrua connect [-h] [--dir DIR] [--agent AGENT] [--model MODEL]
                       [--distro {jazzy,humble}] [--name NAME] [-C WORKSPACE]
                       [--trust-workspace]

``openrua connect``: connect a real robot, with your coding agent doing the
work. A session opens whose terminal reaches this computer's docker, network
and devices, working in the robot's folder. Your agent finds out what the
robot is, builds and starts its ROS 2 drivers and writes the robot's config,
asking you for what only a person can do and before each command that changes
something; OpenRUA checks each step. When every check has passed, the session
moves to the robot's own terminal and the conversation goes on there.

options:
  -h, --help            show this help message and exit
  --dir DIR             adaptation subdirectory inside the project (default:
                        the project root)
  --agent AGENT         coding agent (default: the configured one)
  --model MODEL         model (default: the agent's)
  --distro {jazzy,humble}
                        the connecting terminal's ROS 2; the robot's own comes
                        from its drivers
  --name NAME           session ID (default: a new one)
  -C WORKSPACE, --cd WORKSPACE, --workspace WORKSPACE
                        project directory (default: the current directory)
  --trust-workspace     explicitly trust the project without a question
```

## openrua serve

```
usage: openrua serve [-h] [--sim SIM] [--bench BENCH]
                     [--task-suite TASK_SUITE] [--task-id TASK_ID]
                     [--init-state INIT_STATE] [--task TASK] [--name NAME]
                     [--agent AGENT] [-C WORKSPACE] [--trust-workspace]
                     [--ros-domain ROS_DOMAIN] [--internet {open,model-only}]
                     [--web-search | --no-web-search]
                     [--account-connectors | --no-account-connectors]
                     [--permissions MODE] [--model MODEL] [--port PORT]
                     [--resume-conversation] [--connect FOLDER]
                     [--distro {jazzy,humble}]
                     [robot]

Run a local shared session independently of its connected clients.

positional arguments:
  robot

options:
  -h, --help            show this help message and exit
  --sim SIM             simulator that embodies the robot (openrua
                        simulators); default: the benchmark's; a real robot's
                        file needs none
  --bench BENCH         benchmark whose world to load (openrua benchmarks);
                        default: the saved benchmark, else the simulator's
                        native scene
  --task-suite TASK_SUITE
                        scene suite (default: the profile's)
  --task-id TASK_ID     scene index (default: the profile's)
  --init-state INIT_STATE
                        episode seed / init state (simulated robots)
  --task TASK           task sentence to show the agent (real robots; a
                        simulated robot's comes from the scene)
  --name NAME           session handle (default: openrua)
  --agent AGENT         agent to open (default: the config's)
  -C WORKSPACE, --cd WORKSPACE, --workspace WORKSPACE
                        persistent robot project directory (default: the
                        current directory)
  --trust-workspace     explicitly trust the workspace without an interactive
                        question
  --ros-domain ROS_DOMAIN
                        ROS_DOMAIN_ID (default: the lowest one no running
                        robot uses, so concurrent robots never share a graph;
                        a real robot: its own domain, 0)
  --internet {open,model-only}
                        open, or only the agent's model service and login,
                        through a whitelist proxy (default: not limited)
  --web-search, --no-web-search
                        the agent CLI's own web search and fetch tools
                        (default: not limited)
  --account-connectors, --no-account-connectors
                        what your account brings beyond the CLI: its
                        connectors or apps, synced skills and plugins
                        (default: not limited)
  --permissions MODE    selected agent's native permission mode; the plugin
                        supplies its default; /permissions lists supported
                        choices
  --model MODEL         model (default: the configured agent's)
  --port PORT           local port (default: choose a free port)
  --resume-conversation
                        restart an ended project conversation without
                        replaying actions
  --connect FOLDER      connect a real robot instead (see openrua connect):
                        FOLDER is the robot's adaptation folder inside the
                        project workspace
  --distro {jazzy,humble}
                        with --connect: the connecting terminal's ROS 2
```

## openrua session

```
usage: openrua session [-h] [--name NAME] ACTION ...

positional arguments:
  ACTION

options:
  -h, --help   show this help message and exit
  --name NAME
```

| command | does |
|---|---|
| `web` | open the browser client and explicitly show its access token |
| `status` | snapshot, pending questions and message IDs |
| `robot` | maintain the robot while keeping this conversation open |
| `events` | read retained events after a cursor |
| `send` | enqueue a message without waiting for its result |
| `interrupt` | stop the running message; queued ones go next |
| `withdraw` | withdraw a queued message |
| `resume` | release the currently confirmed pause |
| `edit` | edit a queued message at its current revision |
| `release` | explicitly send one instruction held across a connection change |
| `resolve_unknown` | record checking an uncertain execution, without replaying it |
| `respond` | answer a pending agent question |
| `configure` | change a native permission mode, model or reasoning effort |
| `end` | stop owned resources and retain messages and workspace |

### openrua session web

```
usage: openrua session web [-h] [--no-open]

options:
  -h, --help  show this help message and exit
  --no-open   print connection instructions without opening a browser
```

### openrua session status

```
usage: openrua session status [-h]

options:
  -h, --help  show this help message and exit
```

### openrua session robot

```
usage: openrua session robot [-h] [--request-id REQUEST_ID]
                             {status,logs,start,stop,restart,build}

positional arguments:
  {status,logs,start,stop,restart,build}

options:
  -h, --help            show this help message and exit
  --request-id REQUEST_ID
                        reuse after a lost acknowledgement; never retries an
                        unknown execution
```

### openrua session events

```
usage: openrua session events [-h] [--after AFTER] [--limit LIMIT]

options:
  -h, --help     show this help message and exit
  --after AFTER
  --limit LIMIT
```

### openrua session send

```
usage: openrua session send [-h] [--client-id CLIENT_ID]
                            [--request-id REQUEST_ID]
                            text

positional arguments:
  text

options:
  -h, --help            show this help message and exit
  --client-id CLIENT_ID
                        reuse with request ID for safe retry
  --request-id REQUEST_ID
                        unique for this message; reuse only for retries
```

### openrua session interrupt

```
usage: openrua session interrupt [-h] message_id

positional arguments:
  message_id

options:
  -h, --help  show this help message and exit
```

### openrua session withdraw

```
usage: openrua session withdraw [-h] message_id

positional arguments:
  message_id

options:
  -h, --help  show this help message and exit
```

### openrua session resume

```
usage: openrua session resume [-h] pause_id

positional arguments:
  pause_id    copy the current pause_id from status

options:
  -h, --help  show this help message and exit
```

### openrua session edit

```
usage: openrua session edit [-h] --revision REVISION message_id text

positional arguments:
  message_id
  text

options:
  -h, --help           show this help message and exit
  --revision REVISION
```

### openrua session release

```
usage: openrua session release [-h] --revision REVISION message_id

positional arguments:
  message_id

options:
  -h, --help           show this help message and exit
  --revision REVISION
```

### openrua session resolve_unknown

```
usage: openrua session resolve_unknown [-h] message_id note

positional arguments:
  message_id
  note

options:
  -h, --help  show this help message and exit
```

### openrua session respond

```
usage: openrua session respond [-h] request_id answers

positional arguments:
  request_id
  answers     JSON mapping question IDs to lists of answers

options:
  -h, --help  show this help message and exit
```

### openrua session configure

```
usage: openrua session configure [-h] {permissions,model,effort} value

positional arguments:
  {permissions,model,effort}
  value                 a native permission mode, model ID or supported effort
                        level

options:
  -h, --help            show this help message and exit
```

### openrua session end

```
usage: openrua session end [-h]

options:
  -h, --help  show this help message and exit
```

## openrua agent

```
usage: openrua agent [-h] [--name NAME] [--agent AGENT] [--model MODEL]
                     [--robot ROBOT] [--sim SIM] [--bench BENCH]
                     [--ros-domain ROS_DOMAIN] [--task TASK]
                     [--workspace WORKSPACE] [--trust-workspace]
                     [[agent] [message] ...]

``openrua agent``: your coding agent's own terminal, on a robot. openrua agent
the configured agent openrua agent <name> a named one (openrua agents), or a
manifest's path openrua agent <name> "open the drawer" with an opening message
With a robot already up (``openrua up``, or an earlier ``openrua agent`` kept
running) the agent opens on it: the newest, or the one ``--name`` names.
Otherwise OpenRUA starts one first, the way ``openrua`` does (Docker and the
agent's sign-in set up with you, missing images built, the saved robot unless
``--robot``, ``--sim`` or ``--bench`` say otherwise), and when the agent exits
asks whether to turn it off. This is the agent's own interface: its Esc stops
the agent, not a motion it already sent.

positional arguments:
  [agent] [message]     the agent to open (openrua agents lists them; default:
                        the configured one), then an opening message

options:
  -h, --help            show this help message and exit
  --name NAME           the robot's handle (default: the newest robot up, else
                        a new one)
  --agent AGENT         agent, as the first word does
  --model MODEL         model (default: the one the robot recorded)
  --robot ROBOT         robot to start when none is up (default: the saved
                        one)
  --sim SIM             its simulator (default: the saved one)
  --bench BENCH         its benchmark (default: the saved one)
  --ros-domain ROS_DOMAIN
                        its ROS_DOMAIN_ID
  --task TASK           a real robot's task sentence, for its workspace
  --workspace WORKSPACE
                        persistent project directory (default: the current
                        directory)
  --trust-workspace     explicitly trust this directory without a question
```

## openrua down

```
usage: openrua down [-h] [--name NAME]

options:
  -h, --help   show this help message and exit
  --name NAME  the robot's handle (default: openrua)
```

## openrua bench

```
usage: openrua bench [-h] --config CONFIG [--robot ROBOT] [--sim SIM]
                     [--agent AGENT] [--model MODEL] --run-id RUN_ID
                     --task-suite TASK_SUITE [--task-ids TASK_IDS]
                     [--seeds SEEDS] [--operator {agent,none,script}]
                     [--task TASK] [--script SCRIPT] [--record [CAMERAS]]
                     [--record-every N] [--record-size WxH]
                     [--wall-clock-min WALL_CLOCK_MIN]
                     [--internet {open,model-only}]
                     [--web-search | --no-web-search]
                     [--account-connectors | --no-account-connectors]
                     [--ros-domain ROS_DOMAIN]
                     [--credentials-dir CREDENTIALS_DIR]
                     [--token-file TOKEN_FILE] [--runs-root RUNS_ROOT]
                     [--account-alias ACCOUNT_ALIAS]

``openrua bench``: a task set on a robot, one trial per (task, seed).

options:
  -h, --help            show this help message and exit
  --config CONFIG       benchmark config (benchmarks/<name>.yaml)
  --robot ROBOT         robot (type or your robot's file) name or path;
                        overrides the config's robot: line
  --sim SIM             simulator name or path; overrides the config's
                        simulator: line
  --agent AGENT         agent name (openrua agents); overrides the config's
                        agent.name. A different agent drops the config's agent
                        section: its model and options come from its own
                        manifest and your defaults
  --model MODEL         model name; overrides the config's (or the agent's
                        default) model
  --run-id RUN_ID
  --task-suite TASK_SUITE
  --task-ids TASK_IDS
  --seeds SEEDS
  --operator {agent,none,script}
  --task TASK           task sentence for a real robot (no bridge to ask); a
                        simulated robot's task comes from the benchmark and
                        this is ignored
  --script SCRIPT       command-sequence file for --operator script (canonical
                        source: a trial's commands.sh condensate); runs in the
                        sandbox on the native surface, open-loop best-effort
  --record [CAMERAS]    record the simulated robot's cameras every sim step
                        into the trial's frames/ (what `openrua demo`
                        renders); a comma-separated camera list, or none for
                        the profile's cameras.record. Rendering costs wall
                        clock on whole-room scenes: record a replay
                        (--operator script), not the experiment
  --record-every N      with --record, write one sim step in N (default 1:
                        every step). Each recorded step renders every camera
                        in software; a demo played at --speed N shows nothing
                        of the steps between, so N here cuts the replay's time
                        by about that factor at no cost to the video
  --record-size WxH     with --record, render the frames at this size instead
                        of the profile's camera resolution (the graph's
                        cameras keep theirs, so a replay is unaffected); a
                        sharper demo costs render time per step
  --wall-clock-min WALL_CLOCK_MIN
                        override of the config's
                        protocol.active_wall_clock_minutes (the config is the
                        default's single source)
  --internet {open,model-only}
                        open, or only the agent's model service and login,
                        through a whitelist proxy (default: the config's
                        protocol.limits)
  --web-search, --no-web-search
                        the agent CLI's own web search and fetch tools
                        (default: the config's protocol.limits)
  --account-connectors, --no-account-connectors
                        what your account brings beyond the CLI: its
                        connectors or apps, synced skills and plugins
                        (default: the config's protocol.limits)
  --ros-domain ROS_DOMAIN
                        ROS_DOMAIN_ID for this run's containers (default: each
                        trial takes the lowest domain no running container
                        uses, so concurrent runs never share a DDS graph; a
                        real robot: its own domain, 0)
  --credentials-dir CREDENTIALS_DIR
                        agent login-profile override (default: the config's
                        agent.credentials_dir, then native login discovery)
  --token-file TOKEN_FILE
                        file holding <token_env>=<token> for the sandbox CLI;
                        given, the sandbox authenticates with that token and
                        no credentials file is mounted
  --runs-root RUNS_ROOT
                        where run data lands (default: ./runs)
  --account-alias ACCOUNT_ALIAS
                        non-secret label of the credentials profile, recorded
                        in the trial result for per-account accounting
```

## openrua ps

```
usage: openrua ps [-h] [--all] [--runs-root RUNS_ROOT] [--json]

Every trial directory that carries an attempt's claim: the process, when it
started, whether it is still working (process alive, or its containers
running), and its container name stem. Check this before restarting anything
that launches trials; a live attempt is run over if a second writer starts on
its directory. A stale claim is a crashed attempt and is taken over by the
next run.

options:
  -h, --help            show this help message and exit
  --all, -a             also list stale claims (crashed attempts, and claims
                        archived under attempts/)
  --runs-root RUNS_ROOT
                        where runs live (default: runs, as for openrua bench)
  --json                machine-readable output
```

## openrua demo

```
usage: openrua demo [-h] [--out OUT] [--gif] [--cameras CAMERAS]
                    [--pace FACTOR] [--from-motion SECONDS] [--ops START:END]
                    [--size WxH] [--fps FPS] [--quality CRF] [--speed SPEED]
                    [--font-size FONT_SIZE] [--gif-width GIF_WIDTH]
                    [--gif-fps GIF_FPS] [--no-typing]
                    trial

Compose demo.mp4 from a trial that ran with --record: the commands typed on
the left, the robot's cameras on the right, one sim step per frame. Needs the
demo extra (pip install 'openrua[demo]').

positional arguments:
  trial                 a trial directory under runs/ that ran with --record

options:
  -h, --help            show this help message and exit
  --out OUT             output file (default: <trial>/demo.mp4)
  --gif                 also write demo.gif next to it (640 px wide, 8 fps;
                        keep it short with --ops and --speed)
  --cameras CAMERAS     main view and inset by name (default: the first two
                        recorded)
  --pace FACTOR         the terminal's clock: commands type FACTOR times
                        faster and the rests between events shrink by FACTOR
                        (default 1; the robot's clock is --speed). Most of a
                        clip is typing: a trial of a few hundred sim steps and
                        fifty commands is seconds of motion and minutes of
                        terminal
  --from-motion SECONDS
                        start the clip this many seconds (on the trial's
                        clock) before the robot first moves, skipping the
                        agent's reading of the robot and the scene; the
                        terminal notes how many commands came earlier
  --ops START:END       only the operations with index in [START, END) as
                        numbered in ops.jsonl (default all; a README clip
                        wants the last few)
  --size WxH            video size (default 1280x720)
  --fps FPS             frames per second (default 20)
  --quality CRF         x264 constant rate factor, 0 lossless to 51 worst
                        (default 18, visually lossless; 23 halves the file
                        again)
  --speed SPEED         sim steps per video frame (default 1: real-time robot
                        motion; a trial recorded one step in N plays N at
                        least)
  --font-size FONT_SIZE
                        terminal font size (default 13)
  --gif-width GIF_WIDTH
                        gif width in pixels (default 640)
  --gif-fps GIF_FPS     gif frames per second (default 8)
  --no-typing           show each command at once instead of typing it out
```

## openrua probe

```
usage: openrua probe [-h] [--host] [--static-peers ADDR[,ADDR]]
                     [--discovery-server HOST:PORT] [--ros-domain ROS_DOMAIN]
                     [--distro DISTRO] [--image IMAGE] [--name NAME]

Start a throwaway sandbox that can see the robot's graph (or reuse a live
robot's), read its topics, actions, services and /robot_description, and print
a profile draft with TODO on what only you know. Redirect it to a file and
edit.

options:
  -h, --help            show this help message and exit
  --host                the graph is on the host network
  --static-peers ADDR[,ADDR]
                        reach the graph through these unicast peers
  --discovery-server HOST:PORT
                        reach the graph through a Fast DDS discovery server
  --ros-domain ROS_DOMAIN
                        ROS_DOMAIN_ID of the graph
  --distro DISTRO       the robot's ROS 2 distro: jazzy | humble (picks the
                        sandbox image)
  --image IMAGE         sandbox image to probe from (default: openrua-
                        sandbox-<distro>)
  --name NAME           probe from a robot already up under this handle
                        instead
```

## openrua config

```
usage: openrua config [-h] [--user] [-C WORKSPACE] [--trust-workspace]
                      [--robot ROBOT] [--sim SIM] [--bench BENCH]
                      [--agent AGENT] [--model MODEL] [--version VERSION]
                      [--credentials-dir CREDENTIALS_DIR]
                      [--auth {native,api}] [--api-key-file API_KEY_FILE]
                      {show,set,schema}

Save this project in .openrua/settings.json. Bundled environment choices
create maintained configuration files in .openrua/configs/. --user selects
personal agent preferences and authentication in ~/.openrua/config.yaml.

positional arguments:
  {show,set,schema}     what to do

options:
  -h, --help            show this help message and exit
  --user                read or change personal agent preferences
  -C WORKSPACE, --cd WORKSPACE, --workspace WORKSPACE
                        project directory (default: the current directory)
  --trust-workspace     explicitly trust this project before saving
  --robot ROBOT         robot a command uses when it names none
  --sim SIM             simulator, likewise
  --bench BENCH         benchmark whose world a session loads by default (null
                        = the simulator's native scene)
  --agent AGENT         agent a command uses when it names none (openrua
                        agents)
  --model MODEL         project model; with --user, the personal model under
                        agents.<agent>
  --version VERSION     pin the agent's CLI version (agents.<agent>.version)
  --credentials-dir CREDENTIALS_DIR
                        the agent's login profile directory
                        (agents.<agent>.credentials_dir)
  --auth {native,api}   authentication source; native login is the default,
                        API use is explicit
  --api-key-file API_KEY_FILE
                        file containing a raw key, used only with --auth api
```

## openrua skills

```
usage: openrua skills [-h] [-C WORKSPACE] [--trust-workspace] COMMAND ...

positional arguments:
  COMMAND

options:
  -h, --help            show this help message and exit
  -C WORKSPACE, --cd WORKSPACE, --workspace WORKSPACE
                        project directory (default: current directory)
  --trust-workspace     explicitly trust the selected project before writing
```

| command | does |
|---|---|
| `list` | inspect saved file metadata; native availability may differ |
| `show` | read one saved SKILL.md |
| `add` | import or promote one skill without overwriting existing files |
| `policy` | read or change accumulation instructions; existing skills remain usable |
| `tasks` | inspect or cancel review tasks without starting a robot |
| `review` | review completed project records in isolation and save proposals |
| `reviews` | list this project's review reports or read one |
| `scheduler` | inspect, start or stop the shared background review scheduler |
| `schedule` | manage this project's optional periodic skill reviews |

### openrua skills list

```
usage: openrua skills list [-h] [--json] [--personal]

options:
  -h, --help  show this help message and exit
  --json
  --personal  only personal OpenRUA skills
```

### openrua skills show

```
usage: openrua skills show [-h] [--personal] name

positional arguments:
  name

options:
  -h, --help  show this help message and exit
  --personal
```

### openrua skills add

```
usage: openrua skills add [-h] [--personal] source

positional arguments:
  source      skill directory or SKILL.md

options:
  -h, --help  show this help message and exit
  --personal  save in the user directory for use across projects
```

### openrua skills policy

```
usage: openrua skills policy [-h] [{auto,suggest,manual}]

positional arguments:
  {auto,suggest,manual}

options:
  -h, --help            show this help message and exit
```

### openrua skills tasks

```
usage: openrua skills tasks [-h] [--cancel] [--json] [id]

positional arguments:
  id          task ID from the project menu or task list

options:
  -h, --help  show this help message and exit
  --cancel    request cooperative cancellation of this review
  --json
```

### openrua skills review

```
usage: openrua skills review [-h] [--agent AGENT] [--model MODEL]
                             [--effort EFFORT] [--image IMAGE] [--since SINCE]
                             [--records DIR] [--max-records MAX_RECORDS]
                             [--max-input-mb MAX_INPUT_MB]
                             [--max-file-mb MAX_FILE_MB]
                             [--max-files MAX_FILES] [--max-events MAX_EVENTS]
                             [--wall-clock-min WALL_CLOCK_MIN]
                             [--max-turns MAX_TURNS] [--force] [--json]

options:
  -h, --help            show this help message and exit
  --agent AGENT         agent name or manifest; default: project or user
                        setting
  --model MODEL         model; default: project or selected agent setting
  --effort EFFORT       native effort option; default: selected agent settings
  --image IMAGE         existing agent image; default: standard Jazzy sandbox
  --since SINCE         ISO date/time, UTC unless an offset is given
  --records DIR         additional exec record root; ownership is still
                        checked
  --max-records MAX_RECORDS
                        newest eligible completed records to inspect
  --max-input-mb MAX_INPUT_MB
                        total selected input size in MiB
  --max-file-mb MAX_FILE_MB
                        per-file or conversation export limit in MiB
  --max-files MAX_FILES
                        input and proposal file-count limit
  --max-events MAX_EVENTS
                        recent events per conversation; omitted events are
                        reported
  --wall-clock-min WALL_CLOCK_MIN
                        worker execution time cap in minutes
  --max-turns MAX_TURNS
                        requested native turn cap; adapters may not enforce it
  --force               review again even if identical inputs already have a
                        completed review
  --json
```

### openrua skills reviews

```
usage: openrua skills reviews [-h] [--json] [--input PATH] [id]

positional arguments:
  id            review ID from the list

options:
  -h, --help    show this help message and exit
  --json
  --input PATH  read a relative path from this review's private input snapshot
```

### openrua skills scheduler

```
usage: openrua skills scheduler [-h] [--json] [{status,start,stop}]

positional arguments:
  {status,start,stop}

options:
  -h, --help           show this help message and exit
  --json
```

### openrua skills schedule

```
usage: openrua skills schedule [-h] [--json] COMMAND ...

positional arguments:
  COMMAND

options:
  -h, --help  show this help message and exit
  --json
```

| command | does |
|---|---|
| `list` | list schedules in this project |
| `add` | save a plan; --enable explicitly starts background scheduling |
| `edit` | change a saved name, calendar or review settings |
| `show` | inspect a plan and its last task |
| `pause` | pause future reviews; the current review continues |
| `resume` | resume future reviews after inspecting the project and account |
| `remove` | remove the plan; keep reports and any current review |

## openrua login

```
usage: openrua login [-h] [--dir DIR] [agent]

``openrua login``: sign your coding agent in on this computer. The agent's own
sign-in (its manifest's ``credentials.login``) runs inside OpenRUA's sandbox
image, where its CLI is installed already, so nothing else is installed here.
It shows a link to open in any browser, and a code to paste back or to enter
on the page. The login is written to the directory every session reads it from
(the CLI's own in your home directory, unless your config names another), and
sessions use it from the next start on. ``openrua`` offers this by itself when
the agent you start with is not signed in.

positional arguments:
  agent       the agent to sign in (default: the configured one; openrua
              agents lists them)

options:
  -h, --help  show this help message and exit
  --dir DIR   the login directory (default: the one sessions read)
```

## openrua clean

```
usage: openrua clean [-h] [--name NAME] [--all]

Sessions retain their workspace, native agent profile and state after
shutdown. This command deletes those materials under ``<home>/sandboxes/``: by
default all stopped sessions, or just ``--name``. ``--all`` also stops running
containers before deletion. User-supplied workspace directories outside the
session directory, ``config.yaml`` and ``credentials/`` are never deleted.

options:
  -h, --help   show this help message and exit
  --name NAME  delete only this session (default: all stopped sessions)
  --all        also power running containers off and delete their session
               files
```

## openrua doctor

```
usage: openrua doctor [-h] [-C WORKSPACE] [--sim SIM] [--bench BENCH]
                      [--agent NAME[@VERSION]] [--json]
                      [robot]

positional arguments:
  robot                 also check this robot's images and simulator (with
                        --sim, or --bench, as for openrua exec); default: your
                        saved robot, simulator and benchmark, what openrua
                        starts

options:
  -h, --help            show this help message and exit
  -C WORKSPACE, --cd WORKSPACE, --workspace WORKSPACE
                        project directory (default: the current directory)
  --sim SIM             simulator that embodies the robot
  --bench BENCH         benchmark to check the install of
  --agent NAME[@VERSION]
                        agent(s) the images must carry, @VERSION as pinned at
                        build (default: the robot's config, else the
                        configured default)
  --json                machine-readable output
```

## Exit codes

| code | meaning |
|---|---|
| 0 | done |
| 1 | any other failure |
| 2 | bad arguments |
| 66 | a named robot, benchmark, agent or file does not exist |
| 69 | docker, an image, a container or the simulator is missing |
| 77 | login or credentials |
| 78 | a config file that does not fit the schema |
