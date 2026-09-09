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
usage: openrua [-h] [--version] [--home HOME] <verb> ...
```

| verb | does |
|---|---|
| `robots` | list the bundled robots |
| `simulators` | list the bundled simulators |
| `benchmarks` | list the bundled benchmarks |
| `agents` | list the bundled agents |
| `build` | build the robot, sandbox and proxy images |
| `install` | build a simulator install (checkouts and venv) |
| `run` | bring a robot up, open the agent on it, power off after |
| `up` | bring a robot up with a sandbox terminal on it |
| `agent` | open a coding agent on the robot's terminal |
| `down` | power a robot and its terminal off |
| `bench` | run a benchmark: one trial per task and seed |
| `ps` | list the attempts running under a runs root (--all: stale claims too) |
| `demo` | render a recorded trial as a video (terminal + cameras) |
| `probe` | draft a robot profile from a live ROS 2 graph |
| `config` | your defaults (robot, simulator, benchmark, agent) |
| `doctor` | check the install: docker, images, simulator, login |

Global options: `--home` (the user directory, default `$OPENRUA_HOME` or `~/.openrua`), `--version`.

## openrua robots

```
usage: openrua robots [-h] [--json]

Every robot shipped in the package. A file of your own is not listed; pass it
as a path where a name is expected.

options:
  -h, --help  show this help message and exit
  --json      machine-readable output
```

## openrua simulators

```
usage: openrua simulators [-h] [--json]

Every simulator shipped in the package. A file of your own is not listed; pass
it as a path where a name is expected.

options:
  -h, --help  show this help message and exit
  --json      machine-readable output
```

## openrua benchmarks

```
usage: openrua benchmarks [-h] [--json]

Every benchmark shipped in the package. A file of your own is not listed; pass
it as a path where a name is expected.

options:
  -h, --help  show this help message and exit
  --json      machine-readable output
```

## openrua agents

```
usage: openrua agents [-h] [--json]

Every agent shipped in the package. A file of your own is not listed; pass it
as a path where a name is expected.

options:
  -h, --help  show this help message and exit
  --json      machine-readable output
```

## openrua build

```
usage: openrua build [-h] [<unit>] ...

Build the three images (bare `openrua build`: all of them with their
defaults), or one of them with its options. The sandbox and proxy images take
their install line and whitelist from the manifests of the agents named with
--agent.

positional arguments:
  [<unit>]
    robot     the simulated robot image (Dockerfile.<distro>)
    sandbox   the agent terminal image
    proxy     the whitelist proxy image

options:
  -h, --help  show this help message and exit
```

### openrua build robot

```
usage: openrua build robot [-h] [--distro DISTRO] [--tag TAG]

options:
  -h, --help       show this help message and exit
  --distro DISTRO  ROS 2 distro: jazzy | humble
  --tag TAG        image tag (default: openrua-sim-<distro>)
```

### openrua build sandbox

```
usage: openrua build sandbox [-h] [--agent NAME[@VERSION]]
                             [--preinstall PREINSTALL] [--distro DISTRO]
                             [--robot-uid ROBOT_UID] [--tag TAG]

options:
  -h, --help            show this help message and exit
  --agent NAME[@VERSION]
                        agent(s) to install, repeatable; @VERSION pins the
                        CLI, otherwise the current release (default: the
                        configured default)
  --preinstall PREINSTALL
                        install line to bake instead of the agents' manifests
  --distro DISTRO       ROS 2 distro: jazzy | humble (the robot's; profiles
                        name it as ros_distro)
  --robot-uid ROBOT_UID
                        container uid (default: the current user)
  --tag TAG             image tag (default: openrua-sandbox-<distro>)
```

### openrua build proxy

```
usage: openrua build proxy [-h] [--agent AGENT] [--whitelist WHITELIST]
                           [--tag TAG] [--port PORT]

options:
  -h, --help            show this help message and exit
  --agent AGENT         agent(s) whose hosts to allow; repeatable (default:
                        the configured default)
  --whitelist WHITELIST
                        regexes, one per line, instead of the agents'
                        manifests
  --tag TAG
  --port PORT           listen port, baked in and labelled
```

## openrua install

```
usage: openrua install [-h] [--sim SIM] [--bench BENCH] [--print]

Build what a simulator file, and a benchmark's install: over it, declare:
repositories at pinned commits, a venv at the declared Python with the
requirements lock, editable checkouts, and the package itself. The script is
printed, saved under <home>/simulators/ and run; rerunning is cheap. Needs uv
and git.

options:
  -h, --help     show this help message and exit
  --sim SIM      simulator to install (openrua simulators); default: the
                 benchmark's, else the user config's
  --bench BENCH  benchmark whose install to build (openrua benchmarks);
                 default: the user config's default benchmark, if any
  --print        print the script and stop
```

## openrua run

```
usage: openrua run [-h] [--model MODEL] [--sim SIM] [--bench BENCH]
                   [--task-suite TASK_SUITE] [--task-id TASK_ID]
                   [--init-state INIT_STATE] [--task TASK] [--name NAME]
                   [--agent AGENT] [--workspace WORKSPACE]
                   [--ros-domain ROS_DOMAIN]
                   [robot] [prompt]

Bring a robot up, open the coding agent on its terminal with PROMPT as the
opening message, and power the robot off when the agent exits. Same steps as
up, agent, down.

positional arguments:
  robot                 robot: a type or your robot's file (openrua robots),
                        by name or path; default: --bench's robot, else the
                        user config's default
  prompt                opening message for the agent

options:
  -h, --help            show this help message and exit
  --model MODEL         model (default: the config's)
  --sim SIM             simulator that embodies the robot (openrua
                        simulators); default: the benchmark's; a real robot's
                        file needs none
  --bench BENCH         benchmark whose world to load (openrua benchmarks);
                        default: the simulator's native scene
  --task-suite TASK_SUITE
                        scene suite (default: the profile's)
  --task-id TASK_ID     scene index (default: the profile's)
  --init-state INIT_STATE
                        episode seed / init state (simulated robots)
  --task TASK           task sentence to show the agent (real robots; a
                        simulated robot's comes from the scene)
  --name NAME           handle for this robot, for agent/down (default:
                        openrua)
  --agent AGENT         agent to open (default: the config's)
  --workspace WORKSPACE
                        working directory (default: <home>/workspaces/<name>)
  --ros-domain ROS_DOMAIN
                        ROS_DOMAIN_ID; concurrent robots need distinct ones
```

## openrua up

```
usage: openrua up [-h] [--sim SIM] [--bench BENCH] [--task-suite TASK_SUITE]
                  [--task-id TASK_ID] [--init-state INIT_STATE] [--task TASK]
                  [--name NAME] [--agent AGENT] [--workspace WORKSPACE]
                  [--ros-domain ROS_DOMAIN]
                  [robot]

Bring a robot up (simulated: boot its container; real: join its graph) with a
sandbox terminal on it, then stay in the foreground; Ctrl-C powers it off.
Open a second terminal for `openrua agent`, or use `openrua run` to do all of
it in one.

positional arguments:
  robot                 robot: a type or your robot's file (openrua robots),
                        by name or path; default: --bench's robot, else the
                        user config's default

options:
  -h, --help            show this help message and exit
  --sim SIM             simulator that embodies the robot (openrua
                        simulators); default: the benchmark's; a real robot's
                        file needs none
  --bench BENCH         benchmark whose world to load (openrua benchmarks);
                        default: the simulator's native scene
  --task-suite TASK_SUITE
                        scene suite (default: the profile's)
  --task-id TASK_ID     scene index (default: the profile's)
  --init-state INIT_STATE
                        episode seed / init state (simulated robots)
  --task TASK           task sentence to show the agent (real robots; a
                        simulated robot's comes from the scene)
  --name NAME           handle for this robot, for agent/down (default:
                        openrua)
  --agent AGENT         agent to open (default: the config's)
  --workspace WORKSPACE
                        working directory (default: <home>/workspaces/<name>)
  --ros-domain ROS_DOMAIN
                        ROS_DOMAIN_ID; concurrent robots need distinct ones
```

## openrua agent

```
usage: openrua agent [-h] [--name NAME] [--agent AGENT] [--model MODEL]
                     [prompt]

Open the configured coding agent interactively on a live robot's terminal
(docker exec into its sandbox).

positional arguments:
  prompt         opening message

options:
  -h, --help     show this help message and exit
  --name NAME    the robot's handle (default: openrua)
  --agent AGENT  agent (default: the one `up` opened)
  --model MODEL  model (default: the one `up` recorded)
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
usage: openrua bench [-h] --config CONFIG [--robot ROBOT] [--sim SIM] --run-id
                     RUN_ID --task-suite TASK_SUITE [--task-ids TASK_IDS]
                     [--seeds SEEDS] [--operator {agent,none,script}]
                     [--task TASK] [--script SCRIPT] [--record [CAMERAS]]
                     [--wall-clock-min WALL_CLOCK_MIN]
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
  --wall-clock-min WALL_CLOCK_MIN
                        override of the config's
                        protocol.active_wall_clock_minutes (the config is the
                        default's single source)
  --ros-domain ROS_DOMAIN
                        ROS_DOMAIN_ID for this run's containers; concurrent
                        runs must use distinct domains (one DDS network would
                        cross-talk)
  --credentials-dir CREDENTIALS_DIR
                        agent login-profile override (default: the config's
                        agent.credentials_dir, then
                        ~/.openrua/credentials/<agent>)
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
                    [--ops START:END] [--size WxH] [--fps FPS] [--quality CRF]
                    [--speed SPEED] [--font-size FONT_SIZE]
                    [--gif-width GIF_WIDTH] [--gif-fps GIF_FPS] [--no-typing]
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
  --ops START:END       only the operations with index in [START, END) as
                        numbered in ops.jsonl (default all; a README clip
                        wants the last few)
  --size WxH            video size (default 1280x720)
  --fps FPS             frames per second (default 20)
  --quality CRF         x264 constant rate factor, 0 lossless to 51 worst
                        (default 18, visually lossless; 23 halves the file
                        again)
  --speed SPEED         sim steps per video frame (default 1: real-time robot
                        motion)
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
usage: openrua config [-h] [--robot ROBOT] [--sim SIM] [--bench BENCH]
                      [--agent AGENT] [--model MODEL]
                      [--credentials-dir CREDENTIALS_DIR]
                      {show,set,schema}

Your defaults file, ~/.openrua/config.yaml: `config set` writes the flags it
is given into it (the same flags run takes), `config show` prints it, `config
schema` prints the resolved config's JSON schema, every key with its meaning.

positional arguments:
  {show,set,schema}     what to do

options:
  -h, --help            show this help message and exit
  --robot ROBOT         robot a command uses when it names none
  --sim SIM             simulator, likewise
  --bench BENCH         benchmark whose world run / up load by default (null =
                        the simulator's native scene)
  --agent AGENT         agent (openrua agents)
  --model MODEL         model id for the agent
  --credentials-dir CREDENTIALS_DIR
                        the agent's login profile directory
```

## openrua doctor

```
usage: openrua doctor [-h] [--sim SIM] [--bench BENCH]
                      [--agent NAME[@VERSION]] [--json]
                      [robot]

positional arguments:
  robot                 also check this robot's images and simulator (with
                        --sim, or --bench, as for openrua run)

options:
  -h, --help            show this help message and exit
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
