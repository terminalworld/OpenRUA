---
summary: Every robocli verb, its arguments, and the exit codes
read_when:
  - You want the exact flags of a verb without running --help
  - A script needs to act on robocli's exit code
---

# Command line

Generated from the parsers by `scripts/render_docs.py`; edit the
`add_parser` of a verb, not this page.

```
usage: robocli [-h] [--version] [--home HOME] <verb> ...

The command line: ``robocli <verb> ...``.

positional arguments:
  <verb>
    robots      list the robots: bundled, then ~/.robocli/robots/
    benchmarks  list the benchmarks: bundled, then ~/.robocli/benchmarks/
    agents      list the agents: bundled, then ~/.robocli/agents/
    build       build the robot / sandbox / proxy image
    up          bring a robot up with a sandbox terminal on it
    agent       open a coding agent on the robot's terminal
    down        power a robot and its terminal off
    run         run a task set: one trial per task and seed
    ps          list the attempts running under a runs root (--all: stale
                claims too)
    demo        render a recorded trial as a video (terminal + cameras)
    probe       draft a robot profile from a live ROS 2 graph
    config      the configuration schema
    doctor      check the install: docker, images, simulator, login

options:
  -h, --help    show this help message and exit
  --version     show program's version number and exit
  --home HOME   the user directory: your robots/, benchmarks/, agents/,
                credentials/, simulators/ (default: $ROBOCLI_HOME or
                ~/.robocli)
```

## robocli robots

```
usage: robocli robots [-h] [--json]

Every robot RoboCLI can find: the bundled ones, then yours under
<home>/robots/. A user file that carries a bundled name is reported and not
used.

options:
  -h, --help  show this help message and exit
  --json      machine-readable output
```

## robocli benchmarks

```
usage: robocli benchmarks [-h] [--json]

Every benchmark RoboCLI can find: the bundled ones, then yours under
<home>/benchmarks/. A user file that carries a bundled name is reported and
not used.

options:
  -h, --help  show this help message and exit
  --json      machine-readable output
```

## robocli agents

```
usage: robocli agents [-h] [--json]

Every agent RoboCLI can find: the bundled ones, then yours under
<home>/agents/. A user file that carries a bundled name is reported and not
used.

options:
  -h, --help  show this help message and exit
  --json      machine-readable output
```

## robocli build

```
usage: robocli build [-h] <unit> ...

Build one of the three images. The sandbox and proxy images take their install
line and whitelist from the manifests of the agents named with --agent.

positional arguments:
  <unit>
    robot     the simulated robot image (Dockerfile.<distro>)
    sandbox   the agent terminal image
    proxy     the whitelist proxy image

options:
  -h, --help  show this help message and exit
```

### robocli build robot

```
usage: robocli build robot [-h] [--distro DISTRO] [--tag TAG]

options:
  -h, --help       show this help message and exit
  --distro DISTRO  ROS 2 distro: jazzy | humble
  --tag TAG        image tag (default: robocli-sim-<distro>)
```

### robocli build sandbox

```
usage: robocli build sandbox [-h] [--agent NAME[@VERSION]]
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
  --tag TAG             image tag (default: robocli-sandbox-<distro>)
```

### robocli build proxy

```
usage: robocli build proxy [-h] [--agent AGENT] [--whitelist WHITELIST]
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

## robocli up

```
usage: robocli up [-h] [--bench BENCH] [--task-suite TASK_SUITE]
                  [--task-id TASK_ID] [--init-state INIT_STATE] [--task TASK]
                  [--name NAME] [--agent AGENT] [--workspace WORKSPACE]
                  [--ros-domain ROS_DOMAIN]
                  [robot]

Bring a robot up (simulated: boot its container; real: join its graph) with a
sandbox terminal on it, then stay in the foreground; Ctrl-C powers it off.
Open a second terminal for `robocli agent`.

positional arguments:
  robot                 robot profile: a name (robocli robots) or a path;
                        default: --bench's robot, else the user config's
                        default

options:
  -h, --help            show this help message and exit
  --bench BENCH         benchmark to take the scene from (default: the
                        profile's world:)
  --task-suite TASK_SUITE
                        scene suite (default: the profile's)
  --task-id TASK_ID     scene index (default: the profile's)
  --init-state INIT_STATE
                        episode seed / init state (simulated robots)
  --task TASK           task sentence to show `robocli agent` (real robots; a
                        simulated robot's comes from the scene)
  --name NAME           handle for this robot, for agent/down (default:
                        robocli)
  --agent AGENT         agent to open (default: the config's)
  --workspace WORKSPACE
                        working directory (default: <home>/workspaces/<name>)
  --ros-domain ROS_DOMAIN
                        ROS_DOMAIN_ID; concurrent robots need distinct ones
```

## robocli agent

```
usage: robocli agent [-h] [--name NAME] [--agent AGENT] [--model MODEL]
                     [prompt]

Open the configured coding agent interactively on a live robot's terminal
(docker exec into its sandbox).

positional arguments:
  prompt         opening message

options:
  -h, --help     show this help message and exit
  --name NAME    the robot's handle (default: robocli)
  --agent AGENT  agent (default: the one `up` opened)
  --model MODEL  model (default: the one `up` recorded)
```

## robocli down

```
usage: robocli down [-h] [--name NAME]

options:
  -h, --help   show this help message and exit
  --name NAME  the robot's handle (default: robocli)
```

## robocli run

```
usage: robocli run [-h] --config CONFIG [--robot ROBOT] --run-id RUN_ID
                   --task-suite TASK_SUITE [--task-ids TASK_IDS]
                   [--seeds SEEDS] [--operator {agent,none,script}]
                   [--task TASK] [--script SCRIPT] [--record [CAMERAS]]
                   [--wall-clock-min WALL_CLOCK_MIN] [--ros-domain ROS_DOMAIN]
                   [--credentials-dir CREDENTIALS_DIR]
                   [--token-file TOKEN_FILE] [--runs-root RUNS_ROOT]
                   [--account-alias ACCOUNT_ALIAS]

``robocli run``: a task set on a robot, one trial per (task, seed).

options:
  -h, --help            show this help message and exit
  --config CONFIG       benchmark config (benchmarks/<name>.yaml)
  --robot ROBOT         robot profile name or path; overrides the config's
                        robot: line
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
                        into the trial's frames/ (what `robocli demo`
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
                        ~/.robocli/credentials/<agent>)
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

## robocli ps

```
usage: robocli ps [-h] [--all] [--runs-root RUNS_ROOT] [--json]

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
                        where runs live (default: runs, as for robocli run)
  --json                machine-readable output
```

## robocli demo

```
usage: robocli demo [-h] [--out OUT] [--gif] [--cameras CAMERAS]
                    [--ops START:END] [--size WxH] [--fps FPS] [--speed SPEED]
                    [--font-size FONT_SIZE]
                    trial

Compose demo.mp4 from a trial that ran with --record: the commands typed on
the left, the robot's cameras on the right, one sim step per frame. Needs the
demo extra (pip install 'robocli-harness[demo]').

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
  --speed SPEED         sim steps per video frame (default 1: real-time robot
                        motion)
  --font-size FONT_SIZE
                        terminal font size (default 13)
```

## robocli probe

```
usage: robocli probe [-h] [--host] [--static-peers ADDR[,ADDR]]
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
  --image IMAGE         sandbox image to probe from (default: robocli-
                        sandbox-<distro>)
  --name NAME           probe from a robot already up under this handle
                        instead
```

## robocli config

```
usage: robocli config [-h] {schema}

Configuration: `robocli config schema` prints the resolved config's JSON
schema, every key with its meaning.

positional arguments:
  {schema}    what to show

options:
  -h, --help  show this help message and exit
```

## robocli doctor

```
usage: robocli doctor [-h] [--agent NAME[@VERSION]] [--json] [robot]

positional arguments:
  robot                 also check this robot's images and simulator

options:
  -h, --help            show this help message and exit
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
