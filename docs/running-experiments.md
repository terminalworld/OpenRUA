---
summary: Recording one task with openrua exec or a task set with openrua bench, what lands under runs/, and what every record field means
read_when:
  - You want the agent to do one task on its own, with the run recorded
  - You want to reproduce or extend reported results
  - You are reading a result.json or provenance.json
  - You want to replay a trial, run it without an agent, or make a demo video of it
---

# Running experiments

## One task, one run

```bash
openrua exec panda --bench libero_pro --task-suite libero_goal_task --task-id 3 --seed 0
openrua exec panda "lift the cube" --sim robosuite
openrua exec ./my-robot.yaml "put the cup on the plate"
```

It needs the images prepared, as `openrua doctor` checks (for the
robosuite lines: `openrua build --sim robosuite`, `openrua build sandbox
--distro humble`, and `openrua build proxy`, since the run limits the agent
to its model service).

`openrua exec` gives the agent one task and lets it work on its own, as
`codex exec` or `claude -p` do: the robot comes up, the agent works until
it is done or a budget ends, the robot powers off, and the run is
recorded as one trial. A benchmark scene states its own task and judges
success. A simulator's own scene judges success too, and takes the
sentence you give when it states no task (robosuite's `Lift` states
none). A real robot's task is the sentence you give; nothing resets it,
and at the end `exec` asks you whether the robot did it: your answer is
the result (`verdict: person`), and Enter leaves it open
(`verdict: not_applicable`). The agent's last words are kept beside it.

While the agent works, `exec` shows what it says and each command or tool
it runs, with whether it worked and how long it took; the outputs stay in
the transcript. Ctrl-C stops the agent, cancels a real robot's motions,
judges a simulated scene as it stands, powers the robot off and records
the run as `stopped`.

The run lands under `runs/<label>/<run id>/`: the label is the benchmark,
else `<robot>-<simulator>`, else the robot file's name, and the run id is
`--run-id` or `exec-<UTC time>`. `--agent`, `--model`, `--wall-clock-min`,
`--ros-domain`, `--runs-root`, `--record` and the limit flags work as they
do for `bench`; the rest of the protocol comes from the benchmark's config,
else its defaults.

## A benchmark's task set

```bash
openrua bench --config libero_pro --run-id demo \
            --task-suite libero_goal_task --task-ids 0,1,2,3,4,5,6,7,8,9 --seeds 0,1,2,3,4 \
            --operator agent
```

A run is one benchmark config, one task suite, and a set of task ids
and seeds; each (task, seed) pair is one trial, run in sequence.
`openrua benchmarks <name>` shows what there is to choose from: the
suites, each suite's task ids with their sentences (read from the
benchmark's own files, in its simulator image), and what a seed means
there (an index into fixed initial-state files, the seed of the
benchmark's own reset, or, where the benchmark's reset is unseeded,
only the episode's number). `--config`
names a bundled benchmark (`openrua benchmarks`) or a path to a
benchmark file of your own. `--robot` overrides the config's
robot. Everything else about the protocol (wall clock, turn budget,
what the agent may reach, cameras, control) is in the config, and the
trial writes the resolved copy it ran with.

Three operators:

| `--operator` | what runs in the sandbox | use |
|---|---|---|
| `agent` | the coding agent from the config | the experiment |
| `none` | nothing; bring-up, preflight and teardown only | checking a machine, no quota spent |
| `script` | a command file from `--script`, one operation at a time in one shell, open-loop | replaying a trial's `commands.sh` |

A task on a real robot is recorded with `openrua exec` (above).

## A first trial and its video

One CaP-Bench Lift trial with its cameras recorded, rendered as a video
with the commands the agent typed beside the robot's views. Prepare once
(Docker running, the agent signed in):

```sh
pip install -U 'openrua[demo]'
openrua build --bench capbench
openrua build sandbox --distro humble --agent claude-code
openrua build proxy  # a trial limits the agent to its model service through it
openrua doctor panda --sim robosuite --bench capbench --agent claude-code
```

Then run and render:

```sh
openrua bench --config capbench --run-id lift-demo \
  --task-suite capbench_lift --task-ids 0 --seeds 0 \
  --operator agent --record --record-every 4 --wall-clock-min 10
openrua demo runs/capbench/lift-demo/trials/capbench_lift-0/seed0 --speed 4
```

The video is `demo.mp4` in that trial directory; `result.json` has the
benchmark's verdict and `provenance.json` the resolved configuration and
software versions. The bundled CaP-Bench configuration selects Claude
Opus 5 at high reasoning effort; `--agent` and `--model` select another.
The same `bench` command with `--operator none` and no recording checks
startup and the interfaces without a model call. One trial shows the
method; the [paper](https://arxiv.org/abs/2610.02459)'s aggregate results
come from its full task and seed sets, model settings and budgets.
Recording adds rendering time to the run.

## Where it lands

```
<--runs-root, default ./runs>/<benchmark, or exec's label>/<--run-id>/
  config.json                 the run's arguments
  SUMMARY.md                  rewritten after every trial
  trials/<suite>-<task_id>/seed<N>/
    config.yaml               the resolved config this trial ran with
    provenance.json           code, simulator, images, prompt, engine
    result.json               verdict, preflight, termination, accounting
    transcript.jsonl          the agent's full session (operator agent)
    commands.sh               what ran: the agent's shell, write and edit operations, or the replayed file
    ops.jsonl                 the same operations with output heads and wall times
    frames/                   camera frames: a simulated robot's sim steps (--record), a real robot's cameras two a second
    bridge.log, moveit.log    the robot's own logs
    workspace/                what the agent left in /workspace
    attempts/                 earlier attempts of the same trial, moved aside
```

A trial directory identifies a (suite, task, seed); a rerun of the
same trial moves the previous attempt under `attempts/` and never
overwrites it. Two runs never share a directory: name a new
`--run-id` when the task set changes.

Runs share a machine without arrangement: each bring-up takes the
lowest ROS domain no running container on the internal network uses,
so two `openrua bench` (or a `bench` beside an `openrua exec` or `openrua up`) never
share a DDS graph; the domain is in `result.json` and `provenance.json`.
`--ros-domain` pins one by hand, which a real robot needs (its graph
has a domain already).

While an attempt runs, its trial directory carries a claim (`.running`:
process, host, container stem, start time); a second `openrua bench` on
the same trial refuses to start while the claim is live, and takes over
a stale one. `openrua ps` lists every claim under the runs root with
its state; check it before restarting anything that launches trials.

## result.json

| field | meaning |
|---|---|
| `task_suite`, `task_id`, `init_state_id` | the trial key; `init_state_id` is the seed |
| `task_language` | the task sentence the agent was given, from the benchmark's own files |
| `init_state` | what the loader records about the episode's world (RoboCasa: the kitchen) |
| `success` | the benchmark's own predicate, latched per step (any-step success); on a real robot, the person's answer when `exec` asked (`verdict: person`), else `null` (`verdict: not_applicable`) |
| `success_end_state` | the same predicate at the end of the episode |
| `success_at` | step and time of the first success |
| `steps_total` | simulator steps since reset |
| `end_state` | the world when the operator was done: `objects` (every object's position and quaternion, by the simulator's own names) and `hand` (the hand's position and the grasp point); what a failed trial is measured against later |
| `termination` | `self_finished`, `max_turns`, `wall_clock_cap`, `quota_limit`, `sandbox_lost`, `launcher_failed`, `operator_done`, `anomaly`, or `stopped`: a person stopped the run (Ctrl-C, or the terminal closed); the agent was ended, a real robot's motions cancelled, a simulated robot judged as it stood, and the command exits 130 |
| `anomaly`, `anomaly_traceback` | a harness failure (preflight red, sim stall, bring-up error); the trial does not count |
| `preflight` | every check and its verdict, from the sandbox's own vantage, before the agent started |
| `agent_version` | the CLI version the sandbox reported |
| `limits` | what the agent could reach: `internet`, `web_search`, `account_connectors` |
| `wall_seconds` | the whole harness span (boot, preflight, operator, teardown) |
| `active_wall_cap_min` | the active wall-clock budget the operator ran under |
| `agent_final_message` | what the agent said last: its own account of how the task went |
| `operator_meta` | the operator's own record, below |
| `containers`, `ros_domain`, `account_alias` | the attempt's container names, DDS domain, and (optional) which login it used |

`operator_meta` for `--operator agent`: `agent`, `model`, `options`,
`session_id`, `num_turns`, `usage`, `cost_usd`, `duration_ms`,
`termination`, `launcher_returncode`, `segments` and `segment_detail`
(one launcher run per segment; a new segment only after a quota wall
was waited out), `active_seconds`, `suspended_seconds`,
`resume_on_quota_wall`, `resumes`, `quota_resolved`, `quota_gave_up_on`,
`lost_containers`.

## provenance.json

`openrua_version` and `openrua_commit` (with `git_dirty`), `sim_image`
and the three image digests (the simulator image's pins the simulator,
its patches and its assets),
`config_file` and `config_sha256`, the prompt's hash and the workspace
template's hash, `agent_cli` (name and version pin), `container_engine`
and `sandbox_run_args`, `gpu_render`, `record` (the cameras recorded,
or null), `host`, `ros_domain`, and the run's task ids and seeds. Two trials with equal hashes and digests ran the
same experiment.

## Budgets and quota walls

`protocol.active_wall_clock_minutes` and `protocol.max_turns` are the
two budgets, both spent across segments. A quota rejection in the
transcript ends the trial unless `protocol.resume_on_quota_wall` is
on, in which case the runner waits for the window (bounded by
`max_quota_wait_minutes` and `max_suspensions`) and resumes the same
session. Whether a trial counts is decided afterwards from the
artifacts, never by the runner.

## What the agent can reach

`protocol.limits` says it, by default `internet: model-only`,
`web_search: false`, `account_connectors: false`: the sandbox reaches
only the agent's model service and login, through a whitelist proxy, and
the CLI's own web tools and the account's connectors are off. A
benchmark config may change them; `--internet`, `--[no-]web-search` and
`--[no-]account-connectors` override them for one run and land in the
config the run records. `openrua exec` applies the same limits. Sessions you
start (`openrua`, `up`, `serve`) set none of them by default
([install.md](install.md#what-the-agent-can-reach)).

## Replaying a trial

```bash
openrua bench --config libero_pro --run-id replay --task-suite libero_goal_task \
            --task-ids 3 --seeds 0 --operator script \
            --script runs/libero_pro/demo/trials/libero_goal_task-3/seed0/commands.sh
```

`commands.sh` opens every operation with a `# openrua op N` line
(followed by `ran 12.3s`, how long it took the agent, when the
transcript says), and the script operator runs them one at a time in a
single sandbox shell, the way the agent did, timing each one into
`ops.jsonl`. Each operation is bounded: three times its recorded
duration plus half a minute, never under a minute, never past the wall
clock; an operation with no recorded duration has the wall clock alone.
The agent's own tool bounded its commands the same way, so a command
that never returns on its own (a `tf2_echo`, a topic echo without
`--once`) ends where it ended for the agent instead of holding the
replay until the wall clock. A killed operation hands nothing back:
the shell goes on where the previous operation left it. The agent ran
closed-loop, so an identical outcome under a paused clock and a seeded
reset is likely, never guaranteed. A hand-written command file works
the same way: without markers it is one operation.

## Making a demo video

A demo is a replay with the cameras recorded, then rendered:

```bash
openrua bench --config libero_pro --run-id demo --task-suite libero_goal_task \
            --task-ids 3 --seeds 0 --operator script \
            --script runs/libero_pro/main/trials/libero_goal_task-3/seed0/commands.sh \
            --record --record-every 4
openrua demo runs/libero_pro/demo/trials/libero_goal_task-3/seed0 --gif --speed 4
```

`--record` makes the robot write its sim steps' frames for the
profile's `cameras.record` (or the names you give it) under the
trial's `frames/`, one step in `--record-every` (every step by
default; each recorded step is a software render of every camera, so
recording one step in four makes the replay about four times faster,
and a demo played at `--speed 4` shows nothing of the steps between); `openrua demo` composes them with `ops.jsonl` into
`demo.mp4`, the commands typed on the left and the cameras on the
right, both on the trial's own clock (a command, the motion it caused,
its output), one sim step per video frame, and `--gif` adds a smaller
`demo.gif` for a README. `--from-motion 10` starts the clip ten seconds
(on the trial's clock) before the robot first moves: under a paused
clock the simulator steps only when the robot is driven, so the agent's
reading of the robot and the scene, which comes first and moves
nothing, is skipped and the terminal notes how many commands it took.
`--ops START:END` renders a slice of the
operations and `--speed` plays several sim steps per frame (never
fewer than the trial recorded), which is how a
clip gets short enough for a README. Rendering needs the `demo` extra: `pip install
'openrua[demo]'`.

`--record` works on a live agent run too, but rendering every step
costs wall clock on whole-room scenes, which the agent's budget would
pay for; record the replay, not the experiment.

A real robot's run cannot be replayed, so `openrua exec` records its
cameras unless `--no-record` says not to (`--record wrist,scene` picks
cameras; `bench` records a real robot only with `--record`). The frames
come from the cameras' ROS topics, two a second, into the same
`frames/` layout, each numbered with its wall time, and `openrua demo`
composes them with the agent's commands in the same way. The grabber
reads a camera's `/compressed` topic when there is one and runs at a low
CPU priority, so the robot's control loop on the same computer comes
first.
