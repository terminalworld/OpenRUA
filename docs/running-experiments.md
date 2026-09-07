---
summary: Running a task set with robocli run, what lands under runs/, and what every record field means
read_when:
  - You want to reproduce or extend the paper's numbers
  - You are reading a result.json or provenance.json
  - You want to replay a trial or run it without an agent
---

# Running experiments

## One command, one run

```bash
robocli run --config libero_pro --run-id demo \
            --task-suite libero_goal_task --task-ids 0-9 --seeds 0-4 \
            --operator agent
```

A run is one benchmark config, one task suite, and a set of task ids
and seeds; each (task, seed) pair is one trial, run in sequence. `--config`
names a bundled benchmark (`robocli benchmarks`), a file under
`~/.robocli/benchmarks/`, or a path. `--robot` overrides the config's
robot. Everything else about the protocol (wall clock, turn budget,
cameras, control) is in the config, and the trial writes the resolved
copy it ran with.

Three operators:

| `--operator` | what runs in the sandbox | use |
|---|---|---|
| `agent` | the coding agent from the config | the experiment |
| `none` | nothing; bring-up, preflight and teardown only | checking a machine, no quota spent |
| `script` | a command sequence from `--script`, replayed open-loop | replaying a trial's `commands.sh` |

A real robot has no scoring side: pass `--task "<sentence>"`, the
verdict is recorded as not applicable, and nothing is reset.

## Where it lands

```
<--runs-root, default ./runs>/<benchmark>/<--run-id>/
  config.json                 the run's arguments
  SUMMARY.md                  rewritten after every trial
  trials/<suite>-<task_id>/seed<N>/
    config.yaml               the resolved config this trial ran with
    provenance.json           code, simulator, images, prompt, engine
    result.json               verdict, preflight, termination, accounting
    transcript.jsonl          the agent's full session (operator agent)
    commands.sh               the agent's shell, write and edit operations, in order
    bridge.log, moveit.log    the robot's own logs
    workspace/                what the agent left in /workspace
    attempts/                 earlier attempts of the same trial, moved aside
```

A trial directory identifies a (suite, task, seed); a rerun of the
same trial moves the previous attempt under `attempts/` and never
overwrites it. Two runs never share a directory: name a new
`--run-id` when the task set changes.

## result.json

| field | meaning |
|---|---|
| `task_suite`, `task_id`, `init_state_id` | the trial key; `init_state_id` is the seed |
| `task_language` | the task sentence the agent was given, from the benchmark's own files |
| `init_state` | what the loader records about the episode's world (RoboCasa: the kitchen) |
| `success` | the benchmark's own predicate, latched per step (any-step success); `null` on a real robot |
| `success_end_state` | the same predicate at the end of the episode |
| `success_at` | step and time of the first success |
| `steps_total` | simulator steps since reset |
| `termination` | `self_finished`, `max_turns`, `wall_clock_cap`, `quota_limit`, `sandbox_lost`, `launcher_failed`, `operator_done`, `anomaly` |
| `anomaly`, `anomaly_traceback` | a harness failure (preflight red, sim stall, bring-up error); the trial does not count |
| `preflight` | every check and its verdict, from the sandbox's own vantage, before the agent started |
| `agent_version` | the CLI version the sandbox reported |
| `wall_seconds` | the whole harness span (boot, preflight, operator, teardown) |
| `active_wall_cap_min` | the active wall-clock budget the operator ran under |
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

`robocli_version` and `robocli_commit` (with `git_dirty`),
`simulator_commit` (with `simulator_dirty`), the three image digests,
`config_file` and `config_sha256`, the prompt's hash and the workspace
template's hash, `agent_cli` (name and version pin), `container_engine`
and `sandbox_run_args`, `gpu_render`, `host`, `ros_domain`, and the run's
task ids and seeds. Two trials with equal hashes and digests ran the
same experiment.

## Budgets and quota walls

`protocol.active_wall_clock_minutes` and `protocol.max_turns` are the
two budgets, both spent across segments. A quota rejection in the
transcript ends the trial unless `protocol.resume_on_quota_wall` is
on, in which case the runner waits for the window (bounded by
`max_quota_wait_minutes` and `max_suspensions`) and resumes the same
session. Whether a trial counts is decided afterwards from the
artifacts, never by the runner.

## Replaying a trial

```bash
robocli run --config libero_pro --run-id replay --task-suite libero_goal_task \
            --task-ids 3 --seeds 0 --operator script \
            --script runs/libero_pro/demo/trials/libero_goal_task-3/seed0/commands.sh
```

The agent ran closed-loop, so an identical outcome under a paused
clock and a seeded reset is likely, never guaranteed.
