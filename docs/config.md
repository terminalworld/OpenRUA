---
summary: Every key of the robot profiles, benchmark configs, agent manifests and the defaults file
read_when:
  - You are writing a robot profile, a benchmark config or an agent manifest
  - A config error names a key and you want its meaning and allowed values
---

# Configuration

Generated from the schema by `scripts/render_docs.py`; edit the
`Field(description=...)` in `robocli/config/schema.py`, not this page.
Unknown keys are errors everywhere. `robocli config schema` prints the
same information as JSON Schema.

## Files

| file | schema |
|---|---|
| `robots/<name>.yaml (bundled or ~/.robocli/robots/)` | [RobotProfile](#robotprofile) |
| `benchmarks/<name>.yaml (bundled or ~/.robocli/benchmarks/)` | [Benchmark](#benchmark) |
| `~/.robocli/config.yaml and the package's configs/config.yaml` | [UserConfig](#userconfig) |
| `agents/<name>.yaml (bundled or ~/.robocli/agents/)` | [AgentManifest](#agentmanifest) |
| `<trial>/config.yaml, the resolved view every party reads` | [ResolvedConfig](#resolvedconfig) |

## RobotProfile

| key | type | default | meaning |
|---|---|---|---|
| `world` | [World](#world) \| null | None |  |
| `machine` | [Machine](#machine) | **required** |  |

## World

The scene ``robocli up <robot>`` loads when no benchmark is named.

| key | type | default | meaning |
|---|---|---|---|
| `benchmark` | str | **required** |  |
| `task_suite` | str \| null | None |  |
| `task_id` | int | 0 |  |

## Machine

| key | type | default | meaning |
|---|---|---|---|
| `backend` | [SimBackend](#simbackend) \| [RealBackend](#realbackend) | **required** | how the robot is provided: kind: sim \| real |
| `workspace_template` | str \| null | 'workspace' | workspace tree seeded into the sandbox; null = none |
| `controller` | str | 'JOINT_POSITION' | robosuite controller |
| `controller_config` | str \| null | None | controller json: under robots/ (bundled, then ~/.robocli/robots/) or a path |
| `controller_kp_scale` | float | 10.0 |  |
| `cameras` | [Cameras](#cameras) |  |  |
| `control` | [Control](#control) |  |  |
| `ports` | [Ports](#ports) |  |  |
| `joint_name_map` | dict[str, str] | {} | simulator joint prefix -> published prefix |
| `robot` | [RobotFacts](#robotfacts) \| null | None |  |
| `frames` | [Frames](#frames) \| null | None |  |
| `arm` | [Arm](#arm) \| null | None |  |
| `arms` | list[[ArmSpec](#armspec)] \| null | None |  |
| `gripper` | [Gripper](#gripper) \| null | None |  |
| `hand` | [Hand](#hand) \| null | None |  |
| `planning` | [Planning](#planning) \| null | None |  |
| `tf` | [Tf](#tf) \| null | None |  |
| `base` | [Base](#base) \| null | None |  |

## RealBackend

A real robot: its ROS 2 graph is already there or a launch command starts it.

| key | type | default | meaning |
|---|---|---|---|
| `kind` | 'real' | **required** |  |
| `launch` | str \| null | None | command that brings the robot's ROS 2 graph up; null = already running |
| `image` | str \| null | None | docker image the launch command runs in, on the host network (a vendor driver pinned to its own ROS release); null = run it on the host |
| `discovery` | [Discovery](#discovery) | **required** |  |
| `sandbox_image` | str | 'robocli-sandbox' | agent terminal image (same ROS distro as the robot) |

## Discovery

How the sandbox reaches a real robot's ROS 2 graph; exactly one key.

| key | type | default | meaning |
|---|---|---|---|
| `network` | 'host' \| null | None | host: the sandbox joins the host network |
| `static_peers` | list[str] \| null | None | peer addresses for ROS_STATIC_PEERS |
| `discovery_server` | str \| null | None | host:port of a Fast DDS discovery server |

## SimBackend

A simulated robot: a container running the bridge over a simulator venv.

| key | type | default | meaning |
|---|---|---|---|
| `kind` | 'sim' | **required** |  |
| `image` | str | 'robocli-sim-jazzy' | simulated robot image |
| `sandbox_image` | str | 'robocli-sandbox' | agent terminal image (same ROS distro as the robot) |
| `gpus` | bool | False | render on the GPU (needs nvidia toolkit) |
| `resources` | dict[str, Any] \| null | None | render_threads: int \| off \| auto |
| `simulator` | [Simulator](#simulator) | **required** |  |

## Simulator

| key | type | default | meaning |
|---|---|---|---|
| `venv` | str | **required** | simulator venv: absolute, ~, or relative to ~/.robocli/simulators/ |
| `container` | str \| null | None | which sim image family (sim-jazzy \| sim-humble); documentation |

## Cameras

| key | type | default | meaning |
|---|---|---|---|
| `resolution` | list[int] | (640, 480) |  |
| `names` | list[str] \| null | None | camera names; null = every camera the scene defines |
| `rate_hz` | float | 2.0 | wall-clock publish rate |
| `render_mode` | 'on_demand' \| 'always' | 'on_demand' | render a camera only while subscribed, or always |

## Control

Graph-side actuation behaviour (the facts below feed machine.yaml).

| key | type | default | meaning |
|---|---|---|---|
| `gripper` | [GripperControl](#grippercontrol) |  |  |
| `trajectory` | [TrajectoryControl](#trajectorycontrol) |  |  |

## GripperControl

| key | type | default | meaning |
|---|---|---|---|
| `open_threshold_m` | float | 0.02 | GripperCommand position below this = close |
| `settle_steps` | int | 10 | control ticks a gripper goal advances |

## TrajectoryControl

| key | type | default | meaning |
|---|---|---|---|
| `goal_tolerance_rad` | float | 0.05 |  |
| `settle_steps` | int | 20 |  |

## Ports

ROS 2 names the machine exposes; null = this machine has no such port.

| key | type | default | meaning |
|---|---|---|---|
| `twist` | str \| null | None |  |
| `trajectory` | str \| null | None |  |
| `gripper` | str \| null | None |  |
| `wrench` | str \| null | None |  |
| `base_twist` | str \| null | None |  |
| `odom` | str \| null | None |  |

## RobotFacts

| key | type | default | meaning |
|---|---|---|---|
| `model` | str | **required** |  |
| `description` | str | '' |  |

## Frames

| key | type | default | meaning |
|---|---|---|---|
| `world` | str | 'world' |  |
| `base` | str \| null | 'panda_link0' |  |
| `hand` | str \| null | 'panda_hand' |  |

## Arm

| key | type | default | meaning |
|---|---|---|---|
| `joints` | list[str] | **required** |  |
| `limits_rad` | list[list[float]] | **required** |  |

## ArmSpec

One arm of a multi-arm machine (machine.arms); single-arm profiles are normalised into this shape by the loader.

| key | type | default | meaning |
|---|---|---|---|
| `label` | str | '' |  |
| `joints` | list[str] | [] |  |
| `limits_rad` | list[list[float]] | [] |  |
| `gripper` | [Gripper](#gripper) \| null | None |  |
| `ports` | [Ports](#ports) |  |  |
| `hand_body` | str | 'robot0_right_hand' |  |
| `tf_base_body` | str | 'robot0_base' |  |
| `base_frame` | str | 'panda_link0' |  |
| `hand_frame` | str \| null | 'panda_hand' |  |

## Gripper

| key | type | default | meaning |
|---|---|---|---|
| `open_m` | float | **required** |  |
| `closed_m` | float | **required** |  |
| `max_effort` | float | **required** |  |
| `stops_at` | list[str] | ['open', 'closed'] |  |

## Hand

| key | type | default | meaning |
|---|---|---|---|
| `tcp_offset_m` | float | **required** | hand frame -> fingertip grasp point |

## Planning

| key | type | default | meaning |
|---|---|---|---|
| `moveit` | bool | False |  |
| `move_action` | str | '/move_action' |  |
| `ik_service` | str | '/compute_ik' |  |
| `group` | str \| null | None |  |
| `planning_frame` | str \| null | None |  |

## Tf

| key | type | default | meaning |
|---|---|---|---|
| `hand` | bool | False | bridge publishes hand frames (MoveIt-less runs) |
| `base_body` | str | 'robot0_base' |  |

## Base

A mobile base, when the machine has one.

| key | type | default | meaning |
|---|---|---|---|
| `type` | str \| null | None |  |
| `body` | str \| null | None |  |
| `frame` | str | 'base_footprint' |  |
| `effective_speed_note` | str \| null | None |  |
| `cmd_vel` | str \| null | None |  |
| `odom` | str \| null | None |  |

## Benchmark

A benchmarks/<name>.yaml as written: names its robot or carries a machine.

| key | type | default | meaning |
|---|---|---|---|
| `task` | [Task](#task) | **required** |  |
| `protocol` | [Protocol](#protocol) |  |  |
| `agent` | [AgentConfig](#agentconfig) |  |  |
| `robot` | str \| null | None |  |
| `machine` | [Machine](#machine) \| null | None |  |
| `suite_overrides` | dict[str, dict[str, Any]] | {} | per-suite deep merge into the config; null deletes a key |

## Task

| key | type | default | meaning |
|---|---|---|---|
| `benchmark` | str | **required** | loader name: libero_pro \| capbench \| robocasa365 |
| `suites` | list[str] | **required** | task suites this benchmark runs; robocli run picks one with --task-suite |
| `init_states` | str \| null | None | how episodes start (documentation of the loader's behaviour): benchmark-files \| seeded-reset |
| `split` | str | 'target' | robocasa: object/layout split |
| `task_language` | dict[str, str] | {} | capbench: task name -> instruction |

## Protocol

| key | type | default | meaning |
|---|---|---|---|
| `trials_per_task` | int | 10 | episodes per task |
| `max_turns` | int | 500 | agent turns before the trial stops |
| `active_wall_clock_minutes` | float | 240 | active (not suspended) wall-clock cap per trial |
| `resume_on_quota_wall` | bool | False | suspend at an account quota wall and resume the same session (for subscription accounts run at scale); off: the trial ends |
| `max_quota_wait_minutes` | float | 360 | longest wait at a quota wall before giving up |
| `max_suspensions` | int | 4 | quota-wall suspensions per trial |
| `clock` | [Clock](#clock) |  |  |
| `horizon` | int | 1000000000 | simulator step cap (libero) |
| `base_cmd_policy` | 'queue' \| 'latest' | 'queue' | mobile base: queue every twist (one message = one tick) or let a new one replace the pending setpoint |
| `stall_probe_timeout_s` | float | 600 | seconds a simulator job may run before the trial is declared wedged |

## Clock

| key | type | default | meaning |
|---|---|---|---|
| `mode` | 'paused' \| 'free_running' | 'paused' | paused: the world only advances with commands; free_running is the sub-experiment switch |

## AgentConfig

| key | type | default | meaning |
|---|---|---|---|
| `name` | str \| null | None | agent name (robocli agents lists them); the package default lives in configs/config.yaml |
| `model` | str \| null | None | model id; default: the adapter's |
| `version` | str \| null | None | pin the agent CLI version: the sandbox image must carry it and preflight checks it; default: whatever the image has |
| `credentials_dir` | str \| null | None | login profile directory; default: ~/.robocli/credentials/<agent name> |
| `options` | dict[str, Any] | {} | adapter-specific knobs passed through as given, over the adapter's default_options |

## UserConfig

A defaults file: the package's configs/config.yaml or ~/.robocli/config.yaml.

| key | type | default | meaning |
|---|---|---|---|
| `agent` | [AgentOverrides](#agentoverrides) |  |  |
| `robot` | str \| null | None | robot when a command names none |
| `sandbox` | [SandboxConfig](#sandboxconfig) |  |  |

## AgentOverrides

A defaults file's agent section: same keys, no defaults, so only what the file wrote is layered in.

| key | type | default | meaning |
|---|---|---|---|
| `name` | str \| null | None |  |
| `model` | str \| null | None |  |
| `version` | str \| null | None |  |
| `credentials_dir` | str \| null | None |  |
| `options` | dict[str, Any] \| null | None |  |

## SandboxConfig

How this machine starts the sandbox container. A machine fact, so it lives in the defaults files, never in a benchmark config.

| key | type | default | meaning |
|---|---|---|---|
| `run_args` | list[str] | [] | flags appended to the sandbox's docker run, verbatim (rootless podman needs --userns=keep-id; see docs/podman.md) |

## AgentManifest

configs/agents/<name>.yaml: the facts about one coding agent, no code. The hooks module named by ``hooks`` supplies the behaviour.

| key | type | default | meaning |
|---|---|---|---|
| `name` | str | **required** |  |
| `default_model` | str | **required** |  |
| `binary` | str \| null | None | executable name inside the sandbox |
| `version` | str \| null | None | a CLI version this manifest pins; usually null: pins come from name@version at build time or agent.version in a config |
| `install` | str | '' | shell that installs the agent in the sandbox image |
| `whitelist` | list[str] | [] | regexes of the hosts the proxy lets the agent reach |
| `credentials` | [Credentials](#credentials) \| null | None |  |
| `token_env` | str \| null | None | environment variable carrying an auth token |
| `version_argv` | list[str] \| null | None | command printing the agent's version |
| `instruction_file` | str \| null | None | the file the agent reads instructions from, e.g. AGENTS.md |
| `default_options` | dict[str, Any] | {} |  |
| `hooks` | str \| null | None | hooks module name under plugins/agents/ (bundled, then ~/.robocli/plugins/agents/) |

## Credentials

Where an agent keeps its login and how the sandbox is told about it.

| key | type | default | meaning |
|---|---|---|---|
| `dirname` | str | **required** | profile directory name under ~/.robocli/credentials/ |
| `filename` | str | **required** | the credentials file inside that directory |
| `config_env` | str | **required** | environment variable naming the profile directory |
| `mount_point` | str | **required** | where the profile is mounted inside the sandbox |

## ResolvedConfig

The resolved config every party reads (``<trial>/config.yaml``).

| key | type | default | meaning |
|---|---|---|---|
| `task` | [Task](#task) | **required** |  |
| `protocol` | [Protocol](#protocol) |  |  |
| `agent` | [AgentConfig](#agentconfig) |  |  |
| `machine` | [Machine](#machine) | **required** |  |
| `sandbox` | [SandboxConfig](#sandboxconfig) |  |  |
| `suite_overrides` | dict[str, dict[str, Any]] | {} |  |
