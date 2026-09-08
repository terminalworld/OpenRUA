---
summary: Every key of the robot profiles, benchmark configs, agent manifests and the defaults file
read_when:
  - You are writing a robot profile, a benchmark config or an agent manifest
  - A config error names a key and you want its meaning and allowed values
---

# Configuration

Generated from the schema by `scripts/render_docs.py`; edit the
`Field(description=...)` in `openrua/config/schema.py`, not this page.
Unknown keys are errors everywhere. `openrua config schema` prints the
same information as JSON Schema.

## Files

| file | schema |
|---|---|
| `robots/<name>.yaml (bundled or ~/.openrua/robots/)` | [RobotProfile](#robotprofile) |
| `benchmarks/<name>.yaml (bundled or ~/.openrua/benchmarks/)` | [Benchmark](#benchmark) |
| `~/.openrua/config.yaml and the package's configs/config.yaml` | [UserConfig](#userconfig) |
| `agents/<name>.yaml (bundled or ~/.openrua/agents/)` | [AgentManifest](#agentmanifest) |
| `<trial>/config.yaml, the resolved view every party reads` | [ResolvedConfig](#resolvedconfig) |

## RobotProfile

A robots/<name>.yaml: the robot, and the scene ``up`` opens by default.

| key | type | default | meaning |
|---|---|---|---|
| `world` | [World](#world) \| null | None | default scene for openrua up |
| `machine` | [Machine](#machine) | **required** | the robot |

## World

The scene ``openrua up <robot>`` loads when no benchmark is named.

| key | type | default | meaning |
|---|---|---|---|
| `benchmark` | str | **required** | benchmark config the scene comes from |
| `task_suite` | str \| null | None | suite; default: the benchmark's first |
| `task_id` | int | 0 | scene index within the suite |

## Machine

| key | type | default | meaning |
|---|---|---|---|
| `backend` | [SimBackend](#simbackend) \| [RealBackend](#realbackend) | **required** | how the robot is provided: kind: sim \| real |
| `workspace_template` | str \| null | 'workspace' | workspace tree seeded into the sandbox; null = none |
| `controller` | str | 'JOINT_POSITION' | robosuite controller |
| `controller_config` | str \| null | None | controller json: under robots/ (bundled, then ~/.openrua/robots/) or a path |
| `controller_kp_scale` | float | 10.0 | multiplier on the simulator's joint position gains |
| `cameras` | [Cameras](#cameras) |  | the cameras the graph publishes |
| `control` | [Control](#control) |  | how goals are executed and judged |
| `ports` | [Ports](#ports) |  | the ROS 2 names the robot serves (single arm) |
| `joint_name_map` | dict[str, str] | {} | simulator joint prefix -> published prefix |
| `robot` | [RobotFacts](#robotfacts) \| null | None | model and description |
| `frames` | [Frames](#frames) \| null | None | world, base and hand frames |
| `arm` | [Arm](#arm) \| null | None | the arm (single-arm profiles) |
| `arms` | list[[ArmSpec](#armspec)] \| null | None | the arms (multi-arm profiles); replaces arm, gripper and ports |
| `gripper` | [Gripper](#gripper) \| null | None | the gripper; null = none |
| `hand` | [Hand](#hand) \| null | None | hand geometry |
| `planning` | [Planning](#planning) \| null | None | the MoveIt planner |
| `tf` | [Tf](#tf) \| null | None | what the bridge publishes on TF |
| `base` | [Base](#base) \| null | None | the mobile base; null = fixed |

## RealBackend

A real robot: its ROS 2 graph is already there or a launch command starts it.

| key | type | default | meaning |
|---|---|---|---|
| `kind` | 'real' | **required** | a real robot |
| `ros_distro` | 'jazzy' \| 'humble' | 'jazzy' | the ROS 2 distro the robot runs; the sandbox image is named after it |
| `launch` | str \| null | None | command that brings the robot's ROS 2 graph up; null = already running |
| `image` | str \| null | None | docker image the launch command runs in, on the host network (a vendor driver pinned to its own ROS release); null = run it on the host |
| `discovery` | [Discovery](#discovery) | **required** | how the sandbox reaches the graph |
| `sandbox_image` | str \| null | None | agent terminal image; default: openrua-sandbox-<ros_distro> |

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
| `kind` | 'sim' | **required** | a simulated robot |
| `ros_distro` | 'jazzy' \| 'humble' | 'jazzy' | the ROS 2 distro the robot runs; the robot and sandbox images are named after it |
| `image` | str \| null | None | simulated robot image; default: openrua-sim-<ros_distro> |
| `sandbox_image` | str \| null | None | agent terminal image; default: openrua-sandbox-<ros_distro> |
| `gpus` | bool | False | render on the GPU (needs nvidia toolkit) |
| `resources` | dict[str, Any] \| null | None | render_threads: int \| off \| auto |
| `simulator` | [Simulator](#simulator) | **required** | the simulator venv the bridge runs in |

## Simulator

| key | type | default | meaning |
|---|---|---|---|
| `venv` | str | **required** | simulator venv: absolute, ~, or relative to ~/.openrua/simulators/ |
| `container` | str \| null | None | which sim image family (sim-jazzy \| sim-humble); documentation |

## Cameras

| key | type | default | meaning |
|---|---|---|---|
| `resolution` | list[int] | (640, 480) | width, height in pixels for every camera |
| `names` | list[str] \| null | None | camera names; null = every camera the scene defines |
| `rate_hz` | float | 2.0 | wall-clock publish rate |
| `render_mode` | 'on_demand' \| 'always' | 'on_demand' | render a camera only while subscribed, or always |
| `record` | list[str] \| null | None | the cameras `openrua bench --record` captures when none are named: the first is a demo's main view, the second its inset |

## Control

Graph-side actuation behaviour (the facts below feed machine.yaml).

| key | type | default | meaning |
|---|---|---|---|
| `gripper` | [GripperControl](#grippercontrol) |  | how a GripperCommand goal is executed |
| `trajectory` | [TrajectoryControl](#trajectorycontrol) |  | how a FollowJointTrajectory goal is judged |

## GripperControl

| key | type | default | meaning |
|---|---|---|---|
| `open_threshold_m` | float | 0.02 | GripperCommand position below this = close |
| `settle_steps` | int | 10 | control ticks a gripper goal advances |

## TrajectoryControl

| key | type | default | meaning |
|---|---|---|---|
| `goal_tolerance_rad` | float | 0.05 | a FollowJointTrajectory goal succeeds when every joint is within this of its last point, radians |
| `settle_steps` | int | 20 | control ticks the arm keeps tracking after the last point before the result is judged |

## Ports

ROS 2 names the machine exposes; null = this machine has no such port.

| key | type | default | meaning |
|---|---|---|---|
| `twist` | str \| null | None | TwistStamped topic for Cartesian servoing of the hand |
| `trajectory` | str \| null | None | FollowJointTrajectory action |
| `gripper` | str \| null | None | GripperCommand action |
| `wrench` | str \| null | None | WrenchStamped topic of the wrist force-torque sensor |
| `base_twist` | str \| null | None | Twist topic driving a mobile base |
| `odom` | str \| null | None | Odometry topic of a mobile base |

## RobotFacts

What the agent is told the robot is.

| key | type | default | meaning |
|---|---|---|---|
| `model` | str | **required** | the model as a person would name it |
| `description` | str | '' | one line: arm, gripper, base |

## Frames

The TF frames the workspace docs refer to.

| key | type | default | meaning |
|---|---|---|---|
| `world` | str | 'world' | the fixed frame the robot plans in |
| `base` | str \| null | 'panda_link0' | the arm's root link |
| `hand` | str \| null | 'panda_hand' | the hand frame that twist commands and IK targets refer to |

## Arm

| key | type | default | meaning |
|---|---|---|---|
| `joints` | list[str] | **required** | joint names in the order the trajectory port expects them |
| `limits_rad` | list[list[float]] | **required** | travel per joint as [min, max], radians |

## ArmSpec

One arm of a multi-arm machine (machine.arms); single-arm profiles are normalised into this shape by the loader.

| key | type | default | meaning |
|---|---|---|---|
| `label` | str | '' | how the docs name this arm (left, right); empty on a single arm |
| `joints` | list[str] | [] | joint names in the order the trajectory port expects them |
| `limits_rad` | list[list[float]] | [] | travel per joint as [min, max], radians |
| `gripper` | [Gripper](#gripper) \| null | None | this arm's gripper; null = a tool with no gripper |
| `ports` | [Ports](#ports) |  | this arm's ports |
| `hand_body` | str | 'robot0_right_hand' | simulator body the hand frame is read from |
| `tf_base_body` | str | 'robot0_base' | simulator body the base frame is read from |
| `base_frame` | str | 'panda_link0' | the arm's root link |
| `hand_frame` | str \| null | 'panda_hand' | the hand frame |

## Gripper

| key | type | default | meaning |
|---|---|---|---|
| `open_m` | float | **required** | fingers fully open: position of one finger, metres (the gap is twice this) |
| `closed_m` | float | **required** | fingers fully closed: position of one finger, metres |
| `max_effort` | float | **required** | force ceiling a GripperCommand goal may ask for, newtons |
| `stops_at` | list[str] | ['open', 'closed'] | the only positions the gripper comes to rest at; a commanded width is read as open or closed |

## Hand

| key | type | default | meaning |
|---|---|---|---|
| `tcp_offset_m` | float | **required** | hand frame -> fingertip grasp point |

## Planning

The MoveIt planner, when the graph runs one.

| key | type | default | meaning |
|---|---|---|---|
| `moveit` | bool | False | whether a MoveIt planner runs on this robot |
| `move_action` | str | '/move_action' | MoveIt's plan-and-execute action |
| `ik_service` | str | '/compute_ik' | inverse-kinematics service |
| `group` | str \| null | None | the MoveIt planning group covering the arm |
| `planning_frame` | str \| null | None | the frame MoveIt plans in |

## Tf

| key | type | default | meaning |
|---|---|---|---|
| `hand` | bool | False | bridge publishes hand frames (MoveIt-less runs) |
| `base_body` | str | 'robot0_base' | simulator body the base frame is read from |

## Base

A mobile base, when the machine has one.

| key | type | default | meaning |
|---|---|---|---|
| `type` | str \| null | None | the base as a person would name it (omnidirectional, differential) |
| `body` | str \| null | None | simulator body odometry is read from |
| `frame` | str | 'base_footprint' | the base's TF frame |
| `effective_speed_note` | str \| null | None | one line for the agent on how commanded speed maps to motion |
| `cmd_vel` | str \| null | None | Twist topic driving the base |
| `odom` | str \| null | None | Odometry topic |

## Benchmark

A benchmarks/<name>.yaml as written: names its robot or carries a machine.

| key | type | default | meaning |
|---|---|---|---|
| `task` | [Task](#task) | **required** | what is run |
| `protocol` | [Protocol](#protocol) |  | budgets and clock |
| `agent` | [AgentConfig](#agentconfig) |  | which agent, over the defaults files |
| `robot` | str \| null | None | robot profile name or path; --robot overrides it |
| `machine` | [Machine](#machine) \| null | None | a robot written inline instead of named |
| `suite_overrides` | dict[str, dict[str, Any]] | {} | per-suite deep merge into the config; null deletes a key |

## Task

| key | type | default | meaning |
|---|---|---|---|
| `benchmark` | str | **required** | loader name: libero_pro \| capbench \| robocasa365 |
| `suites` | list[str] | **required** | task suites this benchmark runs; openrua bench picks one with --task-suite |
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
| `clock` | [Clock](#clock) |  | how simulated time advances |
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
| `name` | str \| null | None | agent name (openrua agents lists them); the package default lives in configs/config.yaml |
| `model` | str \| null | None | model id; default: the adapter's |
| `version` | str \| null | None | pin the agent CLI version: the sandbox image must carry it and preflight checks it; default: whatever the image has |
| `credentials_dir` | str \| null | None | login profile directory; default: ~/.openrua/credentials/<agent name> |
| `options` | dict[str, Any] | {} | adapter-specific knobs passed through as given, over the adapter's default_options |

## UserConfig

A defaults file: the package's configs/config.yaml or ~/.openrua/config.yaml.

| key | type | default | meaning |
|---|---|---|---|
| `agent` | [AgentOverrides](#agentoverrides) |  | agent defaults; only the keys written are layered in |
| `robot` | str \| null | None | robot when a command names none |
| `sandbox` | [SandboxConfig](#sandboxconfig) |  | how this machine starts the sandbox |

## AgentOverrides

A defaults file's agent section: same keys, no defaults, so only what the file wrote is layered in.

| key | type | default | meaning |
|---|---|---|---|
| `name` | str \| null | None | see AgentConfig |
| `model` | str \| null | None | see AgentConfig |
| `version` | str \| null | None | see AgentConfig |
| `credentials_dir` | str \| null | None | see AgentConfig |
| `options` | dict[str, Any] \| null | None | merged key by key with the layers above |

## SandboxConfig

How this machine starts the sandbox container. A machine fact, so it lives in the defaults files, never in a benchmark config.

| key | type | default | meaning |
|---|---|---|---|
| `run_args` | list[str] | [] | flags appended to the sandbox's docker run, verbatim (rootless podman needs --userns=keep-id; see docs/podman.md) |

## AgentManifest

configs/agents/<name>.yaml: the facts about one coding agent, no code. The hooks module named by ``hooks`` supplies the behaviour.

| key | type | default | meaning |
|---|---|---|---|
| `name` | str | **required** | the name configs use under agent.name |
| `default_model` | str | **required** | model when the config names none |
| `binary` | str \| null | None | executable name inside the sandbox |
| `version` | str \| null | None | a CLI version this manifest pins; usually null: pins come from name@version at build time or agent.version in a config |
| `install` | str | '' | shell that installs the agent in the sandbox image |
| `whitelist` | list[str] | [] | regexes of the hosts the proxy lets the agent reach |
| `credentials` | [Credentials](#credentials) \| null | None | profile-directory login; null = token only |
| `token_env` | str \| null | None | environment variable carrying an auth token |
| `version_argv` | list[str] \| null | None | command printing the agent's version |
| `instruction_file` | str \| null | None | the file the agent reads instructions from, e.g. AGENTS.md |
| `default_options` | dict[str, Any] | {} | knobs a config may override under agent.options |
| `hooks` | str \| null | None | hooks module name under plugins/agents/ (bundled, then ~/.openrua/plugins/agents/) |

## Credentials

Where an agent keeps its login and how the sandbox is told about it.

| key | type | default | meaning |
|---|---|---|---|
| `dirname` | str | **required** | profile directory name under ~/.openrua/credentials/ |
| `filename` | str | **required** | the credentials file inside that directory |
| `config_env` | str | **required** | environment variable naming the profile directory |
| `mount_point` | str | **required** | where the profile is mounted inside the sandbox |

## ResolvedConfig

The resolved config every party reads (``<trial>/config.yaml``).

| key | type | default | meaning |
|---|---|---|---|
| `task` | [Task](#task) | **required** | what is run |
| `protocol` | [Protocol](#protocol) |  | budgets and clock |
| `agent` | [AgentConfig](#agentconfig) |  | the agent, layered over the defaults files |
| `machine` | [Machine](#machine) | **required** | the robot |
| `sandbox` | [SandboxConfig](#sandboxconfig) |  | from the defaults files |
| `suite_overrides` | dict[str, dict[str, Any]] | {} | as written; applied for the suite that runs |
