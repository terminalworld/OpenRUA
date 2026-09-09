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
| `robots/<type>.yaml (bundled or ~/.openrua/robots/): a robot type` | [RobotType](#robottype) |
| `robots/<name>.yaml: a particular robot (type: + machine:)` | [RobotInstance](#robotinstance) |
| `simulators/<engine>.yaml (bundled or ~/.openrua/simulators/)` | [SimulatorProfile](#simulatorprofile) |
| `benchmarks/<name>.yaml (bundled or ~/.openrua/benchmarks/)` | [Benchmark](#benchmark) |
| `~/.openrua/config.yaml and the package's configs/config.yaml` | [UserConfig](#userconfig) |
| `agents/<name>.yaml (bundled or ~/.openrua/agents/)` | [AgentManifest](#agentmanifest) |
| `<trial>/config.yaml, the resolved view every party reads` | [ResolvedConfig](#resolvedconfig) |

## RobotType

A robots/<type>.yaml: what is true of this robot wherever it runs. No cameras, no backend, no scene: those come from the simulator or the benchmark that embodies it.

| key | type | default | meaning |
|---|---|---|---|
| `robot` | [RobotFacts](#robotfacts) | **required** | model and description |
| `frames` | [Frames](#frames) \| null | None | world, base and hand frames |
| `arm` | [Arm](#arm) \| null | None | the arm (single-arm robots) |
| `arms` | list[[ArmSpec](#armspec)] \| null | None | the arms (multi-arm robots); replaces arm, gripper and ports |
| `gripper` | [Gripper](#gripper) \| null | None | the gripper; null = none |
| `hand` | [Hand](#hand) \| null | None | hand geometry |
| `ports` | [Ports](#ports) |  | the ROS 2 names the robot serves (single arm) |
| `planning` | [Planning](#planning) \| null | None | the MoveIt planner |
| `base` | [Base](#base) \| null | None | the mobile base; null = fixed |
| `workspace_template` | str \| null | 'workspace' | workspace tree seeded into the sandbox; null = none |

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

## RobotInstance

A robots/<name>.yaml describing one particular robot, usually a real one: its ``machine:`` (backend and facts), optionally over a robot type's facts (``type:``).

| key | type | default | meaning |
|---|---|---|---|
| `type` | str \| null | None | robot type this is an instance of; its facts come first, machine: writes over them |
| `machine` | [Machine](#machine) | **required** | backend (kind: real \| sim) and facts |

## Machine

| key | type | default | meaning |
|---|---|---|---|
| `backend` | [SimBackend](#simbackend) \| [RealBackend](#realbackend) | **required** | how the robot is provided: kind: sim \| real |
| `workspace_template` | str \| null | 'workspace' | workspace tree seeded into the sandbox; null = none |
| `engine_model` | str \| null | None | the robot's name inside the simulator (robosuite: Panda, PandaOmron) |
| `controller` | str | 'JOINT_POSITION' | robosuite controller |
| `controller_config` | str \| null | None | controller json: a bundled name under robots/controllers/ or a path relative to the file naming it |
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
| `engine` | str \| null | None | the bridge engine module (resolved from the simulator's entry_point: a module path, or an absolute .py file copied next to the config) |
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

## Tf

| key | type | default | meaning |
|---|---|---|---|
| `hand` | bool | False | bridge publishes hand frames (MoveIt-less runs) |
| `base_body` | str | 'robot0_base' | simulator body the base frame is read from |

## SimulatorProfile

A simulators/<engine>.yaml: the engine, its install, its native scene, and how it drives each robot type it embodies.

| key | type | default | meaning |
|---|---|---|---|
| `engine` | str | **required** | the engine, as a person would name it (robosuite 1.5 on MuJoCo) |
| `entry_point` | str | **required** | the bridge engine: a bundled name (robosuite) or a path to a module of your own, relative to this file |
| `install` | [Install](#install) | **required** | the venv and distro the bridge runs with |
| `native` | [NativeScene](#nativescene) \| null | None | scene loaded with no benchmark; null = a benchmark is required |
| `robots` | dict[str, [Embodiment](#embodiment)] | {} | robot type name -> how this engine drives it |

## Install

A simulator install: the venv the bridge runs in, the ROS distro that goes with its Python, and the recipe ``openrua install`` renders to a script and runs to build it.

| key | type | default | meaning |
|---|---|---|---|
| `venv` | str | **required** | simulator venv: absolute, ~, or relative to ~/.openrua/simulators/ |
| `ros_distro` | 'jazzy' \| 'humble' | 'jazzy' | the ROS 2 distro the robot runs; the robot and sandbox images are named after it |
| `python` | str \| null | None | the venv's Python (3.12 goes with Jazzy, 3.10 with Humble); required to install |
| `checkouts` | list[[Checkout](#checkout)] | [] | repositories cloned at pinned commits |
| `requirements` | str \| null | None | a pip requirements lock installed into the venv, relative to the file naming it |
| `editable` | list[str] | [] | checkouts installed editable with --no-deps (the lock has their dependencies), relative to ~/.openrua/simulators/ |
| `shell` | str \| null | None | shell run last, in the venv, for what the fields above cannot say (asset downloads); {root} = ~/.openrua/simulators, {venv} = the venv, {here} = the directory of the file naming it |
| `container` | str \| null | None | which sim image family (sim-jazzy \| sim-humble); documentation |
| `image` | str \| null | None | simulated robot image; default: openrua-sim-<ros_distro> |
| `sandbox_image` | str \| null | None | agent terminal image; default: openrua-sandbox-<ros_distro> |
| `gpus` | bool | False | render on the GPU (needs nvidia toolkit) |
| `resources` | dict[str, Any] \| null | None | render_threads: int \| off \| auto |

## Checkout

One repository ``openrua install`` clones at a pinned commit.

| key | type | default | meaning |
|---|---|---|---|
| `path` | str | **required** | where it lands, relative to ~/.openrua/simulators/ |
| `repo` | str | **required** | git URL |
| `commit` | str | **required** | the commit checked out (a full hash) |
| `submodules` | list[str] | [] | submodule paths to initialise, relative to the checkout |
| `patch` | str \| null | None | a patch applied to the checkout, relative to the file naming it; already-applied is fine |

## NativeScene

What ``openrua run <robot> --sim <engine>`` loads with no benchmark.

| key | type | default | meaning |
|---|---|---|---|
| `entry_point` | str | **required** | the bridge loader for the engine's own scenes: a bundled name (robosuite) or a path to a module of your own, relative to this file |
| `scene` | str | **required** | the engine's own scene / env name (robosuite: Lift) |
| `cameras` | [Cameras](#cameras) |  | the scene's cameras |

## Embodiment

How an engine drives one robot type: the keys that depend on the simulator, merged over the robot type. Written under a simulator's ``robots:`` (the engine's own robots) or a benchmark's ``scenes.robots:`` (robots the benchmark's assets add or adjust). Only the keys written are merged in.

| key | type | default | meaning |
|---|---|---|---|
| `engine_model` | str \| null | None | the robot's name inside the engine (robosuite: Panda, PandaOmron) |
| `controller` | str \| null | None | robosuite controller type |
| `controller_config` | str \| null | None | controller json: a bundled name under robots/controllers/ or a path relative to the file naming it |
| `controller_kp_scale` | float \| null | None | multiplier on the engine's joint position gains |
| `joint_name_map` | dict[str, str] \| null | None | engine joint prefix -> published prefix |
| `control` | [Control](#control) \| null | None | how goals are executed and judged |
| `tf` | [Tf](#tf) \| null | None | what the bridge publishes on TF |
| `base` | [Base](#base) \| null | None | mobile-base keys the engine decides (body, effective speed) |
| `cameras` | [Cameras](#cameras) \| null | None | robot-mounted cameras this embodiment adds |

## Benchmark

A benchmarks/<name>.yaml as written.

| key | type | default | meaning |
|---|---|---|---|
| `entry_point` | str | **required** | the bridge loader building, resetting and scoring this benchmark's scenes: a bundled name (libero, capbench, robocasa) or a path to a module of your own, relative to this file; the module exposes LOADER |
| `task` | [Task](#task) | **required** | what is run |
| `protocol` | [Protocol](#protocol) |  | budgets and clock |
| `agent` | [AgentConfig](#agentconfig) |  | which agent, over the defaults files |
| `robot` | str \| null | None | robot type (or instance) name or path; --robot overrides it |
| `simulator` | str \| null | None | simulator name or path; --sim overrides it; null with a real-robot instance |
| `install` | [InstallOverrides](#installoverrides) \| null | None | this benchmark's own venv and distro, over the simulator's |
| `scenes` | [Scenes](#scenes) |  | what the benchmark brings into the world |
| `machine` | [Machine](#machine) \| null | None | a robot written inline instead of assembled |
| `suite_overrides` | dict[str, dict[str, Any]] | {} | per-suite deep merge into the config; null deletes a key |

## Task

| key | type | default | meaning |
|---|---|---|---|
| `benchmark` | str | **required** | the benchmark's name as records carry it: libero_pro \| capbench \| robocasa365 \| the engine's name for a native scene |
| `loader` | str \| null | None | the loader the bridge imports, resolved by openrua from the benchmark's (or native scene's) entry_point: a module path or an absolute file path; not written by hand |
| `suites` | list[str] | **required** | task suites this benchmark runs; openrua bench picks one with --task-suite |
| `init_states` | str \| null | None | how episodes start (documentation of the loader's behaviour): benchmark-files \| seeded-reset |
| `split` | str | 'target' | robocasa: object/layout split (robocasa365: target \| pretrain \| all; robocasa: eval \| train \| all) |
| `dataset_root` | str \| null | None | a benchmark dataset kept outside its checkout (robocerebra: the RoboCerebra_Bench directory); default: next to the checkout |
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

## InstallOverrides

A benchmark's install section: same keys as Install, none required; only what is written replaces the simulator's, key by key. A benchmark with a venv of its own writes the whole recipe.

| key | type | default | meaning |
|---|---|---|---|
| `venv` | str \| null | None | see Install |
| `ros_distro` | 'jazzy' \| 'humble' \| null | None | see Install |
| `python` | str \| null | None | see Install |
| `checkouts` | list[[Checkout](#checkout)] \| null | None | see Install |
| `requirements` | str \| null | None | see Install |
| `editable` | list[str] \| null | None | see Install |
| `shell` | str \| null | None | see Install |
| `container` | str \| null | None | see Install |
| `image` | str \| null | None | see Install |
| `sandbox_image` | str \| null | None | see Install |
| `gpus` | bool \| null | None | see Install |
| `resources` | dict[str, Any] \| null | None | see Install |

## Scenes

What a benchmark brings into the simulator's world.

| key | type | default | meaning |
|---|---|---|---|
| `default_suite` | str \| null | None | the suite openrua run / up load when none is named; default: the first in task.suites |
| `cameras` | [Cameras](#cameras) \| null | None | the scene cameras (over the simulator's native ones) |
| `robots` | dict[str, [Embodiment](#embodiment)] | {} | robot type name -> embodiment this benchmark's assets add or adjust (merged over the simulator's) |

## UserConfig

A defaults file: the package's configs/config.yaml or ~/.openrua/config.yaml. Four default names (robot, simulator, benchmark, agent) and, per agent name, this machine's facts about it.

| key | type | default | meaning |
|---|---|---|---|
| `agent` | str \| null | None | agent when a command names none (openrua agents lists them) |
| `agents` | dict[str, [AgentFacts](#agentfacts)] | {} | agent name -> this machine's facts about it (model, version, credentials_dir, options) |
| `robot` | str \| null | None | robot when a command names none |
| `simulator` | str \| null | None | simulator when a command names none (and the robot is a type) |
| `benchmark` | str \| null | None | benchmark whose world openrua run / up load when --bench is not given; null = the simulator's native scene |
| `sandbox` | [SandboxConfig](#sandboxconfig) |  | how this machine starts the sandbox |

## AgentFacts

What a machine knows about one agent (``agents.<name>`` in a defaults file): the model it runs, a CLI version pin, where its login lives, its default knobs. No defaults: only what is written is layered in.

| key | type | default | meaning |
|---|---|---|---|
| `model` | str \| null | None | model id this agent runs by default on this machine |
| `version` | str \| null | None | pin the agent CLI version |
| `credentials_dir` | str \| null | None | login profile directory; default: ~/.openrua/credentials/<name> |
| `options` | dict[str, Any] \| null | None | merged key by key with the layers above |

## SandboxConfig

How this machine starts the sandbox container. A machine fact, so it lives in the defaults files, never in a benchmark config.

| key | type | default | meaning |
|---|---|---|---|
| `run_args` | list[str] | [] | flags appended to the sandbox's docker run, verbatim (rootless podman needs --userns=keep-id; see docs/podman.md) |

## AgentManifest

configs/agents/<name>.yaml: the facts about one coding agent, no code. The module named by ``entry_point`` supplies the behaviour.

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
| `entry_point` | str \| null | None | the module behind this manifest, exposing HOOKS (an Agent subclass): a bundled name under plugins/agents/ or a path to a module of your own, relative to this file |

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
