"""The configuration schema: what a robot profile, a benchmark config,
an agent manifest and the resolved per-trial config may say, with
defaults and a one-line description on every field.

Three files feed one resolved config:

- a robot profile (``robots/<name>.yaml``): ``machine:`` (the robot and
  its runtime shell) plus, for simulated ones, ``world:`` (the scene
  ``robocli up`` loads when no benchmark is named);
- a benchmark config (``benchmarks/<name>.yaml``): ``task:``,
  ``protocol:``, ``agent:``, either ``robot: <name>`` or its own
  ``machine:``, and optional ``suite_overrides:``;
- the user's ``~/.robocli/config.yaml``: defaults for the ``agent:``
  section and a default robot, applied under whatever the benchmark
  config says.

Layering, lowest first: the defaults declared here, the package's
``configs/config.yaml``, the user file, the benchmark config,
command-line flags. Unknown keys are errors at every
level: a misspelled key must fail the load, never silently do nothing.
Defaults are the values the paper's own runs use (RoboCLI-Dev configs);
protocol switches that only matter when running at scale on subscription
accounts (``resume_on_quota_wall``) default off.

The models validate; consumers keep reading plain dicts
(``loader.dump()``), so the container side never imports pydantic.
Leaf module: imports nothing from robocli.
"""

from __future__ import annotations

from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


# ------------------------------------------------------------------ task

class Task(Strict):
    benchmark: str = Field(description="loader name: libero_pro | capbench | robocasa365")
    suites: list[str] = Field(description="task suites this benchmark runs; "
                              "robocli run picks one with --task-suite")
    init_states: str | None = Field(
        default=None, description="how episodes start (documentation of the "
        "loader's behaviour): benchmark-files | seeded-reset")
    split: str = Field(default="target", description="robocasa: object/layout split")
    task_language: dict[str, str] = Field(
        default_factory=dict, description="capbench: task name -> instruction")


# -------------------------------------------------------------- protocol

class Clock(Strict):
    mode: Literal["paused", "free_running"] = Field(
        default="paused", description="paused: the world only advances with "
        "commands; free_running is the sub-experiment switch")


class Protocol(Strict):
    trials_per_task: int = Field(default=10, description="episodes per task")
    max_turns: int = Field(default=500, description="agent turns before the trial stops")
    active_wall_clock_minutes: float = Field(
        default=240, description="active (not suspended) wall-clock cap per trial")
    resume_on_quota_wall: bool = Field(
        default=False, description="suspend at an account quota wall and resume the "
        "same session (for subscription accounts run at scale); off: the trial ends")
    max_quota_wait_minutes: float = Field(
        default=360, description="longest wait at a quota wall before giving up")
    max_suspensions: int = Field(default=4, description="quota-wall suspensions per trial")
    clock: Clock = Field(default_factory=Clock)
    horizon: int = Field(default=10**9, description="simulator step cap (libero)")
    base_cmd_policy: Literal["queue", "latest"] = Field(
        default="queue", description="mobile base: queue every twist (one message = "
        "one tick) or let a new one replace the pending setpoint")
    stall_probe_timeout_s: float = Field(
        default=600, description="seconds a simulator job may run before the trial is "
        "declared wedged")


# ----------------------------------------------------------------- agent

class AgentConfig(Strict):
    name: str | None = Field(default=None, description="agent name (robocli agents lists "
                             "them); the package default lives in configs/config.yaml")
    model: str | None = Field(default=None, description="model id; default: the adapter's")
    credentials_dir: str | None = Field(
        default=None, description="login profile directory; default: "
        "~/.robocli/credentials/<agent name>")
    options: dict[str, Any] = Field(
        default_factory=dict, description="adapter-specific knobs passed through as "
        "given, over the adapter's default_options")


class AgentOverrides(Strict):
    """A defaults file's agent section: same keys, no defaults, so only
    what the file wrote is layered in."""
    name: str | None = None
    model: str | None = None
    credentials_dir: str | None = None
    options: dict[str, Any] | None = None


# --------------------------------------------------------------- machine

class Simulator(Strict):
    venv: str = Field(description="simulator venv: absolute, ~, or relative to "
                      "~/.robocli/simulators/")
    container: str | None = Field(default=None, description="which sim image family "
                                  "(sim-jazzy | sim-humble); documentation")


class SimBackend(Strict):
    """A simulated robot: a container running the bridge over a simulator venv."""
    kind: Literal["sim"]
    image: str = Field(default="robocli-sim-jazzy", description="simulated robot image")
    sandbox_image: str = Field(default="robocli-sandbox", description="agent terminal image "
                               "(same ROS distro as the robot)")
    gpus: bool = Field(default=False, description="render on the GPU (needs nvidia toolkit)")
    resources: dict[str, Any] | None = Field(
        default=None, description="render_threads: int | off | auto")
    simulator: Simulator


class Discovery(Strict):
    """How the sandbox reaches a real robot's ROS 2 graph; exactly one key."""
    network: Literal["host"] | None = Field(
        default=None, description="host: the sandbox joins the host network")
    static_peers: list[str] | None = Field(
        default=None, description="peer addresses for ROS_STATIC_PEERS")
    discovery_server: str | None = Field(
        default=None, description="host:port of a Fast DDS discovery server")

    @model_validator(mode="after")
    def _exactly_one(self):
        chosen = [k for k in ("network", "static_peers", "discovery_server")
                  if getattr(self, k) is not None]
        if len(chosen) != 1:
            raise ValueError("discovery needs exactly one of network, static_peers, "
                             f"discovery_server (got {chosen or 'none'})")
        return self


class RealBackend(Strict):
    """A real robot: its ROS 2 graph is already there or a launch command starts it."""
    kind: Literal["real"]
    launch: str | None = Field(default=None, description="command that brings the "
                               "robot's ROS 2 graph up; null = already running")
    image: str | None = Field(default=None, description="docker image the launch "
                              "command runs in, on the host network (a vendor driver "
                              "pinned to its own ROS release); null = run it on the host")
    discovery: Discovery

    @model_validator(mode="after")
    def _image_needs_launch(self):
        if self.image and not self.launch:
            raise ValueError("backend.image needs a launch command to run in it")
        return self
    sandbox_image: str = Field(default="robocli-sandbox", description="agent terminal image "
                               "(same ROS distro as the robot)")


Backend = Annotated[SimBackend | RealBackend, Field(discriminator="kind")]


class Cameras(Strict):
    resolution: tuple[int, int] = (640, 480)
    # yaml key is ``list``; the attribute cannot be, so it carries an alias
    names: list[str] | None = Field(default=None, alias="list",
                                    description="camera names; null = every "
                                    "camera the scene defines")
    rate_hz: float = Field(default=2.0, description="wall-clock publish rate")
    render_mode: Literal["on_demand", "always"] = Field(
        default="on_demand", description="render a camera only while subscribed, or always")


class GripperControl(Strict):
    open_threshold_m: float = Field(default=0.02, description="GripperCommand position "
                                    "below this = close")
    settle_steps: int = Field(default=10, description="control ticks a gripper goal advances")


class TrajectoryControl(Strict):
    goal_tolerance_rad: float = 0.05
    settle_steps: int = 20


class Control(Strict):
    """Graph-side actuation behaviour (the facts below feed machine.yaml)."""
    gripper: GripperControl = Field(default_factory=GripperControl)
    trajectory: TrajectoryControl = Field(default_factory=TrajectoryControl)


class Ports(Strict):
    """ROS 2 names the machine exposes; null = this machine has no such port."""
    twist: str | None = None
    trajectory: str | None = None
    gripper: str | None = None
    wrench: str | None = None
    base_twist: str | None = None
    odom: str | None = None


class RobotFacts(Strict):
    model: str
    description: str = ""


class Frames(Strict):
    world: str = "world"
    base: str | None = "panda_link0"
    hand: str | None = "panda_hand"


class Arm(Strict):
    joints: list[str]
    limits_rad: list[tuple[float, float]]


class Gripper(Strict):
    open_m: float
    closed_m: float
    max_effort: float
    stops_at: list[str] = Field(default_factory=lambda: ["open", "closed"])


class Hand(Strict):
    tcp_offset_m: float = Field(description="hand frame -> fingertip grasp point")


class Planning(Strict):
    moveit: bool = False
    move_action: str = "/move_action"
    ik_service: str = "/compute_ik"
    group: str | None = None
    planning_frame: str | None = None


class Tf(Strict):
    hand: bool = Field(default=False, description="bridge publishes hand frames (MoveIt-less runs)")
    base_body: str = "robot0_base"


class Base(Strict):
    """A mobile base, when the machine has one."""
    type: str | None = None
    body: str | None = None
    frame: str = "base_footprint"
    effective_speed_note: str | None = None
    cmd_vel: str | None = None
    odom: str | None = None


class ArmSpec(Strict):
    """One arm of a multi-arm machine (machine.arms); single-arm profiles
    are normalised into this shape by the loader."""
    label: str = ""
    joints: list[str] = Field(default_factory=list)
    limits_rad: list[tuple[float, float]] = Field(default_factory=list)
    gripper: Gripper | None = None
    ports: Ports = Field(default_factory=Ports)
    hand_body: str = "robot0_right_hand"
    tf_base_body: str = "robot0_base"
    base_frame: str = "panda_link0"
    hand_frame: str | None = "panda_hand"


class Machine(Strict):
    backend: Backend = Field(description="how the robot is provided: kind: sim | real")
    workspace_template: str | None = Field(
        default="workspace", description="workspace tree seeded into the sandbox; null = none")
    controller: str = Field(default="JOINT_POSITION", description="robosuite controller")
    controller_config: str | None = Field(
        default=None, description="controller json: under robots/ (bundled, then "
        "~/.robocli/robots/) or a path")
    controller_kp_scale: float = 10.0
    cameras: Cameras = Field(default_factory=Cameras)
    control: Control = Field(default_factory=Control)
    ports: Ports = Field(default_factory=Ports)
    joint_name_map: dict[str, str] = Field(
        default_factory=dict, description="simulator joint prefix -> published prefix")
    robot: RobotFacts | None = None
    frames: Frames | None = None
    arm: Arm | None = None
    arms: list[ArmSpec] | None = None
    gripper: Gripper | None = None
    hand: Hand | None = None
    planning: Planning | None = None
    tf: Tf | None = None
    base: Base | None = None


class World(Strict):
    """The scene ``robocli up <robot>`` loads when no benchmark is named."""
    benchmark: str
    task_suite: str | None = None
    task_id: int = 0


# ----------------------------------------------------------------- files

class RobotProfile(Strict):
    world: World | None = None
    machine: Machine


class Benchmark(Strict):
    """A benchmarks/<name>.yaml as written: names its robot or carries a machine."""
    task: Task
    protocol: Protocol = Field(default_factory=Protocol)
    agent: AgentConfig = Field(default_factory=AgentConfig)
    robot: str | None = None
    machine: Machine | None = None
    suite_overrides: dict[str, dict[str, Any]] = Field(
        default_factory=dict, description="per-suite deep merge into the config; "
        "null deletes a key")


class ResolvedConfig(Strict):
    """The resolved config every party reads (``<trial>/config.yaml``)."""
    task: Task
    protocol: Protocol = Field(default_factory=Protocol)
    agent: AgentConfig = Field(default_factory=AgentConfig)
    machine: Machine
    suite_overrides: dict[str, dict[str, Any]] = Field(default_factory=dict)


class UserConfig(Strict):
    """A defaults file: the package's configs/config.yaml or ~/.robocli/config.yaml."""
    agent: AgentOverrides = Field(default_factory=AgentOverrides)
    robot: str | None = Field(default=None, description="robot when a command names none")


# ---------------------------------------------------------------- agents

class Credentials(Strict):
    """Where an agent keeps its login and how the sandbox is told about it."""
    dirname: str = Field(description="profile directory name under ~/.robocli/credentials/")
    filename: str = Field(description="the credentials file inside that directory")
    config_env: str = Field(description="environment variable naming the profile directory")
    mount_point: str = Field(description="where the profile is mounted inside the sandbox")


class AgentManifest(Strict):
    """configs/agents/<name>.yaml: the facts about one coding agent, no code.
    The hooks module named by ``hooks`` supplies the behaviour."""
    name: str
    default_model: str
    binary: str | None = Field(default=None, description="executable name inside the sandbox")
    version: str | None = Field(default=None, description="the CLI version the install "
                                "line pins; {version} in install is replaced with it")
    install: str = Field(default="", description="shell that installs the agent in the "
                         "sandbox image")
    whitelist: list[str] = Field(default_factory=list, description="regexes of the hosts "
                                 "the proxy lets the agent reach")
    credentials: Credentials | None = None
    token_env: str | None = Field(default=None, description="environment variable carrying "
                                  "an auth token")
    version_argv: list[str] | None = Field(default=None, description="command printing the "
                                           "agent's version")
    instruction_file: str | None = Field(default=None, description="the file the agent "
                                         "reads instructions from, e.g. AGENTS.md")
    default_options: dict[str, Any] = Field(default_factory=dict)
    hooks: str | None = Field(default=None, description="hooks module name under "
                              "plugins/agents/ (bundled, then ~/.robocli/plugins/agents/)")
