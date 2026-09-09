"""The configuration schema: what a robot, a simulator, a benchmark, an
agent manifest and the resolved per-trial config may say, with defaults
and a one-line description on every field.

Three kinds of file feed one resolved config, and they depend in one
direction only, benchmark -> simulator -> robot:

- a robot (``robots/<type>.yaml``): what is true of this robot wherever
  it runs (joints, limits, frames, gripper, ports, planner). A real
  robot is an instance: the same file kind with a ``machine:`` section
  carrying its ``real`` backend (and optionally ``type:`` naming the
  robot type it is an instance of);
- a simulator (``simulators/<engine>.yaml``): the engine, its install
  (venv, ROS distro), the native scene ``openrua run`` loads when no
  benchmark is named, and ``robots:``, how it drives each robot type it
  embodies (controller, gains, joint-name map);
- a benchmark (``benchmarks/<name>.yaml``): ``task:``, ``protocol:``,
  ``agent:``, which ``robot:`` and ``simulator:`` it runs on, and what
  it brings along: ``install:`` (its own venv and distro over the
  simulator's) and ``scenes:`` (scene cameras, and robot embodiments
  the benchmark's own assets add or adjust);
- the user's ``~/.openrua/config.yaml``: defaults for the ``agent:``
  section and a default robot, applied under whatever the benchmark
  config says.

``loader.assemble`` folds robot type, embodiments and install into the
one ``machine:`` dict every downstream unit reads (``Machine`` below).

Layering, lowest first: the defaults declared here, the package's
``configs/config.yaml``, the user file, the benchmark config,
command-line flags. Unknown keys are errors at every
level: a misspelled key must fail the load, never silently do nothing.
Defaults are the values the reported runs use;
protocol switches that only matter when running at scale on subscription
accounts (``resume_on_quota_wall``) default off.

The models validate; consumers keep reading plain dicts
(``loader.dump()``), so the container side never imports pydantic.
Leaf module: imports nothing from openrua.
"""

from __future__ import annotations

from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


# ------------------------------------------------------------------ task

class Task(Strict):
    benchmark: str = Field(description="the benchmark's name as records carry it: "
                           "libero_pro | capbench | robocasa365 | the engine's name "
                           "for a native scene")
    loader: str | None = Field(default=None, description="the loader the bridge imports, "
                               "resolved by openrua from the benchmark's (or native "
                               "scene's) entry_point: a module path or an absolute "
                               "file path; not written by hand")
    suites: list[str] = Field(description="task suites this benchmark runs; "
                              "openrua bench picks one with --task-suite")
    init_states: str | None = Field(
        default=None, description="how episodes start (documentation of the "
        "loader's behaviour): benchmark-files | seeded-reset")
    split: str = Field(default="target", description="robocasa: object/layout split "
                       "(robocasa365: target | pretrain | all; robocasa: eval | train | all)")
    dataset_root: str | None = Field(default=None, description="a benchmark dataset kept "
                                     "outside its checkout (robocerebra: the "
                                     "RoboCerebra_Bench directory); default: next to the "
                                     "checkout")
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
    clock: Clock = Field(default_factory=Clock, description="how simulated time advances")
    horizon: int = Field(default=10**9, description="simulator step cap (libero)")
    base_cmd_policy: Literal["queue", "latest"] = Field(
        default="queue", description="mobile base: queue every twist (one message = "
        "one tick) or let a new one replace the pending setpoint")
    stall_probe_timeout_s: float = Field(
        default=600, description="seconds a simulator job may run before the trial is "
        "declared wedged")


# ----------------------------------------------------------------- agent

class AgentConfig(Strict):
    name: str | None = Field(default=None, description="agent name (openrua agents lists "
                             "them); the package default lives in configs/config.yaml")
    model: str | None = Field(default=None, description="model id; default: the adapter's")
    version: str | None = Field(default=None, description="pin the agent CLI version: the "
                                "sandbox image must carry it and preflight checks it; "
                                "default: whatever the image has")
    credentials_dir: str | None = Field(
        default=None, description="login profile directory; default: "
        "~/.openrua/credentials/<agent name>")
    options: dict[str, Any] = Field(
        default_factory=dict, description="adapter-specific knobs passed through as "
        "given, over the adapter's default_options")


class AgentOverrides(Strict):
    """A defaults file's agent section: same keys, no defaults, so only
    what the file wrote is layered in."""
    name: str | None = Field(default=None, description="see AgentConfig")
    model: str | None = Field(default=None, description="see AgentConfig")
    version: str | None = Field(default=None, description="see AgentConfig")
    credentials_dir: str | None = Field(default=None, description="see AgentConfig")
    options: dict[str, Any] | None = Field(default=None, description="merged key by key "
                                           "with the layers above")


# --------------------------------------------------------------- machine

class Simulator(Strict):
    venv: str = Field(description="simulator venv: absolute, ~, or relative to "
                      "~/.openrua/simulators/")
    container: str | None = Field(default=None, description="which sim image family "
                                  "(sim-jazzy | sim-humble); documentation")


Distro = Literal["jazzy", "humble"]


def sim_image(distro: str) -> str:
    """The simulated robot image for a ROS 2 distro; openrua build robot
    tags it so."""
    return f"openrua-sim-{distro}"


def sandbox_image(distro: str) -> str:
    """The agent terminal image for a ROS 2 distro; openrua build sandbox
    tags it so. The sandbox runs the robot's distro: cross-distro
    message definitions break service responses."""
    return f"openrua-sandbox-{distro}"


class SimBackend(Strict):
    """A simulated robot: a container running the bridge over a simulator venv."""
    kind: Literal["sim"] = Field(description="a simulated robot")
    ros_distro: Distro = Field(default="jazzy", description="the ROS 2 distro the robot "
                               "runs; the robot and sandbox images are named after it")
    image: str | None = Field(default=None, description="simulated robot image; default: "
                              "openrua-sim-<ros_distro>")
    sandbox_image: str | None = Field(default=None, description="agent terminal image; "
                                      "default: openrua-sandbox-<ros_distro>")
    gpus: bool = Field(default=False, description="render on the GPU (needs nvidia toolkit)")
    resources: dict[str, Any] | None = Field(
        default=None, description="render_threads: int | off | auto")
    simulator: Simulator = Field(description="the simulator venv the bridge runs in")

    @model_validator(mode="after")
    def _derive_images(self):
        self.image = self.image or sim_image(self.ros_distro)
        self.sandbox_image = self.sandbox_image or sandbox_image(self.ros_distro)
        return self


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
    kind: Literal["real"] = Field(description="a real robot")
    ros_distro: Distro = Field(default="jazzy", description="the ROS 2 distro the robot "
                               "runs; the sandbox image is named after it")
    launch: str | None = Field(default=None, description="command that brings the "
                               "robot's ROS 2 graph up; null = already running")
    image: str | None = Field(default=None, description="docker image the launch "
                              "command runs in, on the host network (a vendor driver "
                              "pinned to its own ROS release); null = run it on the host")
    discovery: Discovery = Field(description="how the sandbox reaches the graph")

    sandbox_image: str | None = Field(default=None, description="agent terminal image; "
                                      "default: openrua-sandbox-<ros_distro>")

    @model_validator(mode="after")
    def _derive(self):
        if self.image and not self.launch:
            raise ValueError("backend.image needs a launch command to run in it")
        self.sandbox_image = self.sandbox_image or sandbox_image(self.ros_distro)
        return self


Backend = Annotated[SimBackend | RealBackend, Field(discriminator="kind")]


class Cameras(Strict):
    resolution: tuple[int, int] = Field(default=(640, 480), description="width, height "
                                        "in pixels for every camera")
    # yaml key is ``list``; the attribute cannot be, so it carries an alias
    names: list[str] | None = Field(default=None, alias="list",
                                    description="camera names; null = every "
                                    "camera the scene defines")
    rate_hz: float = Field(default=2.0, description="wall-clock publish rate")
    render_mode: Literal["on_demand", "always"] = Field(
        default="on_demand", description="render a camera only while subscribed, or always")
    record: list[str] | None = Field(
        default=None, description="the cameras `openrua bench --record` captures when none "
        "are named: the first is a demo's main view, the second its inset")


class GripperControl(Strict):
    open_threshold_m: float = Field(default=0.02, description="GripperCommand position "
                                    "below this = close")
    settle_steps: int = Field(default=10, description="control ticks a gripper goal advances")


class TrajectoryControl(Strict):
    goal_tolerance_rad: float = Field(default=0.05, description="a FollowJointTrajectory "
                                      "goal succeeds when every joint is within this of "
                                      "its last point, radians")
    settle_steps: int = Field(default=20, description="control ticks the arm keeps "
                              "tracking after the last point before the result is judged")


class Control(Strict):
    """Graph-side actuation behaviour (the facts below feed machine.yaml)."""
    gripper: GripperControl = Field(default_factory=GripperControl,
                                    description="how a GripperCommand goal is executed")
    trajectory: TrajectoryControl = Field(default_factory=TrajectoryControl,
                                          description="how a FollowJointTrajectory goal is "
                                          "judged")


class Ports(Strict):
    """ROS 2 names the machine exposes; null = this machine has no such port."""
    twist: str | None = Field(default=None, description="TwistStamped topic for "
                              "Cartesian servoing of the hand")
    trajectory: str | None = Field(default=None, description="FollowJointTrajectory action")
    gripper: str | None = Field(default=None, description="GripperCommand action")
    wrench: str | None = Field(default=None, description="WrenchStamped topic of the "
                               "wrist force-torque sensor")
    base_twist: str | None = Field(default=None, description="Twist topic driving a "
                                   "mobile base")
    odom: str | None = Field(default=None, description="Odometry topic of a mobile base")


class RobotFacts(Strict):
    """What the agent is told the robot is."""
    model: str = Field(description="the model as a person would name it")
    description: str = Field(default="", description="one line: arm, gripper, base")


class Frames(Strict):
    """The TF frames the workspace docs refer to."""
    world: str = Field(default="world", description="the fixed frame the robot plans in")
    base: str | None = Field(default="panda_link0", description="the arm's root link")
    hand: str | None = Field(default="panda_hand", description="the hand frame that "
                             "twist commands and IK targets refer to")


class Arm(Strict):
    joints: list[str] = Field(description="joint names in the order the trajectory "
                              "port expects them")
    limits_rad: list[tuple[float, float]] = Field(description="travel per joint as "
                                                  "[min, max], radians")


class Gripper(Strict):
    open_m: float = Field(description="fingers fully open: position of one finger, "
                          "metres (the gap is twice this)")
    closed_m: float = Field(description="fingers fully closed: position of one finger, "
                            "metres")
    max_effort: float = Field(description="force ceiling a GripperCommand goal may ask "
                              "for, newtons")
    stops_at: list[str] = Field(default_factory=lambda: ["open", "closed"],
                                description="the only positions the gripper comes to "
                                "rest at; a commanded width is read as open or closed")


class Hand(Strict):
    tcp_offset_m: float = Field(description="hand frame -> fingertip grasp point")


class Planning(Strict):
    """The MoveIt planner, when the graph runs one."""
    moveit: bool = Field(default=False, description="whether a MoveIt planner runs on "
                         "this robot")
    move_action: str = Field(default="/move_action", description="MoveIt's plan-and-execute "
                             "action")
    ik_service: str = Field(default="/compute_ik", description="inverse-kinematics service")
    group: str | None = Field(default=None, description="the MoveIt planning group "
                              "covering the arm")
    planning_frame: str | None = Field(default=None, description="the frame MoveIt "
                                       "plans in")


class Tf(Strict):
    hand: bool = Field(default=False, description="bridge publishes hand frames (MoveIt-less runs)")
    base_body: str = Field(default="robot0_base", description="simulator body the base "
                           "frame is read from")


class Base(Strict):
    """A mobile base, when the machine has one."""
    type: str | None = Field(default=None, description="the base as a person would name "
                             "it (omnidirectional, differential)")
    body: str | None = Field(default=None, description="simulator body odometry is read "
                             "from")
    frame: str = Field(default="base_footprint", description="the base's TF frame")
    effective_speed_note: str | None = Field(default=None, description="one line for the "
                                             "agent on how commanded speed maps to motion")
    cmd_vel: str | None = Field(default=None, description="Twist topic driving the base")
    odom: str | None = Field(default=None, description="Odometry topic")


class ArmSpec(Strict):
    """One arm of a multi-arm machine (machine.arms); single-arm profiles
    are normalised into this shape by the loader."""
    label: str = Field(default="", description="how the docs name this arm (left, right); "
                       "empty on a single arm")
    joints: list[str] = Field(default_factory=list, description="joint names in the order "
                              "the trajectory port expects them")
    limits_rad: list[tuple[float, float]] = Field(default_factory=list,
                                                  description="travel per joint as "
                                                  "[min, max], radians")
    gripper: Gripper | None = Field(default=None, description="this arm's gripper; null = "
                                    "a tool with no gripper")
    ports: Ports = Field(default_factory=Ports, description="this arm's ports")
    hand_body: str = Field(default="robot0_right_hand", description="simulator body the "
                           "hand frame is read from")
    tf_base_body: str = Field(default="robot0_base", description="simulator body the "
                              "base frame is read from")
    base_frame: str = Field(default="panda_link0", description="the arm's root link")
    hand_frame: str | None = Field(default="panda_hand", description="the hand frame")


class Machine(Strict):
    backend: Backend = Field(description="how the robot is provided: kind: sim | real")
    workspace_template: str | None = Field(
        default="workspace", description="workspace tree seeded into the sandbox; null = none")
    engine_model: str | None = Field(default=None, description="the robot's name inside "
                                     "the simulator (robosuite: Panda, PandaOmron)")
    controller: str = Field(default="JOINT_POSITION", description="robosuite controller")
    controller_config: str | None = Field(
        default=None, description="controller json: a bundled name under "
        "robots/controllers/ or a path relative to the file naming it")
    controller_kp_scale: float = Field(default=10.0, description="multiplier on the "
                                       "simulator's joint position gains")
    cameras: Cameras = Field(default_factory=Cameras, description="the cameras the graph "
                             "publishes")
    control: Control = Field(default_factory=Control, description="how goals are executed "
                             "and judged")
    ports: Ports = Field(default_factory=Ports, description="the ROS 2 names the robot "
                         "serves (single arm)")
    joint_name_map: dict[str, str] = Field(
        default_factory=dict, description="simulator joint prefix -> published prefix")
    robot: RobotFacts | None = Field(default=None, description="model and description")
    frames: Frames | None = Field(default=None, description="world, base and hand frames")
    arm: Arm | None = Field(default=None, description="the arm (single-arm profiles)")
    arms: list[ArmSpec] | None = Field(default=None, description="the arms (multi-arm "
                                       "profiles); replaces arm, gripper and ports")
    gripper: Gripper | None = Field(default=None, description="the gripper; null = none")
    hand: Hand | None = Field(default=None, description="hand geometry")
    planning: Planning | None = Field(default=None, description="the MoveIt planner")
    tf: Tf | None = Field(default=None, description="what the bridge publishes on TF")
    base: Base | None = Field(default=None, description="the mobile base; null = fixed")


class RobotType(Strict):
    """A robots/<type>.yaml: what is true of this robot wherever it runs.
    No cameras, no backend, no scene: those come from the simulator or
    the benchmark that embodies it."""
    robot: RobotFacts = Field(description="model and description")
    frames: Frames | None = Field(default=None, description="world, base and hand frames")
    arm: Arm | None = Field(default=None, description="the arm (single-arm robots)")
    arms: list[ArmSpec] | None = Field(default=None, description="the arms (multi-arm "
                                       "robots); replaces arm, gripper and ports")
    gripper: Gripper | None = Field(default=None, description="the gripper; null = none")
    hand: Hand | None = Field(default=None, description="hand geometry")
    ports: Ports = Field(default_factory=Ports, description="the ROS 2 names the robot "
                         "serves (single arm)")
    planning: Planning | None = Field(default=None, description="the MoveIt planner")
    base: Base | None = Field(default=None, description="the mobile base; null = fixed")
    workspace_template: str | None = Field(
        default="workspace", description="workspace tree seeded into the sandbox; null = none")


class Embodiment(Strict):
    """How an engine drives one robot type: the keys that depend on the
    simulator, merged over the robot type. Written under a simulator's
    ``robots:`` (the engine's own robots) or a benchmark's
    ``scenes.robots:`` (robots the benchmark's assets add or adjust).
    Only the keys written are merged in."""
    engine_model: str | None = Field(default=None, description="the robot's name inside "
                                     "the engine (robosuite: Panda, PandaOmron)")
    controller: str | None = Field(default=None, description="robosuite controller type")
    controller_config: str | None = Field(
        default=None, description="controller json: a bundled name under "
        "robots/controllers/ or a path relative to the file naming it")
    controller_kp_scale: float | None = Field(default=None, description="multiplier on the "
                                              "engine's joint position gains")
    joint_name_map: dict[str, str] | None = Field(
        default=None, description="engine joint prefix -> published prefix")
    control: Control | None = Field(default=None, description="how goals are executed "
                                    "and judged")
    tf: Tf | None = Field(default=None, description="what the bridge publishes on TF")
    base: Base | None = Field(default=None, description="mobile-base keys the engine "
                              "decides (body, effective speed)")
    cameras: Cameras | None = Field(default=None, description="robot-mounted cameras this "
                                    "embodiment adds")


class Checkout(Strict):
    """One repository ``openrua install`` clones at a pinned commit."""
    path: str = Field(description="where it lands, relative to ~/.openrua/simulators/")
    repo: str = Field(description="git URL")
    commit: str = Field(description="the commit checked out (a full hash)")
    submodules: list[str] = Field(default_factory=list, description="submodule paths to "
                                  "initialise, relative to the checkout")
    patch: str | None = Field(default=None, description="a patch applied to the checkout, "
                              "relative to the file naming it; already-applied is fine")


class Install(Strict):
    """A simulator install: the venv the bridge runs in, the ROS distro
    that goes with its Python, and the recipe ``openrua install`` renders
    to a script and runs to build it."""
    venv: str = Field(description="simulator venv: absolute, ~, or relative to "
                      "~/.openrua/simulators/")
    ros_distro: Distro = Field(default="jazzy", description="the ROS 2 distro the robot "
                               "runs; the robot and sandbox images are named after it")
    python: str | None = Field(default=None, description="the venv's Python (3.12 goes "
                               "with Jazzy, 3.10 with Humble); required to install")
    checkouts: list[Checkout] = Field(default_factory=list, description="repositories "
                                      "cloned at pinned commits")
    requirements: str | None = Field(default=None, description="a pip requirements lock "
                                     "installed into the venv, relative to the file "
                                     "naming it")
    editable: list[str] = Field(default_factory=list, description="checkouts installed "
                                "editable with --no-deps (the lock has their "
                                "dependencies), relative to ~/.openrua/simulators/")
    shell: str | None = Field(default=None, description="shell run last, in the venv, "
                              "for what the fields above cannot say (asset downloads); "
                              "{root} = ~/.openrua/simulators, {venv} = the venv, "
                              "{here} = the directory of the file naming it")
    container: str | None = Field(default=None, description="which sim image family "
                                  "(sim-jazzy | sim-humble); documentation")
    image: str | None = Field(default=None, description="simulated robot image; default: "
                              "openrua-sim-<ros_distro>")
    sandbox_image: str | None = Field(default=None, description="agent terminal image; "
                                      "default: openrua-sandbox-<ros_distro>")
    gpus: bool = Field(default=False, description="render on the GPU (needs nvidia toolkit)")
    resources: dict[str, Any] | None = Field(
        default=None, description="render_threads: int | off | auto")


class InstallOverrides(Strict):
    """A benchmark's install section: same keys as Install, none required;
    only what is written replaces the simulator's, key by key. A
    benchmark with a venv of its own writes the whole recipe."""
    venv: str | None = Field(default=None, description="see Install")
    ros_distro: Distro | None = Field(default=None, description="see Install")
    python: str | None = Field(default=None, description="see Install")
    checkouts: list[Checkout] | None = Field(default=None, description="see Install")
    requirements: str | None = Field(default=None, description="see Install")
    editable: list[str] | None = Field(default=None, description="see Install")
    shell: str | None = Field(default=None, description="see Install")
    container: str | None = Field(default=None, description="see Install")
    image: str | None = Field(default=None, description="see Install")
    sandbox_image: str | None = Field(default=None, description="see Install")
    gpus: bool | None = Field(default=None, description="see Install")
    resources: dict[str, Any] | None = Field(default=None, description="see Install")


class NativeScene(Strict):
    """What ``openrua run <robot> --sim <engine>`` loads with no benchmark."""
    entry_point: str = Field(description="the bridge loader for the engine's own scenes: "
                             "a bundled name (robosuite) or a path to a module of your "
                             "own, relative to this file")
    scene: str = Field(description="the engine's own scene / env name (robosuite: Lift)")
    cameras: Cameras = Field(default_factory=Cameras, description="the scene's cameras")


class SimulatorProfile(Strict):
    """A simulators/<engine>.yaml: the engine, its install, its native
    scene, and how it drives each robot type it embodies."""
    engine: str = Field(description="the engine, as a person would name it "
                        "(robosuite 1.5 on MuJoCo)")
    install: Install = Field(description="the venv and distro the bridge runs with")
    native: NativeScene | None = Field(default=None, description="scene loaded with no "
                                       "benchmark; null = a benchmark is required")
    robots: dict[str, Embodiment] = Field(default_factory=dict, description="robot type "
                                          "name -> how this engine drives it")


class Scenes(Strict):
    """What a benchmark brings into the simulator's world."""
    default_suite: str | None = Field(default=None, description="the suite openrua run / up "
                                      "load when none is named; default: the first in "
                                      "task.suites")
    cameras: Cameras | None = Field(default=None, description="the scene cameras (over "
                                    "the simulator's native ones)")
    robots: dict[str, Embodiment] = Field(default_factory=dict, description="robot type "
                                          "name -> embodiment this benchmark's assets add "
                                          "or adjust (merged over the simulator's)")


class RobotInstance(Strict):
    """A robots/<name>.yaml describing one particular robot, usually a real
    one: its ``machine:`` (backend and facts), optionally over a robot
    type's facts (``type:``)."""
    type: str | None = Field(default=None, description="robot type this is an instance "
                             "of; its facts come first, machine: writes over them")
    machine: Machine = Field(description="backend (kind: real | sim) and facts")


class Benchmark(Strict):
    """A benchmarks/<name>.yaml as written."""
    entry_point: str = Field(description="the bridge loader building, resetting and "
                             "scoring this benchmark's scenes: a bundled name (libero, "
                             "capbench, robocasa) or a path to a module of your own, "
                             "relative to this file; the module exposes LOADER")
    task: Task = Field(description="what is run")
    protocol: Protocol = Field(default_factory=Protocol, description="budgets and clock")
    agent: AgentConfig = Field(default_factory=AgentConfig, description="which agent, "
                               "over the defaults files")
    robot: str | None = Field(default=None, description="robot type (or instance) name or "
                              "path; --robot overrides it")
    simulator: str | None = Field(default=None, description="simulator name or path; "
                                  "--sim overrides it; null with a real-robot instance")
    install: InstallOverrides | None = Field(default=None, description="this benchmark's "
                                             "own venv and distro, over the simulator's")
    scenes: Scenes = Field(default_factory=Scenes, description="what the benchmark brings "
                           "into the world")
    machine: Machine | None = Field(default=None, description="a robot written inline "
                                    "instead of assembled")
    suite_overrides: dict[str, dict[str, Any]] = Field(
        default_factory=dict, description="per-suite deep merge into the config; "
        "null deletes a key")


class SandboxConfig(Strict):
    """How this machine starts the sandbox container. A machine fact, so
    it lives in the defaults files, never in a benchmark config."""
    run_args: list[str] = Field(
        default_factory=list,
        description="flags appended to the sandbox's docker run, verbatim "
        "(rootless podman needs --userns=keep-id; see docs/podman.md)")


class ResolvedConfig(Strict):
    """The resolved config every party reads (``<trial>/config.yaml``)."""
    task: Task = Field(description="what is run")
    protocol: Protocol = Field(default_factory=Protocol, description="budgets and clock")
    agent: AgentConfig = Field(default_factory=AgentConfig, description="the agent, "
                               "layered over the defaults files")
    machine: Machine = Field(description="the robot")
    sandbox: SandboxConfig = Field(default_factory=SandboxConfig, description="from the "
                                   "defaults files")
    suite_overrides: dict[str, dict[str, Any]] = Field(
        default_factory=dict, description="as written; applied for the suite that runs")


class UserConfig(Strict):
    """A defaults file: the package's configs/config.yaml or ~/.openrua/config.yaml."""
    agent: AgentOverrides = Field(default_factory=AgentOverrides, description="agent "
                                  "defaults; only the keys written are layered in")
    robot: str | None = Field(default=None, description="robot when a command names none")
    simulator: str | None = Field(default=None, description="simulator when a command "
                                  "names none (and the robot is a type)")
    benchmark: str | None = Field(default=None, description="benchmark whose world "
                                  "openrua run / up load when --bench is not given; null = "
                                  "the simulator's native scene")
    sandbox: SandboxConfig = Field(default_factory=SandboxConfig, description="how this "
                                   "machine starts the sandbox")


# ---------------------------------------------------------------- agents

class Credentials(Strict):
    """Where an agent keeps its login and how the sandbox is told about it."""
    dirname: str = Field(description="profile directory name under ~/.openrua/credentials/")
    filename: str = Field(description="the credentials file inside that directory")
    config_env: str = Field(description="environment variable naming the profile directory")
    mount_point: str = Field(description="where the profile is mounted inside the sandbox")


class AgentManifest(Strict):
    """configs/agents/<name>.yaml: the facts about one coding agent, no code.
    The module named by ``entry_point`` supplies the behaviour."""
    name: str = Field(description="the name configs use under agent.name")
    default_model: str = Field(description="model when the config names none")
    binary: str | None = Field(default=None, description="executable name inside the sandbox")
    version: str | None = Field(default=None, description="a CLI version this manifest "
                                "pins; usually null: pins come from name@version at build "
                                "time or agent.version in a config")
    install: str = Field(default="", description="shell that installs the agent in the "
                         "sandbox image")
    whitelist: list[str] = Field(default_factory=list, description="regexes of the hosts "
                                 "the proxy lets the agent reach")
    credentials: Credentials | None = Field(default=None, description="profile-directory "
                                            "login; null = token only")
    token_env: str | None = Field(default=None, description="environment variable carrying "
                                  "an auth token")
    version_argv: list[str] | None = Field(default=None, description="command printing the "
                                           "agent's version")
    instruction_file: str | None = Field(default=None, description="the file the agent "
                                         "reads instructions from, e.g. AGENTS.md")
    default_options: dict[str, Any] = Field(default_factory=dict, description="knobs a "
                                            "config may override under agent.options")
    entry_point: str | None = Field(default=None, description="the module behind this "
                                    "manifest, exposing HOOKS (an Agent subclass): a "
                                    "bundled name under plugins/agents/ or a path to a "
                                    "module of your own, relative to this file")
