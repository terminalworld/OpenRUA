"""``openrua up <robot> --sim <simulator> [--bench <benchmark>]``: a live
robot with a sandbox terminal on it.

Stays in the foreground: the robot holds its control line to this
process and powers itself off when the process ends. Open a second
terminal for ``openrua agent``, or use ``openrua run`` for the
one-command form (up, agent, down).
"""

from __future__ import annotations

import shutil
import signal
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from openrua import agents
from openrua.cli import state
from openrua.cli.state import DEFAULT_NAME
from openrua.config import apply_suite_overrides, compose, normalize_arms
from openrua.config import paths
from openrua.errors import UnavailableError
from openrua.proxy.up import ensure as ensure_proxy
from openrua.runner.bringup import bring_up, ensure_internal_network, start_episode
from openrua.sandbox.down import down as sandbox_down


@dataclass
class Session:
    """A robot that is up, with the sandbox terminal on it, and the one
    call that powers both off. The fields are what the user should see
    named: which robot, where it runs, which scene, which agent."""

    name: str
    sim: str
    sandbox: str
    robot: str            # the robot as named on the command line (or by the benchmark)
    robot_model: str      # machine.robot.model
    backend: str          # "simulated by robosuite (ROS 2 jazzy)" or "real"
    benchmark: str | None
    suite: str | None
    task_id: int | None
    task: str             # the scene's own task sentence, if any
    agent: str
    model: str
    power_off: Callable[[], None]

    def scene(self) -> str:
        if not self.suite:
            return "(none)"
        where = (f"{self.benchmark} / {self.suite} #{self.task_id}" if self.benchmark
                 else f"{self.suite} (the simulator's own scene)")
        return f'{where}: "{self.task}"' if self.task else where

    def header(self, tag: str, prompt: str | None = None) -> str:
        """What was picked, one line each, before the agent opens."""
        agent = f"{self.agent} ({self.model})"
        if prompt:
            agent += f', opening message: "{prompt}"'
        return (f"[{tag}] robot     {self.robot}: {self.robot_model}, {self.backend}\n"
                f"[{tag}] scene     {self.scene()}\n"
                f"[{tag}] agent     {agent}")

    def banner(self) -> str:
        return f"""
{self.header("up")}
[up] sandbox   {self.sandbox}, on the robot's ROS 2 graph

     openrua agent --name {self.name}            # your coding agent, on the robot
     docker exec -it -u robot -w /workspace {self.sandbox} bash   # or you

     Ctrl-C here powers the robot off."""


def open_session(args) -> Session:
    """Bring the robot and its sandbox up and record them under
    ``args.name``; the returned session's ``power_off`` takes them down."""
    composed = compose(args.robot, args.sim, args.bench, args.home)
    cfg = composed.cfg
    suite = args.task_suite or composed.suite
    task_id = args.task_id if args.task_id is not None else composed.task_id
    apply_suite_overrides(cfg, suite)
    normalize_arms(cfg)
    sim_name, sandbox_name = state.container_names(args.name)
    workdir = Path(args.workspace or paths.workspaces_dir(args.home) / args.name).resolve()
    if workdir.exists():
        shutil.rmtree(workdir)  # a fresh workspace every time (seeding merges)
    workdir.mkdir(parents=True)
    network = ensure_internal_network()
    proxy_url = ensure_proxy(network)
    cfg_agent = cfg.get("agent", {})
    adapter = agents.get(args.agent or cfg_agent.get("name"), args.home,
                         version=cfg_agent.get("version"))
    # The config's model belongs to the config's agent; with --agent
    # naming another one, that model would be sent to the wrong CLI.
    model = (getattr(args, "model", None)
             or (cfg_agent.get("model") if adapter.name == cfg_agent.get("name") else None)
             or adapter.default_model)
    creds_home = Path(cfg.get("agent", {}).get("credentials_dir")
                      or paths.credentials_dir(args.home) / adapter.name).expanduser()
    cfg_dir, creds_file = agents.prepare_profile(creds_home, adapter)
    print(f"[up] sandbox {sandbox_name}; robot {sim_name} (booting; MoveIt takes a minute)",
          flush=True)
    try:
        _, machine = bring_up(
            cfg, workdir, sim_name, sandbox_name, suite, task_id, network, proxy_url,
            adapter.sandbox_mounts(cfg_dir, creds_file), args.ros_domain,
            robot_log=workdir / "robot.log", home=args.home)
    except Exception as e:  # noqa: BLE001
        shutil.rmtree(cfg_dir, ignore_errors=True)
        raise UnavailableError(f"[up] robot failed to come up: {e}",
                               hint=f"read {workdir / 'robot.log'}") from e

    def power_off() -> None:
        print("\n[down] powering off", flush=True)
        sandbox_down(sandbox_name)
        machine.shutdown()
        state.forget(args.name, args.home)
        shutil.rmtree(cfg_dir, ignore_errors=True)

    try:
        if cfg["machine"]["backend"]["kind"] == "sim":
            task = start_episode(machine, args.init_state).get("language", "")
        else:
            task = args.task or ""
    except Exception as e:  # noqa: BLE001
        sandbox_down(sandbox_name)
        machine.shutdown()
        shutil.rmtree(cfg_dir, ignore_errors=True)
        raise UnavailableError(f"[up] robot failed to reset: {e}",
                               hint=f"read {workdir / 'robot.log'}") from e
    state.save(args.name, args.home, sim=sim_name, sandbox=sandbox_name,
               backend=cfg["machine"]["backend"]["kind"],
               network=network, proxy=proxy_url, agent=adapter.name,
               model=model,
               options={**adapter.default_options,
                        **cfg.get("agent", {}).get("options", {})},
               workspace=str(workdir / "workspace"), task=task)
    backend = cfg["machine"]["backend"]
    where = (f"simulated by {composed.simulator} (ROS 2 {backend.get('ros_distro', '?')})"
             if backend["kind"] == "sim" else "real")
    robot = cfg["machine"].get("robot", {})
    return Session(
        name=args.name, sim=sim_name, sandbox=sandbox_name,
        robot=composed.robot, robot_model=robot.get("model", "?"), backend=where,
        benchmark=composed.benchmark,
        suite=suite, task_id=task_id, task=task,
        agent=adapter.name, model=model, power_off=power_off)


def run(args) -> int:
    session = open_session(args)
    print(session.banner(), flush=True)
    try:
        signal.sigwait([signal.SIGINT, signal.SIGTERM])
    finally:
        session.power_off()
    return 0


def add_options(p) -> None:
    """The bring-up options ``up`` and ``run`` share (everything but the
    robot positional)."""
    p.add_argument("--sim", default=None,
                   help="simulator that embodies the robot (openrua simulators); default: "
                   "the benchmark's; a real robot's file needs none")
    p.add_argument("--bench", default=None,
                   help="benchmark whose world to load (openrua benchmarks); default: the "
                   "simulator's native scene")
    p.add_argument("--task-suite", default=None, help="scene suite (default: the profile's)")
    p.add_argument("--task-id", type=int, default=None, help="scene index (default: the profile's)")
    p.add_argument("--init-state", type=int, default=0,
                   help="episode seed / init state (simulated robots)")
    p.add_argument("--task", default=None,
                   help="task sentence to show the agent (real robots; a "
                   "simulated robot's comes from the scene)")
    p.add_argument("--name", default=DEFAULT_NAME,
                   help=f"handle for this robot, for agent/down (default: {DEFAULT_NAME})")
    p.add_argument("--agent", default=None, help="agent to open (default: the config's)")
    p.add_argument("--workspace", default=None,
                   help="working directory (default: <home>/workspaces/<name>)")
    p.add_argument("--ros-domain", type=int, default=0,
                   help="ROS_DOMAIN_ID; concurrent robots need distinct ones")


def add_parser(sub) -> None:
    p = sub.add_parser("up", help="bring a robot up with a sandbox terminal on it",
                       description="Bring a robot up (simulated: boot its container; "
                       "real: join its graph) with a sandbox terminal on it, then stay "
                       "in the foreground; Ctrl-C powers it off. Open a second terminal "
                       "for `openrua agent`, or use `openrua run` to do all of it in one.")
    p.add_argument("robot", nargs="?", default=None,
                   help="robot: a type or your robot's file (openrua robots), by name "
                   "or path; default: --bench's robot, else the user config's default")
    add_options(p)
    p.set_defaults(fn=run)
