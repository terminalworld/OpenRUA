"""``openrua up <robot> --sim <simulator> [--bench <benchmark>]``: a live
robot with a sandbox terminal on it.

Stays in the foreground: the robot holds its control line to this
process and powers itself off when the process ends. Open a second
terminal for ``openrua agent``, or use ``openrua run`` for the
one-command form (up, agent, down).
"""

from __future__ import annotations

import signal
from pathlib import Path

from openrua.runner.live import LiveRobot, RobotRequest, open_robot
from openrua.runner.live_state import DEFAULT_NAME


class Session(LiveRobot):
    """CLI presentation of the shared live robot handle."""

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

     Ctrl-C here powers the robot off and keeps the workspace."""


def open_session(args) -> Session:
    """Translate command-line options into the shared resource request."""
    live = open_robot(RobotRequest(
        home=args.home, name=args.name, robot=args.robot, simulator=args.sim,
        benchmark=args.bench, agent=args.agent, model=getattr(args, "model", None),
        task_suite=args.task_suite, task_id=args.task_id, task=args.task,
        init_state=args.init_state,
        workspace=Path(args.workspace) if args.workspace else None,
        ros_domain=args.ros_domain))
    return Session(**vars(live))


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
                   help="new or empty working directory, retained after shutdown (default: <home>/sandboxes/<name>/workspace)")
    p.add_argument("--ros-domain", type=int, default=None,
                   help="ROS_DOMAIN_ID (default: the lowest one no running robot "
                        "uses, so concurrent robots never share a graph; a real "
                        "robot: its own domain, 0)")


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
