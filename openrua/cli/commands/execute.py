"""``openrua exec``: the agent does one task on its own, and the run is recorded.

As a coding agent's own non-interactive command runs it without its
terminal, this runs one trial: the robot comes up (simulated or real),
the agent gets the task and works until it is done or a budget ends, the
robot powers off, and everything lands under ``runs/<label>/<run id>/``
(transcript, result, provenance; ``openrua demo`` renders a simulated
one). A benchmark scene states its task and judges success; a
simulator's own scene judges success too, and takes the sentence given
when it states no task; a real robot's task is the sentence given, and a
person judges it. Like ``openrua bench``, which runs every task and seed of a
benchmark this way, the agent may reach only its model service unless
the limit flags say otherwise.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path

from openrua.cli import updates
from openrua.config import compose, paths
from openrua.errors import UsageError
from openrua.runner.limits import LIMITS, add_limit_options
from openrua.runner.main import run_set


def _source(composed) -> Path:
    """The file the run's config was assembled from, for its record: the
    benchmark's, else the real robot's, else the simulator's."""
    if composed.benchmark:
        return paths.find("benchmarks", composed.benchmark).resolve()
    if composed.cfg["machine"]["backend"]["kind"] == "real":
        return paths.find("robots", composed.robot).resolve()
    return paths.find("simulators", composed.simulator).resolve()


def run(args) -> int:
    home = paths.home(args.home)
    # A sentence for a simulated robot names the simulator's own scene,
    # whatever benchmark is saved as the default.
    bench = args.bench if args.bench is not None else ("" if args.task else None)
    composed = compose(args.robot, args.sim, bench, home, agent=args.agent,
                       model=args.model)
    real = composed.cfg["machine"]["backend"]["kind"] == "real"
    if real and not args.task:
        raise UsageError("a real robot's task is a sentence",
                         hint='openrua exec <robot file> "<what the agent should do>"')
    if composed.benchmark and args.task:
        raise UsageError("a benchmark scene states its own task, and judges that one",
                         hint="choose it with --task-suite and --task-id (openrua "
                         "benchmarks lists them), or drop --bench for the simulator's "
                         "own scene")
    suite = args.task_suite or composed.suite or "task"
    task_id = args.task_id if args.task_id is not None else (composed.task_id or 0)
    label = composed.benchmark or "-".join(
        part for part in (Path(composed.robot).stem, composed.simulator) if part)
    run_id = args.run_id or datetime.now(timezone.utc).strftime("exec-%Y%m%d-%H%M%S")
    trials = argparse.Namespace(
        task=args.task, task_suite=suite, task_ids=[task_id], seeds=[args.seed],
        operator="agent", record=None if args.record is None else tuple(
            c.strip() for c in args.record.split(",") if c.strip()),
        record_every=1, record_size=None, wall_clock_min=args.wall_clock_min,
        runs_root=args.runs_root, run_id=run_id, ros_domain=args.ros_domain,
        credentials_dir=None, account_alias=None, script=None, token_file=None,
        **{key: getattr(args, key) for key in LIMITS})
    updates.notice(composed, home)
    print(f"[exec] {label}: {suite}:{task_id} seed {args.seed}; "
          f"recorded under {Path(args.runs_root or 'runs') / label / run_id}", flush=True)
    return run_set(composed.cfg, _source(composed), trials, home, label)


def add_parser(sub) -> None:
    p = sub.add_parser("exec", help="the agent does one task on its own; the run is recorded",
                       description=__doc__)
    p.add_argument("robot", nargs="?", default=None,
                   help="robot: a type or your robot's file, by name or path; default: "
                   "--bench's robot, else your default")
    p.add_argument("task", nargs="?", default=None,
                   help="the task, as a sentence: a real robot's, or a simulator's own "
                   "scene's when it states none (a benchmark scene states its own)")
    p.add_argument("--sim", default=None, help="simulator for a robot type (openrua simulators)")
    p.add_argument("--bench", default=None, help="benchmark whose scene to load "
                   "(openrua benchmarks); default: the simulator's own scene")
    p.add_argument("--task-suite", default=None, help="scene suite (default: the scene's)")
    p.add_argument("--task-id", type=int, default=None, help="task in the suite (default: the scene's)")
    p.add_argument("--seed", type=int, default=0, help="initial state of a simulated scene")
    p.add_argument("--agent", default=None, help="agent (default: the configured one)")
    p.add_argument("--model", default=None, help="model (default: the agent's)")
    p.add_argument("--run-id", default=None, help="the run's name (default: exec-<UTC time>)")
    p.add_argument("--runs-root", default=None, help="where runs land (default: ./runs)")
    p.add_argument("--wall-clock-min", type=float, default=None,
                   help="active minutes before the run stops (default: the protocol's)")
    p.add_argument("--ros-domain", type=int, default=None,
                   help="ROS_DOMAIN_ID (default: a free one; a real robot: 0)")
    p.add_argument("--record", nargs="?", const="", default=None, metavar="CAMERAS",
                   help="record a simulated robot's cameras for openrua demo (names, or "
                   "none for the robot's default)")
    add_limit_options(p, "default: only the model service, as in a benchmark run")
    p.set_defaults(fn=run)
