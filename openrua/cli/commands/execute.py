"""``openrua exec``: the agent does one task on its own, and the run is recorded.

As a coding agent's own non-interactive command runs it without its
terminal, this works in the current project: the robot comes up (simulated or real),
the agent gets the task and works until it is done or a budget ends, the
robot powers off, and records land under ``.openrua/runs/<run id>/``
(transcript, result, provenance; ``openrua demo`` renders a simulated
one; a real robot's cameras are recorded too, unless --no-record).
While the agent works, what it says and each command it runs show
as they happen; Ctrl-C stops it and powers the robot off. A benchmark
scene supplies a default task and judges success; an explicit sentence is
the agent's task, without changing the scene's success predicate. A real
robot's task is the sentence given, and at the end you are asked whether
it was done (your answer is the result; the agent's last words are kept
beside it). Like ``openrua bench``, which runs every task and seed of a
benchmark this way, the agent may reach only its model service unless
the limit flags say otherwise.
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

from openrua.cli import agent_settings, updates, workspace
from openrua.cli.progress import Progress
from openrua.config import compose, load_robot, paths, projects
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


def ask_person(words: str) -> bool | None:
    """Whether the robot did the task, as the person beside it says; None
    for no answer (Enter, Ctrl-C, no terminal)."""
    try:
        answer = input("[exec] Did the robot do the task? [y]es / [n]o / Enter to skip: ")
    except (EOFError, KeyboardInterrupt):
        print()
        return None
    answer = answer.strip().lower()
    return True if answer in ("y", "yes") else False if answer in ("n", "no") else None


def recording(args, real: bool) -> tuple[str, ...] | None:
    """The cameras to record (empty: all), or None. A real robot's run
    cannot be replayed, so it is recorded unless --no-record says not to;
    a simulated one can, so it is recorded only when --record asks."""
    if args.no_record:
        return None
    if args.record is not None:
        return tuple(c.strip() for c in args.record.split(",") if c.strip())
    return () if real else None


def _task(args) -> tuple[str | None, str | None]:
    """Accept a task alone as well as the original robot/task spelling."""
    word = args.task_or_robot
    if args.task is not None:
        if args.robot is not None:
            raise UsageError("name the robot either positionally or with --robot")
        return word, args.task
    if word is not None and args.robot is None:
        robot_file = Path(word).suffix in {'.yaml', '.yml'} and (
            not any(c.isspace() for c in word) or word.startswith(('./', '/', '~/')))
        if word in {entry.name for entry in paths.available('robots')} or robot_file:
            return word, None
    return args.robot, word


def run(args) -> int:
    home = paths.home(args.home)
    from openrua.cli.skill_scheduler import autostart
    autostart(home)
    robot, task = _task(args)
    root = workspace.choose(args.workspace or Path.cwd(), home, accept=args.trust_workspace)
    if root is None:
        return 0
    saved = projects.selection(root)
    sim = args.sim if args.sim is not None else saved.get('sim')
    instance = robot is not None and load_robot(robot, home)[0] == 'instance'
    if instance and args.sim is None:
        sim = None
    # A project keeps its scene when given a new task. Without a project
    # selection, a sentence selects the simulator's own scene.
    bench = args.bench if args.bench is not None else saved.get('bench', "" if task else None)
    if instance and args.bench is None:
        bench = ''
    composed = compose(robot if robot is not None else saved.get('robot'),
                       sim, bench, home,
                       agent=args.agent if args.agent is not None else saved.get('agent'),
                       model=args.model if args.model is not None else agent_settings.project_model(saved, args.agent))
    real = composed.cfg["machine"]["backend"]["kind"] == "real"
    if real and not task:
        raise UsageError("a real robot's task is a sentence",
                         hint='openrua exec <robot file> "<what the agent should do>"')
    suite = args.task_suite or composed.suite or "task"
    task_id = args.task_id if args.task_id is not None else (composed.task_id or 0)
    label = composed.benchmark or "-".join(
        part for part in (Path(composed.robot).stem, composed.simulator) if part)
    run_id = args.run_id or datetime.now(timezone.utc).strftime("exec-%Y%m%d-%H%M%S")
    projects.save_selection(root, dict(robot=composed.robot, sim=composed.simulator or '',
                                      bench=composed.benchmark or '',
                                      agent=args.agent if args.agent is not None else saved.get('agent') or composed.cfg['agent']['name'],
                                      model=composed.cfg['agent'].get('model', '')))
    trials = argparse.Namespace(
        task=task, task_suite=suite, task_ids=[task_id], seeds=[args.seed], project=root,
        operator="agent", record=recording(args, real),
        record_every=1, record_size=None, wall_clock_min=args.wall_clock_min,
        runs_root=args.runs_root, run_id=run_id, ros_domain=args.ros_domain,
        credentials_dir=None, account_alias=None, script=None, token_file=None,
        **{key: getattr(args, key) for key in LIMITS})
    updates.notice(composed, home)
    print(f"[exec] {label}: {suite}:{task_id} seed {args.seed}; project {root}", flush=True)
    trials.progress = Progress()
    # Nothing scores a real robot: the person beside it does, when there is
    # a terminal to ask on. The running line goes first.
    trials.judge = (lambda words: (trials.progress.close(), ask_person(words))[1]) \
        if real and sys.stdin.isatty() else None
    try:
        return run_set(composed.cfg, _source(composed), trials, home, label)
    finally:
        trials.progress.close()


def add_parser(sub) -> None:
    p = sub.add_parser("exec", help="the agent does one task on its own; the run is recorded",
                       description=__doc__)
    p.add_argument("task_or_robot", nargs="?", default=None,
                   help="task sentence, or a robot name/file followed by its task")
    p.add_argument("task", nargs="?", default=None,
                   help="the task, as a sentence: a real robot's, or a simulator's own "
                   "scene's (default: the scene's task)")
    p.add_argument('--robot', default=argparse.SUPPRESS,
                   help="robot name or file (default: the project's, else your default)")
    p.add_argument('-C', '--cd', '--workspace', dest='workspace', default=argparse.SUPPRESS,
                   help='persistent project directory (default: the current directory)')
    p.add_argument('--trust-workspace', action='store_true', default=argparse.SUPPRESS,
                   help='explicitly trust the project without an interactive question')
    p.add_argument("--sim", default=None, help="simulator for a robot type (default: the project's or configured one)")
    p.add_argument("--bench", default=None, help="benchmark whose scene to load "
                   "(openrua benchmarks); default: the project's; an empty value selects the native scene")
    p.add_argument("--task-suite", default=None, help="scene suite (default: the scene's)")
    p.add_argument("--task-id", type=int, default=None, help="task in the suite (default: the scene's)")
    p.add_argument("--seed", type=int, default=0, help="initial state of a simulated scene")
    p.add_argument("--agent", default=None, help="agent (default: the project's or configured one)")
    p.add_argument("--model", default=None, help="model (default: the project's or agent's)")
    p.add_argument("--run-id", default=None, help="the run's name (default: exec-<UTC time>)")
    p.add_argument("--runs-root", default=None,
                   help="record runs under DIR/<label>/ (default: the project's .openrua/runs/)")
    p.add_argument("--wall-clock-min", type=float, default=None,
                   help="active minutes before the run stops (default: the protocol's)")
    p.add_argument("--ros-domain", type=int, default=None,
                   help="ROS_DOMAIN_ID (default: a free one; a real robot: 0)")
    p.add_argument("--record", nargs="?", const="", default=None, metavar="CAMERAS",
                   help="record the cameras for openrua demo (names, or none for all); "
                   "a real robot's run is recorded anyway, two frames a second")
    p.add_argument("--no-record", action="store_true",
                   help="do not record a real robot's cameras")
    add_limit_options(p, "default: only the model service, as in a benchmark run")
    p.set_defaults(fn=run)
