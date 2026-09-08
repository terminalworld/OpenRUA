"""``openrua bench``: a task set on a robot, one trial per (task, seed).

``openrua bench --config <benchmark> --run-id <label> --task-suite S
--task-ids 0,1 --seeds 0,1,2 --operator agent``
"""

from __future__ import annotations

import argparse
from pathlib import Path

from openrua import agents, proxy
from openrua.config import (apply_suite_overrides, load_config, normalize_arms,
                            resolve_wall_clock_min)
from openrua.config import paths
from openrua.errors import UsageError
from openrua.runner import lock
from openrua.runner import record
from openrua.runner.bringup import simulator_venv
from openrua.runner.operators import OPERATORS
from openrua.runner.trial import run_trial
from openrua.sandbox import workspace

# Run data lands under the caller's working directory by default
# (--runs-root overrides); the package never writes into itself.
RUNS_ROOT = Path("runs")



def add_arguments(ap: argparse.ArgumentParser, include_home: bool = True) -> None:
    """The ``openrua bench`` arguments, on any parser (the cli passes its
    own subparser and supplies ``--home`` itself)."""
    ap.add_argument("--config", required=True,
                    help="benchmark config (benchmarks/<name>.yaml)")
    ap.add_argument("--robot", default=None,
                    help="robot profile name or path; overrides the "
                    "config's robot: line")
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--task-suite", required=True)
    ap.add_argument("--task-ids", default="0")
    ap.add_argument("--seeds", default="0")
    ap.add_argument("--operator", default="none", choices=sorted(OPERATORS))
    ap.add_argument(
        "--task", default=None,
        help="task sentence for a real robot (no bridge to ask); a simulated "
        "robot's task comes from the benchmark and this is ignored",
    )
    ap.add_argument(
        "--script", default=None,
        help="command-sequence file for --operator script (canonical "
        "source: a trial's commands.sh condensate); runs in the sandbox "
        "on the native surface, open-loop best-effort",
    )
    ap.add_argument(
        "--record", nargs="?", const="", default=None, metavar="CAMERAS",
        help="record the simulated robot's cameras every sim step into the "
        "trial's frames/ (what `openrua demo` renders); a comma-separated "
        "camera list, or none for the profile's cameras.record. Rendering "
        "costs wall clock on whole-room scenes: record a replay "
        "(--operator script), not the experiment",
    )
    ap.add_argument(
        "--wall-clock-min", type=float, default=None,
        help="override of the config's protocol.active_wall_clock_minutes "
        "(the config is the default's single source)",
    )
    ap.add_argument(
        "--ros-domain", type=int, default=0,
        help="ROS_DOMAIN_ID for this run's containers; concurrent runs "
        "must use distinct domains (one DDS network would cross-talk)",
    )
    ap.add_argument(
        "--credentials-dir", default=None,
        help="agent login-profile override (default: the config's "
        "agent.credentials_dir, then ~/.openrua/credentials/<agent>)",
    )
    ap.add_argument(
        "--token-file", default=None,
        help="file holding <token_env>=<token> for the sandbox CLI; given, "
        "the sandbox authenticates with that token and no credentials "
        "file is mounted",
    )
    ap.add_argument(
        "--runs-root", default=None,
        help="where run data lands (default: ./runs)",
    )
    if include_home:
        ap.add_argument(
            "--home", default=None,
            help="the user directory (default: ~/.openrua); robot and benchmark "
            "names, simulators and login profiles are looked up under it",
        )
    ap.add_argument(
        "--account-alias", default=None,
        help="non-secret label of the credentials profile, recorded in the "
        "trial result for per-account accounting",
    )


def run(args: argparse.Namespace) -> int:
    """Run the task set described by parsed arguments; returns the exit status."""
    if args.token_file:
        # See --token-file: the launcher runs with cwd=trial_dir, so a
        # relative path would resolve to nothing by the time docker reads it.
        args.token_file = str(Path(args.token_file).expanduser().resolve())
    args.task_ids = [int(x) for x in str(args.task_ids).split(",")]
    args.seeds = [int(x) for x in str(args.seeds).split(",")]
    # None: no recording; (): the profile's cameras.record; names: those.
    # Parsed once, here; provenance records the same value.
    args.record = (None if args.record is None
                   else tuple(c.strip() for c in args.record.split(",") if c.strip()))
    record_cameras = args.record

    home = paths.home(args.home)
    cfg_path = paths.find("benchmarks", args.config, home).resolve()
    cfg = load_config(cfg_path, args.robot, home)
    if cfg["machine"]["backend"].get("kind") != "sim" and not args.task:
        raise UsageError("a real robot has no benchmark task to ask for",
                         hint="pass --task \"<what the agent should do>\"")
    if record_cameras is not None and cfg["machine"]["backend"].get("kind") != "sim":
        raise UsageError("a real robot has no renderer to record from",
                         hint="drop --record")
    if record_cameras == () and not cfg["machine"].get("cameras", {}).get("record"):
        raise UsageError("--record needs camera names: the robot profile sets no "
                         "cameras.record", hint="pass --record <main>,<inset> or add "
                         "cameras.record to the profile")
    if args.wall_clock_min is None:
        args.wall_clock_min = resolve_wall_clock_min(cfg)
    # The per-suite view, computed exactly once; everyone downstream
    # reads the resulting file.
    apply_suite_overrides(cfg, args.task_suite)
    normalize_arms(cfg)
    runs_root = Path(args.runs_root).expanduser() if args.runs_root else RUNS_ROOT
    run_dir = runs_root.resolve() / cfg["task"]["benchmark"] / args.run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    agent = agents.get(cfg.get("agent", {}).get("name"), home,
                       version=cfg.get("agent", {}).get("version"))
    prov = {**record.provenance(
        cfg_path, cfg, args, agent=agent,
        template_hash=workspace.template_hash(cfg),
        prompt=agents.PROMPT, resume_prompt=agents.RESUME_PROMPT,
        code_root=paths.code_root(), proxy_image=proxy.IMAGE,
        simulator_venv=simulator_venv(cfg["machine"].get("backend", {}), home),
    ), "config": cfg}
    record.write_run_config(run_dir, prov)

    op = OPERATORS[args.operator]
    for task_id in args.task_ids:
        for seed in args.seeds:
            try:
                rec = run_trial(
                    cfg, cfg_path, run_dir, args.task_suite, task_id, seed, op,
                    args.wall_clock_min, ros_domain=args.ros_domain,
                    credentials_dir=args.credentials_dir,
                    account_alias=args.account_alias, script=args.script,
                    token_file=args.token_file, home=home, task=args.task,
                    record_cameras=record_cameras,
                )
            except lock.TrialLocked as e:
                # Not a failure of this trial: someone else is doing it.
                raise SystemExit(f"[trial] {e}")
            trial_dir = (run_dir / "trials" / f"{args.task_suite}-{task_id}"
                         / f"seed{seed}")
            record.write_trial_provenance(trial_dir, prov)
            record.write_run_summary(run_dir)
            print(
                f"[{args.task_suite}:{task_id} seed{seed}] "
                f"success={rec['success']} term={rec['termination']} "
                f"wall={rec['wall_seconds']}s"
            )
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(prog="openrua bench", description=__doc__.split("\n\n")[0])
    add_arguments(ap)
    return run(ap.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
