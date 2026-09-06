"""``robocli up <robot>``: a live robot with a sandbox terminal on it.

Stays in the foreground: the robot holds its control line to this
process and powers itself off when the process ends. Open a second
terminal for ``robocli agent``.
"""

from __future__ import annotations

import shutil
import signal
from pathlib import Path

from robocli import agents
from robocli.cli import state
from robocli.cli.state import DEFAULT_NAME
from robocli.config import apply_suite_overrides, compose, normalize_arms
from robocli.config import paths
from robocli.errors import UnavailableError
from robocli.proxy.up import ensure as ensure_proxy
from robocli.runner.bringup import bring_up, ensure_internal_network
from robocli.sandbox.down import down as sandbox_down


def run(args) -> int:
    cfg, suite, task_id = compose(args.robot, args.bench, args.home)
    suite = args.task_suite or suite
    task_id = args.task_id if args.task_id is not None else task_id
    apply_suite_overrides(cfg, suite)
    normalize_arms(cfg)
    sim_name, sandbox_name = state.container_names(args.name)
    workdir = Path(args.workspace or paths.workspaces_dir(args.home) / args.name).resolve()
    if workdir.exists():
        shutil.rmtree(workdir)  # a fresh workspace every time (seeding merges)
    workdir.mkdir(parents=True)
    network = ensure_internal_network()
    proxy_url = ensure_proxy(network)
    adapter = agents.get(args.agent or cfg.get("agent", {}).get("name"), args.home,
                         version=cfg.get("agent", {}).get("version"))
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
    try:
        if cfg["machine"]["backend"]["kind"] == "sim":
            r = machine.rpc({"cmd": "reset", "init_state_id": args.init_state})
            if not r.get("ok"):
                raise RuntimeError(f"reset failed: {r}")
            task = machine.rpc({"cmd": "task_info"}).get("language", "")
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
                model=cfg.get("agent", {}).get("model") or adapter.default_model,
                options={**adapter.default_options,
                         **cfg.get("agent", {}).get("options", {})},
                workspace=str(workdir / "workspace"), task=task)
    print(f"""
[up] ready.
     robot     {sim_name}   (ROS 2 graph live; scene: {suite} #{task_id})
     terminal  {sandbox_name}
     task      {task or '(none)'}

     robocli agent --name {args.name}            # your coding agent, on the robot
     docker exec -it -u robot -w /workspace {sandbox_name} bash   # or you

     Ctrl-C here powers the robot off.""", flush=True)
    stop = signal.SIGINT
    try:
        signal.sigwait([stop, signal.SIGTERM])
    finally:
        print("\n[down] powering off", flush=True)
        sandbox_down(sandbox_name)
        machine.shutdown()
        state.forget(args.name, args.home)
        shutil.rmtree(cfg_dir, ignore_errors=True)
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("up", help="bring a robot up with a sandbox terminal on it",
                       description="Bring a robot up (simulated: boot its container; "
                       "real: join its graph) with a sandbox terminal on it, then stay "
                       "in the foreground; Ctrl-C powers it off. Open a second terminal "
                       "for `robocli agent`.")
    p.add_argument("robot", nargs="?", default=None,
                   help="robot profile: a name (robocli robots) or a path; "
                   "default: --bench's robot, else the user config's default")
    p.add_argument("--bench", default=None,
                   help="benchmark to take the scene from (default: the profile's world:)")
    p.add_argument("--task-suite", default=None, help="scene suite (default: the profile's)")
    p.add_argument("--task-id", type=int, default=None, help="scene index (default: the profile's)")
    p.add_argument("--init-state", type=int, default=0,
                   help="episode seed / init state (simulated robots)")
    p.add_argument("--task", default=None,
                   help="task sentence to show `robocli agent` (real robots; a "
                   "simulated robot's comes from the scene)")
    p.add_argument("--name", default=DEFAULT_NAME,
                   help=f"handle for this robot, for agent/down (default: {DEFAULT_NAME})")
    p.add_argument("--agent", default=None, help="agent to open (default: the config's)")
    p.add_argument("--workspace", default=None,
                   help="working directory (default: <home>/workspaces/<name>)")
    p.add_argument("--ros-domain", type=int, default=0,
                   help="ROS_DOMAIN_ID; concurrent robots need distinct ones")
    p.set_defaults(fn=run)
