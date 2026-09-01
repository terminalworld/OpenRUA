"""The front door: ``robocli <verb> ...``.

    robocli robots                      list the shipped robot profiles
    robocli build <robot|sandbox|proxy> build one of the three images
    robocli up --robot panda-sim        a live robot (real or simulated) with
                                        a sandbox terminal on its ROS 2 graph
    robocli agent [--cli claude-code]   open a coding agent on that terminal
    robocli down                        power everything off
    robocli run --config benchmarks/... run a task set (robocli.bench.run)
    robocli doctor                      check docker, images, substrate, login

``up`` stays in the foreground (like ``docker compose up``): the robot
holds its control line to this process and powers itself off when the
process ends. Open a second terminal for ``agent``.

The verbs below are thin: every capability lives in its own unit
(robot, sandbox, proxy, agents, bench), each with its own front door
(``python -m robocli.<unit> --help``). This file only composes them.
"""

from __future__ import annotations

import argparse
import importlib
import os
import shutil
import signal
import subprocess
import sys
from pathlib import Path

import yaml

from robocli import agents
from robocli.bench import record
from robocli.bench.run import (REPO_ROOT, MachineClient, apply_suite_overrides,
                               ensure_internal_network, load_config,
                               load_robot, normalize_arms, substrate_venv,
                               FASTDDS_PEERS_XML)
from robocli.proxy.up import ensure as ensure_proxy
from robocli.robot.down import down as robot_down
from robocli.robot.up import up as robot_up
from robocli.sandbox.down import down as sandbox_down
from robocli.sandbox.up import up as sandbox_up

DEFAULT_NAME = "robocli"
STATE_DIR = Path("~/.robocli").expanduser()


# ----------------------------------------------------------------- helpers

def _names(name: str) -> tuple[str, str]:
    return f"{name}-sim", f"{name}-sandbox"


def _state_path(name: str) -> Path:
    return STATE_DIR / f"{name}.yaml"


def _save_state(name: str, **facts) -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    _state_path(name).write_text(yaml.safe_dump(facts, sort_keys=False))


def _load_state(name: str) -> dict:
    p = _state_path(name)
    if not p.is_file():
        raise SystemExit(
            f"no live robot named {name!r} (nothing at {p}); "
            f"start one with: robocli up --robot <profile> --name {name}")
    return yaml.safe_load(p.read_text())


def compose(robot: str | None, bench: str | None) -> tuple[dict, str, int]:
    """robot profile + benchmark config -> one assembled config, plus the
    (task_suite, task_id) to load. Without --bench the profile's
    ``world:`` says which scene to load."""
    if not robot and not bench:
        raise SystemExit("name a robot (--robot) or a benchmark (--bench)")
    world = {}
    if robot:
        world = load_robot(robot).get("world", {})
    bench = bench or world.get("benchmark")
    if not bench:
        raise SystemExit(f"robot {robot!r} names no world: and no --bench given")
    bench_path = Path(bench)
    if not bench_path.suffix:
        bench_path = REPO_ROOT / "benchmarks" / f"{bench}.yaml"
    cfg = load_config(bench_path, robot)
    suite = world.get("task_suite") or cfg["task"]["suites"][0]
    task_id = int(world.get("task_id", 0))
    return cfg, suite, task_id


# ------------------------------------------------------------------- verbs

def cmd_robots(args) -> int:
    for p in sorted((REPO_ROOT / "robots").glob("*.yaml")):
        d = yaml.safe_load(p.read_text())
        m = d.get("machine", {})
        r = m.get("robot", {})
        print(f"{p.stem:<20} {r.get('model', '?'):<28} {r.get('description', '')}")
    return 0


def cmd_build(args) -> int:
    module = f"robocli.{args.unit}.build"
    sys.argv = [f"robocli build {args.unit}", *args.rest]
    return importlib.import_module(module).main()


def cmd_up(args) -> int:
    cfg, suite, task_id = compose(args.robot, args.bench)
    suite = args.task_suite or suite
    task_id = args.task_id if args.task_id is not None else task_id
    apply_suite_overrides(cfg, suite)
    normalize_arms(cfg)
    sim_name, sandbox_name = _names(args.name)
    workdir = Path(args.workspace or f"workspaces/{args.name}").resolve()
    if workdir.exists():
        shutil.rmtree(workdir)  # a fresh workspace every time (seeding merges)
    workdir.mkdir(parents=True)
    assembly = record.write_assembly(workdir, cfg)
    network = ensure_internal_network()
    proxy_url = ensure_proxy(network)
    adapter = agents.get(args.cli or cfg.get("agent", {}).get("cli"))
    creds_home = Path(cfg.get("agent", {}).get(
        "credentials_dir", adapter.DEFAULT_CREDENTIALS_DIR)).expanduser()
    cfg_dir, creds_file = agents.prepare_profile(creds_home, adapter)
    body = cfg["machine"].get("body", {})
    print(f"[up] sandbox {sandbox_name}", flush=True)
    sandbox_up(config=assembly, workspace=workdir / "workspace",
               image=body.get("sandbox_image"), network=network,
               static_peer=sim_name,
               peers_xml=FASTDDS_PEERS_XML.format(peer=sim_name),
               ros_domain=args.ros_domain, internet=f"proxy:{proxy_url}",
               name=sandbox_name,
               mounts=adapter.sandbox_mounts(cfg_dir, creds_file))
    print(f"[up] robot {sim_name} (booting; MoveIt takes a minute)", flush=True)
    venv = substrate_venv(body)
    proc = robot_up(
        name=sim_name, image=body.get("image", "robocli-sim-jazzy"),
        config_path=str(assembly), task_suite=suite, task_id=task_id,
        substrate=str(venv).rsplit("/.venv", 1)[0], venv=str(venv),
        repo_root=str(REPO_ROOT), log_path=workdir / "robot.log",
        moveit_log=str(workdir / "moveit.log"), network=network,
        static_peer=sandbox_name,
        peers_xml=FASTDDS_PEERS_XML.format(peer=sandbox_name),
        ros_domain=args.ros_domain, gpus=bool(body.get("gpus", False)),
        resources=body.get("resources"))
    machine = MachineClient(proc, sim_name)
    try:
        machine.wait_ready()
        r = machine.rpc({"cmd": "reset", "init_state_id": args.init_state})
        if not r.get("ok"):
            raise RuntimeError(f"reset failed: {r}")
        task = machine.rpc({"cmd": "task_info"}).get("language", "")
    except Exception as e:  # noqa: BLE001
        sandbox_down(sandbox_name)
        machine.shutdown()
        raise SystemExit(f"[up] robot failed to come up: {e}\n"
                         f"      log: {workdir / 'robot.log'}")
    _save_state(args.name, sim=sim_name, sandbox=sandbox_name,
                network=network, proxy=proxy_url, cli=adapter.NAME,
                model=cfg.get("agent", {}).get("model", adapter.DEFAULT_MODEL),
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
        _state_path(args.name).unlink(missing_ok=True)
        shutil.rmtree(cfg_dir, ignore_errors=True)
    return 0


def cmd_agent(args) -> int:
    st = _load_state(args.name)
    adapter = agents.get(args.cli or st["cli"])
    argv = adapter.interactive_argv(
        st["sandbox"], args.model or st["model"], st["proxy"],
        prompt=args.prompt)
    os.execvp(argv[0], argv)


def cmd_down(args) -> int:
    sim_name, sandbox_name = _names(args.name)
    sandbox_down(sandbox_name)
    robot_down(sim_name)
    _state_path(args.name).unlink(missing_ok=True)
    return 0


def cmd_doctor(args) -> int:
    ok = True

    def check(label: str, good: bool, fix: str = "") -> None:
        nonlocal ok
        ok &= good
        print(f"  [{'ok' if good else '!!'}] {label}" + ("" if good else f"\n       fix: {fix}"))

    print("robocli doctor")
    check("docker on PATH", shutil.which("docker") is not None,
          "install Docker Engine: https://docs.docker.com/engine/install/")
    cfg, _, _ = compose(args.robot, None)
    body = cfg["machine"].get("body", {})
    for label, image in (("robot image", body.get("image")),
                         ("sandbox image", body.get("sandbox_image")),
                         ("proxy image", "robocli-proxy")):
        present = subprocess.run(["docker", "image", "inspect", image],
                                 capture_output=True).returncode == 0
        check(f"{label} {image}", present, f"robocli build {label.split()[0]}")
    venv = substrate_venv(body)
    check(f"substrate venv {venv}", venv.is_dir(), "see docs/simulation.md")
    adapter = agents.get(cfg.get("agent", {}).get("cli"))
    creds = Path(cfg.get("agent", {}).get(
        "credentials_dir", adapter.DEFAULT_CREDENTIALS_DIR)).expanduser()
    check(f"{adapter.NAME} login at {creds}",
          (creds / adapter.CREDENTIALS_FILENAME).exists(),
          adapter.login_hint(creds))
    return 0 if ok else 1


# --------------------------------------------------------------------- main

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="robocli", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="verb", metavar="<verb>")

    p = sub.add_parser("robots", help="list the shipped robot profiles")
    p.set_defaults(fn=cmd_robots)

    p = sub.add_parser("build", help="build the robot / sandbox / proxy image")
    p.add_argument("unit", choices=("robot", "sandbox", "proxy"))
    p.add_argument("rest", nargs=argparse.REMAINDER, help="unit's own options")
    p.set_defaults(fn=cmd_build)

    p = sub.add_parser("up", help="bring a robot up with a sandbox terminal on it")
    p.add_argument("--robot", help="profile name under robots/ or a path")
    p.add_argument("--bench", help="benchmark config to take the scene from")
    p.add_argument("--task-suite", default=None)
    p.add_argument("--task-id", type=int, default=None)
    p.add_argument("--init-state", type=int, default=0)
    p.add_argument("--name", default=DEFAULT_NAME, help="handle for this robot")
    p.add_argument("--cli", default=None, help="agent adapter to seat")
    p.add_argument("--workspace", default=None, help="default workspaces/<name>")
    p.add_argument("--ros-domain", type=int, default=0)
    p.set_defaults(fn=cmd_up)

    p = sub.add_parser("agent", help="open a coding agent on the robot's terminal")
    p.add_argument("prompt", nargs="?", default=None, help="opening message")
    p.add_argument("--name", default=DEFAULT_NAME)
    p.add_argument("--cli", default=None)
    p.add_argument("--model", default=None)
    p.set_defaults(fn=cmd_agent)

    p = sub.add_parser("down", help="power a robot and its terminal off")
    p.add_argument("--name", default=DEFAULT_NAME)
    p.set_defaults(fn=cmd_down)

    sub.add_parser("run", help="run a task set (robocli run --help)")

    p = sub.add_parser("doctor", help="check the install")
    p.add_argument("--robot", default="panda-sim")
    p.set_defaults(fn=cmd_doctor)
    return ap


VERBS = ("robots", "build", "up", "agent", "down", "run", "doctor")
# Verbs that forward their whole argv to another front door (argparse
# would otherwise eat their --help).
FORWARDED = {"run": "robocli.bench.run"}


def main() -> int:
    ap = build_parser()
    argv = sys.argv[1:]
    if argv and not argv[0].startswith("-") and argv[0] not in VERBS:
        print(f"unknown verb {argv[0]!r}\n", file=sys.stderr)
        ap.print_help(sys.stderr)
        return 2
    if argv and argv[0] in FORWARDED:
        sys.argv = [f"robocli {argv[0]}", *argv[1:]]
        return importlib.import_module(FORWARDED[argv[0]]).main() or 0
    args = ap.parse_args(argv)
    if not args.verb:
        ap.print_help()
        return 0
    return args.fn(args) or 0


if __name__ == "__main__":
    sys.exit(main())
