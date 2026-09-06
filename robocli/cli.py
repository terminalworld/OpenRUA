"""The front door: ``robocli <verb> ...``.

    robocli robots                      list the shipped robot profiles
    robocli build <robot|sandbox|proxy> build one of the three images
    robocli up --robot panda-sim        a live robot (real or simulated) with
                                        a sandbox terminal on its ROS 2 graph
    robocli agent [--cli <name>]        open a coding agent on that terminal
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

from robocli import __version__, agents, config, paths
from robocli.bench import record
from robocli.bench.run import (MachineClient, apply_suite_overrides,
                               ensure_internal_network, load_config,
                               load_robot, normalize_arms, resolve_body_files,
                               substrate_venv, FASTDDS_PEERS_XML)
from robocli.proxy.up import ensure as ensure_proxy
from robocli.robot.down import down as robot_down
from robocli.robot.up import up as robot_up
from robocli.sandbox.down import down as sandbox_down
from robocli.sandbox.up import up as sandbox_up

DEFAULT_NAME = "robocli"


# ----------------------------------------------------------------- helpers

def _names(name: str) -> tuple[str, str]:
    return f"{name}-sim", f"{name}-sandbox"


def _state_path(name: str, home: Path) -> Path:
    return paths.state_dir(home) / f"{name}.yaml"


def _save_state(name: str, home: Path, **facts) -> None:
    paths.state_dir(home).mkdir(parents=True, exist_ok=True)
    _state_path(name, home).write_text(yaml.safe_dump(facts, sort_keys=False))


def _load_state(name: str, home: Path) -> dict:
    p = _state_path(name, home)
    if not p.is_file():
        raise SystemExit(
            f"no live robot named {name!r} (nothing at {p}); "
            f"start one with: robocli up --robot <profile> --name {name}")
    return yaml.safe_load(p.read_text())


def compose(robot: str | None, bench: str | None,
            home: Path | None = None) -> tuple[dict, str, int]:
    """robot profile + benchmark config -> one assembled config, plus the
    (task_suite, task_id) to load. Without --bench the profile's
    ``world:`` says which scene to load."""
    if not robot and not bench:
        raise SystemExit("name a robot (--robot) or a benchmark (--bench)")
    world = {}
    if robot:
        world = load_robot(robot, home).get("world", {})
    bench = bench or world.get("benchmark")
    if not bench:
        raise SystemExit(f"robot {robot!r} names no world: and no --bench given")
    cfg = load_config(bench, robot, home)
    suite = world.get("task_suite") or cfg["task"]["suites"][0]
    task_id = int(world.get("task_id", 0))
    return cfg, suite, task_id


# ------------------------------------------------------------------- verbs

def cmd_robots(args) -> int:
    """Bundled profiles, then the user's (``<home>/robots/``); a user
    file shadowed by a bundled name is pointed out, not used."""
    for e in paths.available("robots", args.home):
        try:
            r = yaml.safe_load(e.path.read_text()).get("machine", {}).get("robot", {})
            desc = f"{r.get('model', '?'):<28} {r.get('description', '')}"
        except Exception as exc:  # noqa: BLE001  a bad user file must not hide the rest
            desc = f"(unreadable: {exc})"
        tag = "" if e.source == "bundled" else "  [user]"
        print(f"{e.name:<20} {desc}{tag}")
        if e.shadowed_by:
            print(f"{'':<20} note: {e.shadowed_by} has the same name and is ignored; "
                  f"rename it to use it")
    return 0


def cmd_build(args) -> int:
    module = f"robocli.{args.unit}.build"
    sys.argv = [f"robocli build {args.unit}", *args.rest]
    return importlib.import_module(module).main()


def cmd_up(args) -> int:
    cfg, suite, task_id = compose(args.robot, args.bench, args.home)
    suite = args.task_suite or suite
    task_id = args.task_id if args.task_id is not None else task_id
    apply_suite_overrides(cfg, suite)
    normalize_arms(cfg)
    sim_name, sandbox_name = _names(args.name)
    workdir = Path(args.workspace or paths.workspaces_dir(args.home) / args.name).resolve()
    if workdir.exists():
        shutil.rmtree(workdir)  # a fresh workspace every time (seeding merges)
    workdir.mkdir(parents=True)
    resolve_body_files(cfg, workdir, args.home)
    assembly = record.write_assembly(workdir, cfg)
    network = ensure_internal_network()
    proxy_url = ensure_proxy(network)
    adapter = agents.get(args.cli or cfg.get("agent", {}).get("cli"), args.home)
    creds_home = Path(cfg.get("agent", {}).get("credentials_dir")
                      or paths.credentials_dir(args.home) / adapter.name).expanduser()
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
    venv = substrate_venv(body, args.home)
    proc = robot_up(
        name=sim_name, image=body.get("image", "robocli-sim-jazzy"),
        config_path=str(assembly), task_suite=suite, task_id=task_id,
        substrate=str(paths.substrate_root(venv)), venv=str(venv),
        code_root=str(paths.code_root()), log_path=workdir / "robot.log",
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
    _save_state(args.name, args.home, sim=sim_name, sandbox=sandbox_name,
                network=network, proxy=proxy_url, cli=adapter.name,
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
        _state_path(args.name, args.home).unlink(missing_ok=True)
        shutil.rmtree(cfg_dir, ignore_errors=True)
    return 0


def cmd_agent(args) -> int:
    st = _load_state(args.name, args.home)
    adapter = agents.get(args.cli or st["cli"], args.home)
    argv = adapter.interactive_argv(
        st["sandbox"], args.model or st["model"], st["proxy"],
        options=st.get("options"), prompt=args.prompt)
    if argv is None:
        raise SystemExit(
            f"{adapter.name} has no interactive mode; open a shell on the "
            f"terminal instead:\n  docker exec -it -u robot -w /workspace "
            f"{st['sandbox']} bash")
    os.execvp(argv[0], argv)


def cmd_down(args) -> int:
    sim_name, sandbox_name = _names(args.name)
    sandbox_down(sandbox_name)
    robot_down(sim_name)
    _state_path(args.name, args.home).unlink(missing_ok=True)
    return 0


def cmd_config(args) -> int:
    if args.what == "schema":
        import json
        print(json.dumps(config.Assembly.model_json_schema(), indent=2))
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
    cfg, _, _ = compose(args.robot, None, args.home)
    body = cfg["machine"].get("body", {})
    for label, image in (("robot image", body.get("image")),
                         ("sandbox image", body.get("sandbox_image")),
                         ("proxy image", "robocli-proxy")):
        present = subprocess.run(["docker", "image", "inspect", image],
                                 capture_output=True).returncode == 0
        check(f"{label} {image}", present, f"robocli build {label.split()[0]}")
    venv = substrate_venv(body, args.home)
    check(f"substrate venv {venv}", venv.is_dir(),
          f"build it under {paths.substrates_dir(args.home)} (docs/simulation.md) "
          "or point machine.body.substrate.venv at it")
    adapter = agents.get(cfg.get("agent", {}).get("cli"), args.home)
    if adapter.credentials is not None:
        creds = Path(cfg.get("agent", {}).get("credentials_dir")
                     or paths.credentials_dir(args.home) / adapter.name).expanduser()
        check(f"{adapter.name} login at {creds}",
              (creds / adapter.credentials.filename).exists(),
              adapter.login_hint(creds))
    return 0 if ok else 1


# --------------------------------------------------------------------- main

def build_parser(default_home: str | None = None) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="robocli", description=__doc__.split("\n\n")[0])
    ap.add_argument("--version", action="version", version=f"robocli {__version__}")
    ap.add_argument("--home", default=default_home, type=paths.home,
                    help="the user directory: your robots/, benchmarks/, agents/, "
                    "credentials/, substrates/ (default: $ROBOCLI_HOME or ~/.robocli)")
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
    p.add_argument("--workspace", default=None,
                   help="working directory (default: <home>/workspaces/<name>)")
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

    p = sub.add_parser("config", help="the configuration schema")
    p.add_argument("what", choices=("schema",),
                   help="schema: print the assembled config's JSON schema")
    p.set_defaults(fn=cmd_config)

    p = sub.add_parser("doctor", help="check the install")
    p.add_argument("--robot", default="panda-sim")
    p.set_defaults(fn=cmd_doctor)
    return ap


VERBS = ("robots", "build", "up", "agent", "down", "run", "config", "doctor")
# Verbs that forward their whole argv to another front door (argparse
# would otherwise eat their --help).
FORWARDED = {"run": "robocli.bench.run"}


def main() -> int:
    # The one place the environment is read: ROBOCLI_HOME seeds --home's
    # default; from here on the user directory travels as a parameter.
    ap = build_parser(default_home=os.environ.get("ROBOCLI_HOME") or None)
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
    try:
        return args.fn(args) or 0
    except (config.ConfigError, FileNotFoundError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 78 if isinstance(e, config.ConfigError) else 66


if __name__ == "__main__":
    sys.exit(main())
