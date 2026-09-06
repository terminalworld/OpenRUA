"""The front door: ``robocli <verb> ...``.

    robocli robots | benchmarks | agents   what is available (bundled + yours)
    robocli build <robot|sandbox|proxy>    build one of the three images
    robocli up panda-sim                   a live robot (real or simulated) with
                                           a sandbox terminal on its ROS 2 graph
    robocli agent                          open a coding agent on that terminal
    robocli down                           power everything off
    robocli run --config libero_pro ...    run a task set (robocli run --help)
    robocli config schema                  every config key and its meaning
    robocli doctor [robot]                 check docker, images, simulator, login

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
import json
import os
import shutil
import signal
import subprocess
import sys
from pathlib import Path

import yaml

from robocli import __version__, agents, doctor
from robocli.config import paths
from robocli.errors import NotFound, RoboCLIError, UnavailableError
from robocli.bench import record
from robocli.bench.run import (apply_suite_overrides, bring_up, compose,
                               ensure_internal_network, normalize_arms)
from robocli.proxy.up import ensure as ensure_proxy
from robocli.robot.down import down as robot_down
from robocli.sandbox.down import down as sandbox_down

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
        raise NotFound(f"no live robot named {name!r} (nothing at {p})",
                       hint=f"robocli up <robot> --name {name}")
    return yaml.safe_load(p.read_text())


# ------------------------------------------------------------------- verbs

def _describe(kind: str, path: Path) -> str:
    """One line about a robot profile or a benchmark config, or why it
    could not be read (a bad user file must not hide the rest)."""
    try:
        d = yaml.safe_load(path.read_text()) or {}
    except Exception as exc:  # noqa: BLE001
        return f"(unreadable: {exc})"
    if kind == "robots":
        r = d.get("machine", {}).get("robot", {})
        return f"{r.get('model', '?'):<28} {r.get('description', '')}"
    t = d.get("task", {})
    suites = t.get("suites", [])
    return f"{t.get('benchmark', '?'):<14} {len(suites)} suites; robot: {d.get('robot', '(own machine:)')}"


def _print_listing(rows: list[dict], as_json: bool) -> None:
    """Rows of {name, source, description, shadowed_by}: a table, or JSON."""
    if as_json:
        print(json.dumps(rows, indent=2))
        return
    for r in rows:
        tag = "" if r["source"] == "bundled" else f"  [{r['source']}]"
        print(f"{r['name']:<20} {r['description']}{tag}")
        if r.get("shadowed_by"):
            print(f"{'':<20} note: {r['shadowed_by']} has the same name and is "
                  "ignored; rename it to use it")


def _list_kind(kind: str, args) -> int:
    """Bundled entries, then the user's (``<home>/<kind>/``); a user file
    shadowed by a bundled name is pointed out, not used."""
    rows = [{"name": e.name, "source": e.source, "path": str(e.path),
             "description": _describe(kind, e.path),
             "shadowed_by": str(e.shadowed_by) if e.shadowed_by else None}
            for e in paths.available(kind, args.home)]
    _print_listing(rows, args.json)
    return 0


def cmd_robots(args) -> int:
    return _list_kind("robots", args)


def cmd_benchmarks(args) -> int:
    return _list_kind("benchmarks", args)


def cmd_agents(args) -> int:
    rows = []
    for a in agents.available(args.home):
        desc = (f"FAILED: {a.error}" if a.agent is None else
                f"{a.agent.default_model:<24} {' '.join(sorted(a.agent.capabilities))}")
        rows.append({"name": a.name, "source": a.source, "path": str(a.path),
                     "description": desc,
                     "shadowed_by": str(a.shadowed_by) if a.shadowed_by else None,
                     "capabilities": sorted(a.agent.capabilities) if a.agent else None})
    _print_listing(rows, args.json)
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
    network = ensure_internal_network()
    proxy_url = ensure_proxy(network)
    adapter = agents.get(args.agent or cfg.get("agent", {}).get("name"), args.home)
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
        r = machine.rpc({"cmd": "reset", "init_state_id": args.init_state})
        if not r.get("ok"):
            raise RuntimeError(f"reset failed: {r}")
        task = machine.rpc({"cmd": "task_info"}).get("language", "")
    except Exception as e:  # noqa: BLE001
        sandbox_down(sandbox_name)
        machine.shutdown()
        shutil.rmtree(cfg_dir, ignore_errors=True)
        raise UnavailableError(f"[up] robot failed to reset: {e}",
                               hint=f"read {workdir / 'robot.log'}") from e
    _save_state(args.name, args.home, sim=sim_name, sandbox=sandbox_name,
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
        _state_path(args.name, args.home).unlink(missing_ok=True)
        shutil.rmtree(cfg_dir, ignore_errors=True)
    return 0


def cmd_agent(args) -> int:
    st = _load_state(args.name, args.home)
    adapter = agents.get(args.agent or st["agent"], args.home)
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
        from robocli import config
        print(json.dumps(config.ResolvedConfig.model_json_schema(), indent=2))
    return 0


def cmd_doctor(args) -> int:
    report = doctor.run(robot=args.robot, agent_names=args.agent, home=args.home)
    return doctor.main_report(report, as_json=True if args.json else None)


# --------------------------------------------------------------------- main

def build_parser(default_home: str | None = None) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="robocli", description=__doc__.split("\n\n")[0])
    ap.add_argument("--version", action="version", version=f"robocli {__version__}")
    ap.add_argument("--home", default=default_home, type=paths.home,
                    help="the user directory: your robots/, benchmarks/, agents/, "
                    "credentials/, simulators/ (default: $ROBOCLI_HOME or ~/.robocli)")
    sub = ap.add_subparsers(dest="verb", metavar="<verb>")

    def add_json(parser):
        parser.add_argument("--json", action="store_true",
                            help="machine-readable output")

    # Registration order is the --help order; keep it stable.
    for kind, fn in (("robots", cmd_robots), ("benchmarks", cmd_benchmarks),
                     ("agents", cmd_agents)):
        p = sub.add_parser(kind, help=f"list the {kind}: bundled, then ~/.robocli/{kind}/",
                           description=f"Every {kind[:-1]} RoboCLI can find: the bundled "
                           f"ones, then yours under <home>/{kind}/. A user file that "
                           "carries a bundled name is reported and not used.")
        add_json(p)
        p.set_defaults(fn=fn)

    p = sub.add_parser("build", help="build the robot / sandbox / proxy image",
                       description="Build one image; the unit's own options follow "
                       "(robocli build sandbox --help).")
    p.add_argument("unit", choices=("robot", "sandbox", "proxy"))
    p.add_argument("rest", nargs=argparse.REMAINDER, help="unit's own options")
    p.set_defaults(fn=cmd_build)

    p = sub.add_parser("up", help="bring a robot up with a sandbox terminal on it",
                       description="Bring a robot up (simulated: boot its body; real: "
                       "join its graph) with a sandbox terminal on it, then stay in the "
                       "foreground; Ctrl-C powers it off. Open a second terminal for "
                       "`robocli agent`.")
    p.add_argument("robot", nargs="?", default=None,
                   help="robot profile: a name (robocli robots) or a path; "
                   "default: --bench's robot, else the user config's default")
    p.add_argument("--bench", default=None,
                   help="benchmark to take the scene from (default: the profile's world:)")
    p.add_argument("--task-suite", default=None, help="scene suite (default: the profile's)")
    p.add_argument("--task-id", type=int, default=None, help="scene index (default: the profile's)")
    p.add_argument("--init-state", type=int, default=0, help="episode seed / init state")
    p.add_argument("--name", default=DEFAULT_NAME,
                   help=f"handle for this robot, for agent/down (default: {DEFAULT_NAME})")
    p.add_argument("--agent", default=None, help="agent adapter to seat (default: the config's)")
    p.add_argument("--workspace", default=None,
                   help="working directory (default: <home>/workspaces/<name>)")
    p.add_argument("--ros-domain", type=int, default=0,
                   help="ROS_DOMAIN_ID; concurrent robots need distinct ones")
    p.set_defaults(fn=cmd_up)

    p = sub.add_parser("agent", help="open a coding agent on the robot's terminal",
                       description="Open the seated coding agent interactively on a "
                       "live robot's terminal (docker exec into its sandbox).")
    p.add_argument("prompt", nargs="?", default=None, help="opening message")
    p.add_argument("--name", default=DEFAULT_NAME, help=f"the robot's handle (default: {DEFAULT_NAME})")
    p.add_argument("--agent", default=None, help="agent adapter (default: the one `up` seated)")
    p.add_argument("--model", default=None, help="model (default: the one `up` recorded)")
    p.set_defaults(fn=cmd_agent)

    p = sub.add_parser("down", help="power a robot and its terminal off")
    p.add_argument("--name", default=DEFAULT_NAME, help=f"the robot's handle (default: {DEFAULT_NAME})")
    p.set_defaults(fn=cmd_down)

    sub.add_parser("run", help="run a task set (robocli run --help)")

    p = sub.add_parser("config", help="the configuration schema",
                       description="Configuration: `robocli config schema` prints the "
                       "assembled config's JSON schema, every key with its meaning.")
    p.add_argument("what", choices=("schema",), help="what to show")
    p.set_defaults(fn=cmd_config)

    p = sub.add_parser("doctor", help="check the install: docker, images, simulator, login")
    p.add_argument("robot", nargs="?", default=None,
                   help="also check this robot's images and simulator")
    p.add_argument("--agent", action="append", default=None,
                   help="agent(s) the images must carry (default: the robot's config, "
                   "else the bundled default)")
    add_json(p)
    p.set_defaults(fn=cmd_doctor)
    return ap


VERBS = ("robots", "benchmarks", "agents", "build", "up", "agent", "down", "run",
         "config", "doctor")
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
    except RoboCLIError as e:
        # The one place an error becomes a message and a status: what is
        # wrong, how to fix it, and a sysexits code scripts can branch on.
        print(f"error: {e.message}", file=sys.stderr)
        if e.hint:
            print(f"hint: {e.hint}", file=sys.stderr)
        return e.exit_code


if __name__ == "__main__":
    sys.exit(main())
