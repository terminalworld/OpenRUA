"""``openrua agent``: your coding agent's own terminal, on a robot.

    openrua agent                           the configured agent
    openrua agent <name>                    a named one (openrua agents), or a manifest's path
    openrua agent <name> "open the drawer"  with an opening message

With a robot already up (``openrua up``, or an earlier ``openrua agent``
kept running) the agent opens on it: the newest, or the one ``--name``
names. Otherwise OpenRUA starts one first, the way ``openrua`` does
(Docker and the agent's sign-in set up with you, missing images built,
the saved robot unless ``--robot``, ``--sim`` or ``--bench`` say
otherwise), and when the agent exits asks whether to turn it off. This
is the agent's own interface: its Esc stops the agent, not a motion it
already sent.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from openrua import agents
from openrua.cli import docker_setup, state, workspace
from openrua.cli.commands import login, start
from openrua.config import paths, projects
from openrua.errors import UnavailableError, UsageError
from openrua.runner import history
from openrua.sessions.client import Client

STARTUP_S = 900    # a first start may build images and boot a simulator


def _is_agent(word: str) -> bool:
    if paths.is_path(word):
        return Path(word).expanduser().is_file()
    return word in {e.name for e in paths.available("agents")}


def _split(words: list[str]) -> tuple[str | None, str | None]:
    """(agent, opening message) from the positionals: the first names an
    agent when it is one, the rest is the message."""
    if words and _is_agent(words[0]):
        agent, words = words[0], words[1:]
    else:
        agent = None
    if len(words) > 1:
        raise UsageError("one opening message, quoted",
                         hint='openrua agent [agent] "the opening message"')
    return agent, (words[0] if words else None)


def _running(home: Path, project: Path | None = None) -> list[str]:
    """Robots up for the agent's own terminal, newest first: their owner
    still holds the control endpoint and the sandbox is running."""
    found = []
    for directory in (paths.home(home) / "sandboxes").glob("*"):
        control = directory / "control.json"
        if not control.is_file() or (directory / "endpoint.json").exists():
            continue
        try:
            facts = state.load(directory.name, home)
        except Exception:  # noqa: BLE001; a stopped or unreadable one is not up
            continue
        if project is not None and facts.get('project') != str(project.resolve()):
            continue
        alive = subprocess.run(["docker", "inspect", "-f", "{{.State.Running}}", facts.get("sandbox", "")],
                               capture_output=True, text=True).stdout.strip() == "true"
        if alive:
            found.append((control.stat().st_mtime, directory.name))
    return [name for _, name in sorted(found, reverse=True)]


def _argv(args, name: str, agent: str | None, prompt: str | None) -> tuple[list[str], dict]:
    st = state.load(name, args.home)
    adapter = agents.get(agent or st.get("agent_spec", st["agent"]), args.home,
                         version=None if agent else st.get("agent_version"))
    argv = adapter.interactive_argv(
        st["sandbox"], args.model or st["model"], st["proxy"],
        options=st.get("options"), limits=st.get("limits"), prompt=prompt)
    if argv is None:
        raise UnavailableError(
            f"{adapter.name} has no interactive mode",
            hint=f"open a shell on the terminal instead: docker exec -it -u robot "
                 f"-w /workspace {st['sandbox']} bash")
    return argv, st


def _start_robot(args, agent: str | None) -> tuple[str, subprocess.Popen] | None:
    """Start a robot for the agent's terminal in the background, as
    ``openrua up`` would, and wait until it is up."""
    selected = workspace.choose(getattr(args, 'workspace', None) or Path.cwd(), args.home,
                                accept=getattr(args, 'trust_workspace', False))
    if selected is None:
        return None
    docker_setup.ensure()
    name = args.name or history.new_id()
    values, _ = start.values_for(argparse.Namespace(
        home=args.home, robot=args.robot, sim=args.sim, bench=args.bench,
        agent=agent, model=args.model, name=name, workspace=str(selected)))
    if not values["robot"] and not values["bench"]:
        raise UsageError("no robot to start", hint="openrua agent --robot <robot> (openrua robots "
                         "lists them), or openrua config set --robot <robot>")
    start.prepare(args.home, values, lambda line: None if line.startswith(("Checking", "Reusing"))
                  else print(line, flush=True))
    login.ensure(args.home, values["agent"] or None, values.get("auth_mode", "native"))
    argv = [sys.executable, "-m", "openrua", "--home", str(args.home), "up"]
    argv += [values["robot"]] if values["robot"] else []
    argv += ["--sim", values["sim"], "--bench", values["bench"], "--name", name]
    argv += ['--workspace', str(selected)]
    for flag, value in (("--agent", values["agent"]), ('--model', values['model']), ("--ros-domain", args.ros_domain),
                        ("--task", args.task)):
        if value not in (None, ""):
            argv += [flag, str(value)]
    log = paths.launch_dir(name, args.home) / "up.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    print("Starting the robot; this can take a minute...", flush=True)
    with log.open("ab") as out:
        owner = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=out, stderr=subprocess.STDOUT,
                                 start_new_session=True)
    control = paths.sandbox_dir(name, args.home) / "control.json"
    shown, deadline = 0, time.monotonic() + STARTUP_S
    try:
        while not control.exists():
            lines = log.read_text(errors="replace").splitlines()
            for line in lines[shown:]:
                if line.startswith("["):
                    print(line, flush=True)
            shown = len(lines)
            if owner.poll() is not None:
                raise UnavailableError(f"the robot did not start (exit {owner.returncode})",
                                       hint=f"its output is in {log}")
            if time.monotonic() > deadline:
                raise UnavailableError("the robot is still starting", hint=f"its output is in {log}")
            time.sleep(0.5)
    except BaseException:
        if owner.poll() is None:
            owner.send_signal(signal.SIGINT)    # up powers the robot off on SIGINT
        raise
    return name, owner


def _turn_off(args, name: str, owner: subprocess.Popen) -> None:
    """Ask, as the agent exits, whether the robot it started goes off."""
    try:
        answer = input("\nTurn the robot off? [Y/n] ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        answer = "n"    # no answer leaves the robot as it is
    if answer not in ("", "y", "yes"):
        print(f"The robot keeps running: openrua agent opens the agent on it again, "
              f"openrua down --name {name} turns it off.")
        return
    control = paths.sandbox_dir(name, args.home) / "control.json"
    try:
        Client.from_file(control).end()
    except (OSError, ValueError, RuntimeError):
        owner.send_signal(signal.SIGINT)
    try:
        owner.wait(timeout=180)
    except subprocess.TimeoutExpired:
        raise UnavailableError("the robot has not finished turning off",
                               hint=f"openrua down --name {name}") from None
    print(f"The robot is off; the workspace is kept in {paths.sandbox_dir(name, args.home)}.")


def run(args) -> int:
    agent, prompt = _split(args.words)
    agent = agent or args.agent
    if args.name and (paths.sandbox_dir(args.name, args.home) / "endpoint.json").exists():
        raise UnavailableError(
            "this robot has a managed conversation; direct terminal input would bypass its queue",
            hint=f"openrua chat --name {args.name}; use a separate up session for a native terminal")
    if args.name and state.path(args.name, args.home).is_file():
        name, owner = args.name, None
    elif args.name is None and (up := _running(args.home, Path(getattr(args, 'workspace', None) or Path.cwd()))):
        name, owner = up[0], None
        if len(up) > 1:
            print(f"Opening the agent on {name}, the newest of {len(up)} robots up "
                  f"(--name picks another).", flush=True)
    else:
        if not (sys.stdin.isatty() and sys.stdout.isatty()):
            raise UsageError("the agent's terminal needs an interactive terminal")
        started = _start_robot(args, agent)
        if started is None:
            return 0
        name, owner = started
    argv, st = _argv(args, name, agent, prompt)
    if st.get("backend") == "real":
        print("This is the agent's own terminal: Esc stops the agent, not a motion it "
              "already sent. Keep the emergency stop at hand.", flush=True)
    if owner is None:
        os.execvp(argv[0], argv)
        return 0
    # Ctrl-C belongs to the agent while it runs; the robot answers to us.
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        subprocess.call(argv)
    finally:
        signal.signal(signal.SIGINT, previous)
    _turn_off(args, name, owner)
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("agent", help="your coding agent's own terminal, on a robot",
                       description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("words", nargs="*", metavar="[agent] [message]",
                   help="the agent to open (openrua agents lists them; default: the configured "
                   "one), then an opening message")
    p.add_argument("--name", default=None,
                   help="the robot's handle (default: the newest robot up, else a new one)")
    p.add_argument("--agent", default=None, help="agent, as the first word does")
    p.add_argument("--model", default=None, help="model (default: the one the robot recorded)")
    p.add_argument("--robot", default=None,
                   help="robot to start when none is up (default: the saved one)")
    p.add_argument("--sim", default=None, help="its simulator (default: the saved one)")
    p.add_argument("--bench", default=None, help="its benchmark (default: the saved one)")
    p.add_argument("--ros-domain", type=int, default=None, help="its ROS_DOMAIN_ID")
    p.add_argument("--task", default=None, help="a real robot's task sentence, for its workspace")
    p.add_argument('--workspace', default=None, help='persistent project directory (default: the current directory)')
    p.add_argument('--trust-workspace', action='store_true', help='explicitly trust this directory without a question')
    p.set_defaults(fn=run)
