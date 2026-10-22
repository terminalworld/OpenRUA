"""Connect a real robot: a session whose terminal can build and start drivers.

``openrua connect`` opens a session like any other, except for what its
sandbox reaches: the host's docker, network and devices, with the
robot's adaptation folder as its workspace. The agent writes the folder
(robot.yaml, driver/, cameras/) with the person at the robot; each step
is checked here, on the host, when the agent asks (``check.py <step>``
in the sandbox, through the session's local service). When every check
has passed, the session moves to the robot's own terminal and the
conversation goes on there.
"""

from __future__ import annotations

import asyncio
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import uuid
from dataclasses import dataclass
from pathlib import Path

import yaml

from openrua import agents
from openrua.agents.registry import split_pin
from openrua.config import compose, paths, projects
from openrua.config.loader import default_agent_name, layer_agent, load_user_config
from openrua.config.schema import Limits, RobotInstance, sandbox_image
from openrua.errors import UnavailableError, UsageError
from openrua.runner import live_state as state
from openrua.runner.bringup import host_network_dds
from openrua.runner.live import LiveRobot, RobotRequest, check_agent_cli, open_robot
from openrua.runner.preflight import run_preflight
from openrua.sandbox.down import down as sandbox_down
from openrua.sandbox.up import up as sandbox_up

STEPS = ("host", "drivers", "probe", "config", "robot")
TEMPLATES = Path(__file__).resolve().parents[1] / "configs" / "connect"
CHECKS_DIR = ".openrua-checks"
DISK_MIN_GB = 30


@dataclass(frozen=True)
class ConnectRequest:
    home: Path
    name: str
    folder: Path
    agent: str | None = None
    model: str | None = None
    distro: str = "jazzy"   # the connecting terminal's ROS 2 (the robot's comes later)
    project: Path | None = None


def robot_file(folder: Path) -> Path:
    return folder / "robot.yaml"


def scaffold(folder: Path) -> None:
    """The folder's starting files; what is there already stays."""
    folder.mkdir(parents=True, exist_ok=True)
    for sub in ("driver", "cameras", CHECKS_DIR):
        (folder / sub).mkdir(exist_ok=True)
    if not robot_file(folder).exists():
        shutil.copyfile(TEMPLATES / "robot.yaml", robot_file(folder))
    ignore = folder / ".gitignore"
    lines = ignore.read_text().splitlines() if ignore.exists() else []
    if f"{CHECKS_DIR}/" not in lines:
        ignore.write_text("\n".join(lines + [f"{CHECKS_DIR}/"]) + "\n")
    (folder / "CONNECT.md").write_text(
        (TEMPLATES / "CONNECT.md").read_text().replace("{folder}", str(folder)))


def _run(argv: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(argv, capture_output=True, text=True, timeout=60)


def host_report() -> tuple[bool, str]:
    """This computer, as connecting a robot needs it; failures come with
    their fix."""
    ok, lines = True, []
    info = _run(["docker", "info", "--format", "{{.DockerRootDir}}"])
    if info.returncode:
        ok = False
        lines.append("docker: this account cannot use it ("
                     + (info.stderr.strip().splitlines() or ["no answer"])[-1] + "). Fix: "
                     "sudo usermod -aG docker $USER, then log out and in again")
    else:
        root = info.stdout.strip() or "/var/lib/docker"
        try:
            free = shutil.disk_usage(root).free / 1e9
        except OSError:
            free = shutil.disk_usage("/").free / 1e9
        mark = "" if free >= DISK_MIN_GB else f" (low: driver images often need {DISK_MIN_GB} GB)"
        lines.append(f"docker: usable; {free:.0f} GB free where it keeps images{mark}")
    rt = _run(["bash", "-c", "ulimit -r"]).stdout.strip()
    lines.append(f"real-time priority limit (ulimit -r): {rt or 'unknown'}"
                 + ("" if rt in ("99", "unlimited") else
                    " (a 1 kHz arm controller run here needs 99: a group with rtprio 99 in "
                    "/etc/security/limits.d, then a full new login)"))
    lines.append(f"groups: {_run(['id', '-nG']).stdout.strip()}")
    serial = sorted(p.name for p in Path("/dev/serial/by-id").glob("*")) \
        if Path("/dev/serial/by-id").is_dir() else []
    video = sorted(p.name for p in Path("/dev").glob("video*"))
    lines.append(f"serial devices: {', '.join(serial) or 'none'}")
    lines.append(f"video devices: {', '.join(video) or 'none'}")
    return ok, "\n".join(lines)


def _agent(request: ConnectRequest):
    """The agent, its machine facts and its model, as a session would pick them."""
    package = load_user_config(paths.package_config_path())
    user = load_user_config(paths.config_path(request.home))
    name, version = split_pin(request.agent or default_agent_name(package, user) or "")
    if not name:
        raise UsageError("name an agent", hint="openrua connect --agent <name> (openrua agents lists them)")
    selected = layer_agent({}, package, user, agent=name)
    selected["version"] = version or selected.get("version")
    adapter = agents.get(name, request.home, version=selected["version"])
    return adapter, selected, request.model or selected.get("model") or adapter.default_model, user


def _service_files(directory: Path) -> None:
    """What the sandbox reads from OpenRUA: the check command, the robot
    config's schema and the bundled robot types as examples."""
    directory.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(TEMPLATES / "check.py", directory / "check.py")
    (directory / "robot-schema.json").write_text(
        json.dumps(RobotInstance.model_json_schema(), indent=1))
    examples = directory / "examples"
    examples.mkdir(exist_ok=True)
    for path in paths.bundled("robots").glob("*.yaml"):
        shutil.copyfile(path, examples / path.name)


def open_connect(request: ConnectRequest, continuing: bool = False) -> LiveRobot:
    """The connecting sandbox and its handle; nothing about a robot is
    known yet, so no robot is started. ``continuing`` reopens it for a
    session that already has its handle and profile (a move to the
    robot's own terminal that did not work out)."""
    folder = request.folder.expanduser().resolve()
    scaffold(folder)
    adapter, selected, model, user = _agent(request)
    try:
        key_file = agents.api_key_file(adapter, selected.get("auth"))
    except ValueError as exc:
        raise UsageError(str(exc)) from exc
    socket = Path("/var/run/docker.sock")
    docker_cli = shutil.which("docker")
    if not socket.exists() or not docker_cli:
        raise UnavailableError("connecting a robot needs docker on this computer",
                               hint="install docker and add this account to its group")
    session_dir = paths.sandbox_dir(request.name, request.home)
    limits = Limits().model_dump()
    if continuing:
        # Prepared already: preparing again would replace the profile, and
        # with it the conversation's native record.
        profile_mounts = tuple(state.load(request.name, request.home,
                                          require_running=False)["mounts"])
    else:
        state.reserve(request.name, request.home)
        if key_file is not None:
            profile = adapter.prepare_api_profile(key_file, session_dir / "profile", limits=limits)
        else:
            creds_home = agents.resolve_profile(adapter, selected.get("credentials_dir"),
                                                paths.credentials_dir(request.home) / adapter.name,
                                                user_home=Path.home(), environment=os.environ,
                                                link=True)
            profile = adapter.prepare_profile(creds_home, session_dir / "profile", limits=limits)
        profile_mounts = profile.mounts
    service = session_dir / "connect"
    _service_files(service)
    _, sandbox_name = state.container_names(request.name)
    dds_profile, dds_env = host_network_dds(request.distro, [])
    print(f"[connect] terminal {sandbox_name}, working in {folder}", flush=True)
    sandbox_up({"machine": {"workspace_template": None}}, folder,
               image=sandbox_image(request.distro), network="host", internet="open",
               name=sandbox_name, seed_workspace=False, ros_domain=0,
               peers_xml=dds_profile, env=dds_env,
               mounts=(*profile_mounts, f"{folder}:{folder}",
                       f"{socket}:{socket}", f"{Path(docker_cli).resolve()}:/usr/local/bin/docker:ro",
                       "/dev:/host/dev:ro", f"{service}:/openrua-connect:ro"),
               run_args=("--group-add", str(socket.stat().st_gid),
                         *user.sandbox.run_args))
    try:
        check_agent_cli(sandbox_name, adapter, {"sandbox_image": sandbox_image(request.distro),
                                                "ros_distro": request.distro}, "")
    except BaseException:
        sandbox_down(sandbox_name)
        raise
    state.save(request.name, request.home, status="running", sim="", sandbox=sandbox_name,
               backend="connect", network="host", proxy="", agent=adapter.name,
               agent_title=adapter.title or adapter.name,
               agent_spec=adapter.name, agent_version=selected.get("version"), model=model,
               auth_mode="api" if key_file is not None else "native",
               options={**adapter.default_options, **(selected.get("options") or {})},
               workspace=str(folder), task="", limits=limits, permissions="ask",
               mounts=list(profile_mounts), robot=str(robot_file(folder)),
               robot_model="being connected", connect={"folder": str(folder)})
    if request.project is not None:
        facts = state.load(request.name, request.home)
        state.save(request.name, request.home, **{**facts, 'project': str(request.project.resolve())})

    def power_off() -> None:
        sandbox_down(sandbox_name)
        state.stopped(request.name, request.home)

    return LiveRobot(name=request.name, sim="", sandbox=sandbox_name,
                     robot=str(robot_file(folder)), robot_model="being connected",
                     backend="connecting a robot", benchmark=None, suite=None, task_id=None,
                     task="", agent=adapter.name, model=model, power_off=power_off)


def opening_message(folder: Path, host: str) -> str:
    """The first message the agent gets: where it is and what it is for."""
    return (f"We are connecting a new robot to OpenRUA. Its folder is /workspace here "
            f"({folder} on the host). CONNECT.md in it describes the steps, the files to "
            "write and how each step is checked. This computer, as OpenRUA found it:\n" + host +
            "\nStart with what the robot is: ask me what you cannot find out yourself.")


class Checks:
    """Each step's check, on the host, one at a time; results go to the
    session's state (``checks``) and to ``<folder>/.openrua-checks``."""

    def __init__(self, request: ConnectRequest, session, agent: str):
        self.request, self.session, self.agent = request, session, agent
        self.folder = request.folder.expanduser().resolve()
        self._busy = threading.Lock()

    def log(self, step: str) -> Path:
        return self.folder / CHECKS_DIR / f"{step}.log"

    def passed(self) -> list[str]:
        checks = self.session.store.snapshot()["state"].get("checks", {})
        return [step for step in STEPS if checks.get(step, {}).get("status") == "passed"]

    def start(self, step: str) -> dict:
        """Start ``step`` in a thread; returns at once with its run ID.
        ``done`` answers at once: every check has passed, or which have not."""
        if step == "done":
            missing = [s for s in STEPS if s not in self.passed()]
            if missing:
                return {"status": "failed", "summary": "not passed yet: " + ", ".join(missing)}
            return {"status": "passed", "summary": "every check has passed; the session moves "
                    "to the robot's own terminal when this turn ends"}
        if step not in STEPS:
            raise ValueError(f"unknown step {step!r}; steps: {', '.join(STEPS + ('done',))}")
        if not self._busy.acquire(blocking=False):
            raise RuntimeError("another check is running; wait for it")
        run = uuid.uuid4().hex
        self.session.checked(step, "running", "", run)

        def work():
            try:
                with self.log(step).open("w") as out:
                    ok, summary = getattr(self, "_" + step)(out)
            except Exception as exc:  # noqa: BLE001; the check's own failure is its result
                ok, summary = False, f"{type(exc).__name__}: {exc}"
            finally:
                self._busy.release()
            self.session.checked(step, "passed" if ok else "failed", summary, run)
        threading.Thread(target=work, daemon=True).start()
        return {"status": "running", "run": run}

    def _openrua(self, out, *argv: str) -> int:
        """An openrua command, its output into the check's log."""
        return subprocess.call([sys.executable, "-m", "openrua", "--home",
                                str(self.request.home), *argv], stdout=out, stderr=subprocess.STDOUT)

    def _host(self, out) -> tuple[bool, str]:
        ok, text = host_report()
        out.write(text + "\n")
        return ok, text

    def _config(self, out) -> tuple[bool, str]:
        todo = [f"{p.relative_to(self.folder)}:{i}: {line.strip()}"
                for p in [robot_file(self.folder), *sorted((self.folder / "cameras").glob("*.yaml"))]
                for i, line in enumerate(p.read_text().splitlines(), 1)
                if "TODO" in line.split("#", 1)[0]]
        if todo:
            out.write("\n".join(todo) + "\n")
            return False, f"{len(todo)} TODO left:\n" + "\n".join(todo[:20])
        composed = compose(str(robot_file(self.folder)), "", "", self.request.home)
        machine = composed.cfg["machine"]
        arms = machine.get("arms") or ([machine["arm"]] if machine.get("arm") else [])
        text = (f"loads: {machine.get('robot', {}).get('model')}, {len(arms)} arm(s); "
                f"launch: {machine['backend'].get('launch')}")
        out.write(text + "\n")
        return True, text

    def _drivers(self, out) -> tuple[bool, str]:
        data = yaml.safe_load(robot_file(self.folder).read_text()) or {}
        images = (data.get("machine", {}).get("backend", {}) or {}).get("build") or {}
        if not images:
            return False, "robot.yaml builds no image yet (backend.build: image: directory)"
        failed = [image for image in images
                  if self._openrua(out, "build", "driver", "--robot", str(robot_file(self.folder)),
                                   "--image", image)]
        if failed:
            return False, f"did not build: {', '.join(failed)}; the end of the log says why"
        return True, f"built: {', '.join(images)}"

    def _probe(self, out) -> tuple[bool, str]:
        data = yaml.safe_load(robot_file(self.folder).read_text()) or {}
        distro = (data.get("machine", {}).get("backend", {}) or {}).get("ros_distro")
        distro = distro if distro in ("humble", "jazzy") else self.request.distro
        draft = self.folder / "robot.draft.yaml"
        with tempfile.TemporaryFile("w+") as drafted:
            code = subprocess.call([sys.executable, "-m", "openrua", "--home", str(self.request.home),
                                    "probe", "--host", "--distro", distro],
                                   stdout=drafted, stderr=out)
            drafted.seek(0)
            text = drafted.read()
        if code or not text.strip():
            return False, "the probe read no graph; are the drivers running? (the log says more)"
        draft.write_text(text)
        return True, f"wrote robot.draft.yaml ({len(text.splitlines())} lines, ROS 2 {distro})"

    def _images(self, out, backend: dict) -> list[str]:
        """Build what this robot needs and lacks: its driver images and its
        sandbox (its own when it installs packages into it). Returns what
        did not build."""
        def missing(image: str) -> bool:
            return subprocess.run(["docker", "image", "inspect", image],
                                  capture_output=True).returncode != 0
        file, failed = str(robot_file(self.folder)), []
        for image in backend.get("build") or {}:
            if missing(image) and self._openrua(out, "build", "driver", "--robot", file,
                                                "--image", image):
                failed.append(image)
        sandbox = backend["sandbox_image"]
        if missing(sandbox):
            build = (["build", "sandbox", "--robot", file] if backend.get("sandbox_install") else
                     ["build", "sandbox", "--distro", backend["ros_distro"], "--tag", sandbox])
            if self._openrua(out, *build, "--agent", self.agent):
                failed.append(sandbox)
        return failed

    def _robot(self, out) -> tuple[bool, str]:
        file = str(robot_file(self.folder))
        backend = compose(file, "", "", self.request.home).cfg["machine"]["backend"]
        failed = self._images(out, backend)
        if failed:
            return False, f"did not build: {', '.join(failed)}; the end of the log says why"
        notes = []
        for attempt in (1, 2):
            name = f"{self.request.name}-check-{attempt}-{uuid.uuid4().hex[:6]}"
            with tempfile.TemporaryDirectory() as scratch:
                workspace = Path(scratch) / "w"
                try:
                    live = open_robot(RobotRequest(home=self.request.home, name=name, robot=file,
                                                   agent=self.agent, workspace=workspace))
                except Exception as exc:
                    _append(out, workspace / "robot.log")
                    shutil.rmtree(paths.sandbox_dir(name, self.request.home), ignore_errors=True)
                    return False, f"start {attempt}: did not come up: {exc}"
                try:
                    if attempt == 1:
                        cfg = yaml.safe_load((workspace / "config.yaml").read_text())
                        verdict = run_preflight(cfg, live.sandbox, agents.get(self.agent))
                        out.write(json.dumps(verdict, indent=1) + "\n")
                        if not verdict["ok"]:
                            return False, ("came up, but the graph does not match robot.yaml: "
                                           + ", ".join(verdict["failed"]) + " (the log has each check)")
                    notes.append(f"start {attempt}: up")
                finally:
                    try:
                        live.power_off()
                    finally:
                        _append(out, workspace / "robot.log")
                        shutil.rmtree(paths.sandbox_dir(name, self.request.home), ignore_errors=True)
            notes.append(f"stop {attempt}: clean")
        return True, "; ".join(notes) + "; robot.yaml matches the live graph"


def _append(out, log: Path) -> None:
    """A bring-up's robot log, into the check's own (its workspace goes away)."""
    if log.is_file():
        out.write(f"--- {log.name} ---\n" + log.read_text(errors="replace")[-20000:] + "\n")


FIRST_MOTION = (
    "The robot is up in its own terminal now, started from robot.yaml, which is "
    "the default robot from here on (openrua starts it). This terminal has no "
    "docker; read the workspace instructions for the current robot's machine "
    "manifest, docs and tools. Last step: "
    "with the person's word that the emergency stop is at hand, open and close the "
    "gripper once and move one joint a few degrees, then say how it went.")


class ConnectFlow:
    """The session side of connecting: answers check.py, opens the
    conversation, and moves it to the robot's own terminal once every
    check has passed and the turn that asked has ended."""

    def __init__(self, owned, request: ConnectRequest):
        self.owned, self.request = owned, request
        self.folder = request.folder.expanduser().resolve()
        facts = state.load(request.name, request.home)
        # The same agent, at the same version, for the robot it connects.
        self.agent = facts["agent_spec"] + (f"@{facts['agent_version']}"
                                            if facts.get("agent_version") else "")
        self.checks = Checks(request, owned.execution.session, self.agent)
        self.server = None
        self._move = False
        self._task = None

    async def check(self, body: dict) -> dict:
        """POST /api/actions/check {"step": ...}."""
        result = self.checks.start(str(body.get("step", "")))
        if body.get("step") == "done" and result["status"] == "passed":
            self._move = True
        return result

    async def begin(self, server) -> None:
        """Tell the sandbox where to ask, check this computer, and open
        the conversation."""
        self.server = server
        endpoint = paths.sandbox_dir(self.request.name, self.request.home) / "connect" / "endpoint.json"
        endpoint.write_text(json.dumps({"url": server.url, "token": server.token}))
        endpoint.chmod(0o600)
        ok, text = host_report()
        (self.folder / CHECKS_DIR / "host.log").write_text(text + "\n")
        self.owned.execution.session.checked("host", "passed" if ok else "failed", text,
                                              uuid.uuid4().hex)
        await self.owned.execution.command("enqueue", client_id="openrua",
                                           request_id=uuid.uuid4().hex,
                                           text=opening_message(self.folder, text))
        self._task = asyncio.create_task(self._watch())

    async def close(self) -> None:
        if self._task is not None:
            self._task.cancel()

    async def _say(self, text: str) -> None:
        await self.owned.execution.command("enqueue", client_id="openrua",
                                           request_id=uuid.uuid4().hex, text=text)

    async def _watch(self) -> None:
        while True:
            await asyncio.sleep(1)
            if not self._move or self.owned.store.snapshot()["state"]["active"]:
                continue
            self._move = False
            await self._move_to_robot()

    async def _move_to_robot(self) -> None:
        file = str(robot_file(self.folder))
        settings = self.owned.store.snapshot()["state"].get("settings") or {}
        request = RobotRequest(home=self.request.home, name=self.request.name, robot=file,
                               agent=self.agent, model=settings.get("model"), continuing=True,
                               permissions=settings.get("permissions", "ask"), project=self.request.project)
        print("[connect] every check passed: moving to the robot's own terminal", flush=True)
        try:
            await self.owned.move_to(lambda: open_robot(request),
                                     "the robot is connected: moving to its own terminal")
        except Exception as exc:  # noqa: BLE001; go back to connecting and say why
            print(f"[connect] moving failed: {exc}", flush=True)
            await self.owned.move_to(lambda: open_connect(self.request, continuing=True),
                                     "moving to the robot's terminal failed")
            self.server.execution = self.owned.execution
            await self._say(f"Moving to the robot's own terminal did not work: {exc}. This is "
                            "the connecting terminal again; the robot's log from that start is "
                            f"in the service log ({paths.launch_dir(self.request.name, self.request.home)}).")
            return
        self.server.execution = self.owned.execution
        subprocess.call([sys.executable, "-m", "openrua", "--home", str(self.request.home),
                         "config", "set", "--robot", file], stdout=subprocess.DEVNULL)
        if self.request.project is not None:
            projects.save_selection(self.request.project, dict(robot=file, sim='', bench='', agent=self.agent,
                                                               model=settings.get('model') or ''))
        await self._say(FIRST_MOTION)
