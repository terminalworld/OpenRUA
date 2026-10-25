"""Create interactive robot resources shared by command-line and service hosts.

The caller owns the returned handle and closes it explicitly. This layer
uses the same bring-up path as trials but owns no benchmark scoring or
agent conversation loop.
"""

from __future__ import annotations

import json
import os
import subprocess
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable

from openrua import agents
from openrua.config import apply_suite_overrides, compose, normalize_arms, paths, projects
from openrua.config.schema import Limits, sandbox_image
from openrua.errors import UnavailableError, UsageError
from openrua.proxy.up import ensure as ensure_proxy
from openrua.robot import not_ready
from openrua.robot.cameras import camera_topics
from openrua.runner import live_state as state
from openrua.runner.motion import cancel_motion, motion_actions
from openrua.runner.record import session_provenance
from openrua.runner.bringup import (agent_proxy_url, bring_up, ensure_internal_network,
                                    internet_access, start_episode, prepare_sandbox)
from openrua.sandbox.down import down as sandbox_down
from openrua.sandbox import project


@dataclass(frozen=True)
class RobotRequest:
    home: Path
    name: str = state.DEFAULT_NAME
    robot: str | None = None
    simulator: str | None = None
    benchmark: str | None = None
    agent: str | None = None
    model: str | None = None
    task_suite: str | None = None
    task_id: int | None = None
    task: str | None = None
    init_state: int = 0
    workspace: Path | None = None
    project: Path | None = None
    ros_domain: int | None = None
    # What the agent may reach, over a free session's defaults (nothing
    # limited): config.schema.Limits field -> value.
    limits: dict | None = None
    # Whether the agent asks before each command that changes something
    # ("ask") or runs it ("auto"); /permissions switches it later.
    permissions: str = "auto"
    # The handle already exists with the agent's profile in it: a robot
    # just connected takes over the session it was connected in, and the
    # conversation goes on (its native record lives in that profile).
    continuing: bool = False


@dataclass
class LiveRobot:
    """Resource facts and an explicit shutdown operation, without UI state."""

    name: str
    sim: str
    sandbox: str
    robot: str            # the robot as named on the command line (or by the benchmark)
    robot_model: str      # machine.robot.model
    backend: str          # "simulated by robosuite (ROS 2 jazzy)" or "real"
    benchmark: str | None
    suite: str | None
    task_id: int | None
    task: str             # the scene's own task sentence, if any
    agent: str
    model: str
    power_off: Callable[[], None]
    # Why the robot's resources ended on their own, once they have; the
    # owner of a long-lived session polls it.
    stopped: Callable[[], str | None] = lambda: None
    robot_log: Path | None = None
    # (camera name -> color topic, topic pattern): what a person watching sees
    cameras: tuple = ({}, None)
    # Cancel every goal on the robot's motion actions: what an interrupt
    # does to the robot. Returns each action's outcome.
    stop_motion: Callable[[], dict[str, bool]] = lambda: {}
    # Interactive owners maintain this connection independently of the sandbox.
    connection: object | None = None


def check_agent_cli(sandbox: str, adapter, backend: dict, robot: str) -> str | None:
    """The sandbox's agent CLI version (None if the CLI cannot say).

    Refuses a sandbox whose agent CLI is not the pinned version, or is
    older than every flag the adapter passes needs, before the agent
    starts: an old CLI otherwise exits on the first unknown flag and the
    session shows only a disconnected agent. The error says how to
    rebuild the sandbox."""
    found = None
    if adapter.version_probe():
        found = subprocess.run(["docker", "exec", sandbox, "bash", "-c", adapter.version_probe()],
                               capture_output=True, text=True, timeout=60).stdout.strip() or None
    check = adapter.sandbox_cli_check()
    if not check:
        return found
    r = subprocess.run(["docker", "exec", sandbox, "bash", "-c", check[1]],
                       capture_output=True, text=True, timeout=60)
    if r.returncode == 0:
        return found
    want = (f"version {adapter.version}" if adapter.version else
            f"{adapter.min_version} or newer" if adapter.min_version else 'an installed, working CLI')
    pin = f"@{adapter.version}" if adapter.version else ""
    if backend.get("sandbox_install"):
        build = f"openrua build sandbox --robot {robot} --agent {adapter.name}{pin}"
    else:
        image, distro = backend["sandbox_image"], backend.get("ros_distro", "jazzy")
        tag = "" if image == sandbox_image(distro) else f" --tag {image}"
        build = f"openrua build sandbox --distro {distro}{tag} --agent {adapter.name}{pin}"
    raise UnavailableError(
        f"the sandbox's {adapter.name} is {found or 'unreadable'}; this session needs {want}",
        hint=build)


def not_ready_message(unready: list[dict]) -> str:
    return "the robot is not ready to start: " + "; ".join(c["name"] for c in unready)


def not_ready_hint(unready: list[dict], backend: dict) -> str:
    """Each failed check's fix, then where to go on if the robot itself changed."""
    fixes = "\n".join(f"{c['name']}: {c['fix']}" for c in unready)
    where = backend.get("cwd")
    return fixes + (f"\nif the robot itself changed: openrua connect --dir {where}" if where else "")


def _robot_file(robot: str) -> Path | None:
    """The robot's config file; None for a benchmark's inline machine."""
    try:
        return paths.find("robots", robot).resolve()
    except Exception:  # noqa: BLE001
        return None


def stop_resources(sandbox: str, shutdown_machine: Callable[[], None]) -> None:
    """Attempt both shutdowns and report failures without declaring success."""
    failures = []
    for label, stop in (("sandbox", lambda: sandbox_down(sandbox)),
                        ("robot", shutdown_machine)):
        try:
            stop()
        except Exception as exc:
            failures.append((label, exc))
    if failures:
        detail = "; ".join(f"{label}: {exc}" for label, exc in failures)
        raise UnavailableError(
            f"resource shutdown incomplete ({detail})",
            hint="retry resource shutdown before deleting session files") from failures[0][1]


def open_robot(request: RobotRequest, *, interactive: bool = False, resolved=None) -> LiveRobot:
    """Bring up resources under a new handle and retain files after shutdown."""
    composed = resolved or compose(request.robot, request.simulator, request.benchmark, request.home, agent=request.agent)
    cfg = composed.cfg
    suite = request.task_suite or composed.suite
    task_id = request.task_id if request.task_id is not None else composed.task_id
    if suite is not None:
        apply_suite_overrides(cfg, suite)
    normalize_arms(cfg)
    unready = [] if interactive else not_ready(cfg["machine"]["backend"])
    if unready:
        raise UnavailableError(not_ready_message(unready),
                               hint=not_ready_hint(unready, cfg["machine"]["backend"]))
    previous = state.load(request.name, request.home, require_running=False) \
        if request.continuing else None
    cfg_agent = cfg["agent"]
    adapter = agents.get(cfg_agent["name"], request.home,
                         version=previous.get('agent_version') if previous else cfg_agent.get("version"))
    permission_modes = getattr(adapter, 'permission_modes', agents.Agent.permission_modes)
    if request.permissions not in permission_modes:
        raise UsageError(f'{adapter.name} does not support permission mode {request.permissions!r}',
                         hint='choose one of: ' + ', '.join(permission_modes))
    key_file = None
    if previous is None:
        try:
            key_file = agents.api_key_file(adapter, cfg_agent.get("auth"))
        except ValueError as exc:
            raise UsageError(str(exc), hint="check the selected API key file, or use openrua config set --auth native") from exc
    sim_name, sandbox_name = state.container_names(request.name)
    sandbox_dir = paths.sandbox_dir(request.name, request.home)
    workdir = Path(request.workspace or sandbox_dir / "workspace").resolve()
    if request.project is None and workdir.exists() and (not workdir.is_dir() or any(workdir.iterdir())):
        raise UsageError(f"working directory is not empty: {workdir}",
                         hint="use --workspace <new-empty-directory>; existing files are retained")
    if not request.continuing:
        state.reserve(request.name, request.home)
    if request.project is None:
        workdir.mkdir(parents=True, exist_ok=True)
    limits = Limits(**(request.limits or {})).model_dump()
    network = ensure_internal_network()
    # A proxy only when the agent may reach no more than its model
    # service; otherwise the sandbox goes out directly.
    proxy_url = (ensure_proxy(network, "\n".join(adapter.whitelist))
                 if limits["internet"] == "model-only" else "")
    # cfg["agent"] is already the chosen agent's (compose took --agent):
    # its model, version, login directory and options are that agent's.
    model = request.model or cfg_agent.get("model") or adapter.default_model
    if previous is not None:
        # Prepared already; preparing again would replace it, and with it
        # the conversation's native record.
        mounts = tuple(previous["mounts"])
    elif key_file is not None:
        print(f"[up] {adapter.name}: explicitly selected API authentication", flush=True)
        profile = adapter.prepare_api_profile(key_file, sandbox_dir / "profile", limits=limits)
    else:
        creds_home = agents.resolve_profile(adapter, cfg_agent.get('credentials_dir'),
                                            paths.credentials_dir(request.home) / adapter.name,
                                            user_home=Path.home(), environment=os.environ, link=True)
        print(f"[up] {adapter.name} native login: {creds_home} (source: {creds_home.resolve()})", flush=True)
        profile = adapter.prepare_profile(creds_home, sandbox_dir / "profile", limits=limits)
    if previous is None:
        mounts = profile.mounts
    project_root = request.project.expanduser().resolve() if request.project is not None else None
    if project_root is not None:
        materials = sandbox_dir / "robot-materials"
        project.prepare(cfg, project_root, materials)
        from openrua.runner.skills import prepare as prepare_skills
        skill_mounts = prepare_skills(project_root, request.home, adapter, materials)
        workdir = project.run_directory(project_root, request.name)
        mounts = (*(m for m in mounts if not m.endswith(":/robot:ro")), f"{materials}:/robot:ro")
        if skill_mounts:
            destinations = {mount.split(':')[1] for mount in skill_mounts}
            mounts = (*(mount for mount in mounts if mount.split(':')[1] not in destinations), *skill_mounts)
    if interactive:
        from openrua.runner.robot_client import prepare_client, guide
        service = sandbox_dir / 'maintenance'
        prepare_client(service)
        mounts = (*(m for m in mounts if m.split(':')[1] not in {'/openrua-runtime', '/adapter'}),
                  f'{service}:/openrua-runtime:ro')
        source = _robot_file(composed.robot)
        if source is not None and not source.is_relative_to(paths.bundled('robots')):
            mounts = (*mounts, f'{source.parent}:/adapter')
            with (service / 'README.md').open('a') as stream:
                stream.write(f'\nMaintained robot configuration: /adapter/{source.name}\n')
    if cfg["machine"]["backend"]["kind"] == "sim":
        print(f"[up] sandbox {sandbox_name}; robot {sim_name} (booting; MoveIt takes a minute)",
              flush=True)
    else:
        print(f"[up] sandbox {sandbox_name}; waiting for the robot's ROS 2 graph", flush=True)
    connection = None
    try:
        if interactive:
            from openrua.runner.connection import Connection
            config_path, domain = prepare_sandbox(
                cfg, workdir, sim_name, sandbox_name, network, internet_access(proxy_url),
                mounts, request.ros_domain,
                **({"workspace": project_root, "seed_workspace": False} if project_root is not None else {}))
            chosen = cfg_agent['name'] + ('@' + adapter.version if adapter.version else '')
            connection = Connection(replace(request, agent=chosen, model=model, project=project_root,
                                            robot=None if composed.robot.startswith('(') else composed.robot,
                                            simulator=composed.simulator or '',
                                            benchmark=composed.source if composed.benchmark else ''),
                                    composed, workdir, config_path, domain, network, sandbox_name)
            guide(materials if project_root is not None else workdir / 'workspace')
            machine = connection
        else:
            _, machine, _ = bring_up(
                cfg, workdir, sim_name, sandbox_name, suite, task_id, network,
                internet_access(proxy_url), mounts, request.ros_domain,
                robot_log=workdir / "robot.log", home=request.home,
                **({"workspace": project_root, "seed_workspace": False} if project_root is not None else {}))
    except Exception as e:
        if interactive:
            sandbox_down(sandbox_name)
        component = 'environment' if interactive else 'robot'
        raise UnavailableError(f"[up] {component} failed to come up: {e}",
                               hint=f"read {workdir / 'robot.log'}") from e

    try:
        cli_version = check_agent_cli(sandbox_name, adapter, cfg["machine"]["backend"],
                                      composed.robot)
        (sandbox_dir / "provenance.json").write_text(json.dumps(
            session_provenance(_robot_file(composed.robot), cfg, paths.code_root(),
                               agent_cli={"name": adapter.name, "version": cli_version},
                               limits=limits),
            indent=2) + "\n")
    except BaseException:
        stop_resources(sandbox_name, machine.shutdown)
        raise

    def power_off() -> None:
        print("\n[down] powering off", flush=True)
        stop_resources(sandbox_name, machine.shutdown)
        state.stopped(request.name, request.home)
        print(f"[down] files retained at {sandbox_dir}; workspace: {project_root or workdir / 'workspace'}")

    robot = cfg["machine"].get("robot", {})
    try:
        if not interactive and cfg["machine"]["backend"]["kind"] == "sim":
            task = start_episode(machine, request.init_state).get("language", "")
        else:
            task = request.task or ""
        state.save(request.name, request.home, status="running", sim=sim_name, sandbox=sandbox_name,
                   backend=cfg["machine"]["backend"]["kind"], interactive=interactive,
                   network=network,
                   proxy=agent_proxy_url(cfg["machine"]["backend"], proxy_url),
                   limits=limits, permissions=request.permissions, mounts=list(mounts),
                   agent=adapter.name, agent_title=adapter.title or adapter.name,
                   agent_spec=cfg_agent["name"], agent_version=agents.version_from_output(cli_version)
                       or (previous or {}).get('agent_version') or cfg_agent.get('version'),
                   model=model,
                   auth_mode=previous.get('auth_mode', 'native') if previous else "api" if key_file is not None else "native",
                   options={**adapter.default_options,
                            **cfg.get("agent", {}).get("options", {})},
                   workspace=str(project_root or workdir / "workspace"), task=task,
                   **({'configuration': '/adapter/' + source.name} if interactive and source is not None
                       and not source.is_relative_to(paths.bundled('robots')) else {}),
                   **({"project": str(project_root), "run_directory": str(workdir)} if project_root is not None else {}),
                   # What the terminal shows in its header; the resolved config is
                   # the source and these are its names, not a second copy of facts.
                   robot=composed.robot, robot_model=robot.get("model"),
                   simulator=composed.simulator, benchmark=composed.source if composed.benchmark else '',
                   suite=suite, task_id=task_id, init_state=request.init_state)
    except BaseException:
        stop_resources(sandbox_name, machine.shutdown)
        raise
    backend = cfg["machine"]["backend"]
    where = (f"simulated by {composed.simulator} (ROS 2 {backend.get('ros_distro', '?')})"
             if backend["kind"] == "sim" else "real")
    return LiveRobot(
        name=request.name, sim=sim_name, sandbox=sandbox_name,
        robot=composed.robot, robot_model=robot.get("model", "?"), backend=where,
        benchmark=composed.benchmark,
        suite=suite, task_id=task_id, task=task,
        agent=adapter.name, model=model, power_off=power_off,
        stopped=getattr(machine, "stopped", lambda: None), robot_log=workdir / "robot.log",
        cameras=camera_topics(cfg),
        stop_motion=(connection.stop_motion if connection else
                     lambda actions=motion_actions(cfg["machine"]): cancel_motion(sandbox_name, actions)),
        connection=connection)
