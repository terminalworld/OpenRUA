"""Create interactive robot resources shared by command-line and service hosts.

The caller owns the returned handle and closes it explicitly. This layer
uses the same bring-up path as trials but owns no benchmark scoring or
agent conversation loop.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from openrua import agents
from openrua.config import apply_suite_overrides, compose, normalize_arms, paths
from openrua.errors import UnavailableError, UsageError
from openrua.proxy.up import ensure as ensure_proxy
from openrua.runner import live_state as state
from openrua.runner.bringup import bring_up, ensure_internal_network, start_episode
from openrua.sandbox.down import down as sandbox_down


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
    ros_domain: int | None = None


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


def open_robot(request: RobotRequest) -> LiveRobot:
    """Bring up resources under a new handle and retain files after shutdown."""
    composed = compose(request.robot, request.simulator, request.benchmark, request.home, agent=request.agent)
    cfg = composed.cfg
    suite = request.task_suite or composed.suite
    task_id = request.task_id if request.task_id is not None else composed.task_id
    if suite is not None:
        apply_suite_overrides(cfg, suite)
    normalize_arms(cfg)
    cfg_agent = cfg["agent"]
    adapter = agents.get(cfg_agent["name"], request.home, version=cfg_agent.get("version"))
    try:
        key_file = agents.api_key_file(adapter, cfg_agent.get("auth"))
    except ValueError as exc:
        raise UsageError(str(exc), hint="check the selected API key file, or use openrua config set --auth native") from exc
    sim_name, sandbox_name = state.container_names(request.name)
    sandbox_dir = paths.sandbox_dir(request.name, request.home)
    workdir = Path(request.workspace or sandbox_dir / "workspace").resolve()
    if workdir.exists() and (not workdir.is_dir() or any(workdir.iterdir())):
        raise UsageError(f"working directory is not empty: {workdir}",
                         hint="use --workspace <new-empty-directory>; existing files are retained")
    state.reserve(request.name, request.home)
    workdir.mkdir(parents=True, exist_ok=True)
    network = ensure_internal_network()
    proxy_url = ensure_proxy(network)
    # cfg["agent"] is already the chosen agent's (compose took --agent):
    # its model, version, login directory and options are that agent's.
    model = request.model or cfg_agent.get("model") or adapter.default_model
    if key_file is not None:
        print(f"[up] {adapter.name}: explicitly selected API authentication", flush=True)
        profile = adapter.prepare_api_profile(key_file, sandbox_dir / "profile")
    else:
        creds_home = agents.resolve_profile(adapter, cfg_agent.get('credentials_dir'),
                                            paths.credentials_dir(request.home) / adapter.name,
                                            user_home=Path.home(), environment=os.environ, link=True)
        print(f"[up] {adapter.name} native login: {creds_home} (source: {creds_home.resolve()})", flush=True)
        profile = adapter.prepare_profile(creds_home, sandbox_dir / "profile")
    print(f"[up] sandbox {sandbox_name}; robot {sim_name} (booting; MoveIt takes a minute)",
          flush=True)
    try:
        _, machine, _ = bring_up(
            cfg, workdir, sim_name, sandbox_name, suite, task_id, network, proxy_url,
            profile.mounts, request.ros_domain,
            robot_log=workdir / "robot.log", home=request.home)
    except Exception as e:  # noqa: BLE001
        # The sandbox directory stays for the read below; openrua clean sweeps it.
        raise UnavailableError(f"[up] robot failed to come up: {e}",
                               hint=f"read {workdir / 'robot.log'}") from e

    def power_off() -> None:
        print("\n[down] powering off", flush=True)
        stop_resources(sandbox_name, machine.shutdown)
        state.stopped(request.name, request.home)
        print(f"[down] files retained at {sandbox_dir}; workspace: {workdir / 'workspace'}")

    robot = cfg["machine"].get("robot", {})
    try:
        if cfg["machine"]["backend"]["kind"] == "sim":
            task = start_episode(machine, request.init_state).get("language", "")
        else:
            task = request.task or ""
        state.save(request.name, request.home, status="running", sim=sim_name, sandbox=sandbox_name,
                   backend=cfg["machine"]["backend"]["kind"],
                   network=network, proxy=proxy_url, agent=adapter.name,
                   agent_spec=cfg_agent["name"], agent_version=cfg_agent.get("version"),
                   model=model,
                   auth_mode="api" if key_file is not None else "native",
                   options={**adapter.default_options,
                            **cfg.get("agent", {}).get("options", {})},
                   workspace=str(workdir / "workspace"), task=task,
                   # What the terminal shows in its header; the resolved config is
                   # the source and these are its names, not a second copy of facts.
                   robot=composed.robot, robot_model=robot.get("model"),
                   simulator=composed.simulator, benchmark=composed.benchmark,
                   suite=suite, task_id=task_id)
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
        agent=adapter.name, model=model, power_off=power_off)
