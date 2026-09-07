"""Bring a robot and its sandbox up from one resolved config.

The same path serves ``robocli up`` and every trial: the config is
written once as ``<dest>/config.yaml`` and every party reads that file
(the workspace seeding, the robot's bridge, preflight). Files the robot
opens by path are copied next to it first, so the container sees them
through the one mount it has on that directory.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

from robocli import robot
from robocli.config import paths
from robocli.errors import NotFound
from robocli import proxy
from robocli.proxy.up import url_from_network as proxy_url_from_network
from robocli.runner import record
from robocli.sandbox.down import down as sandbox_down
from robocli.sandbox.up import up as sandbox_up

# ROS_STATIC_PEERS exists only from Iron on; Humble's Fast DDS ignores
# it. The portable equivalent is a Fast DDS initial-peers profile; DNS
# names resolve inside it, so container names work as-is. Both sides of
# a multicast-less network get a rendered copy (robot container and
# sandbox).
FASTDDS_PEERS_XML = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="robocli_initial_peers" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
{locators}        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
"""


def peers_profile(peers: list[str]) -> str:
    """The Fast DDS initial-peers profile naming every peer (container
    names or addresses)."""
    locators = "".join(f"          <locator><udpv4><address>{p}</address></udpv4></locator>\n"
                       for p in peers)
    return FASTDDS_PEERS_XML.format(locators=locators)


def sandbox_reachability(backend: dict, network: str, robot_name: str,
                         proxy_url: str, proxy_name: str = proxy.NAME) -> dict:
    """How the sandbox reaches the robot's graph and the proxy, as
    ``sandbox.up`` keyword arguments.

    A simulated robot shares the internal docker network with the sandbox;
    the two find each other by container name through a static peer. A
    real robot is on the host's network, so the sandbox joins the host
    network and ``backend.discovery`` says how it finds the graph:
    ``network: host`` (multicast), ``static_peers`` (unicast, rendered
    into the peers profile) or ``discovery_server`` (ROS_DISCOVERY_SERVER).
    On the host network the proxy is reached by address, not by name.
    """
    if backend.get("kind") != "real":
        return {"network": network, "static_peer": robot_name,
                "peers_xml": peers_profile([robot_name]),
                "internet": f"proxy:{proxy_url}", "env": ()}
    discovery = backend.get("discovery", {})
    out = {"network": "host", "static_peer": None, "peers_xml": None,
           "internet": f"proxy:{proxy_url_from_network(proxy_name)}", "env": ()}
    if discovery.get("static_peers"):
        peers = list(discovery["static_peers"])
        out["static_peer"] = ",".join(peers)
        out["peers_xml"] = peers_profile(peers)
    elif discovery.get("discovery_server"):
        out["env"] = (f"ROS_DISCOVERY_SERVER={discovery['discovery_server']}",)
    return out


def start_episode(machine: robot.Handle, init_state_id: int) -> dict:
    """Reset a simulated robot to an initial state and ask its bridge
    what the task is. Returns the bridge's task_info answer (``language``
    and whatever else the loader records about this episode's world)."""
    r = machine.rpc({"cmd": "reset", "init_state_id": init_state_id})
    if not r.get("ok"):
        raise RuntimeError(f"reset failed: {r}")
    return machine.rpc({"cmd": "task_info"})


def graph_probe(sandbox_name: str) -> list[str]:
    """A command that exits 0 once the sandbox sees at least one ROS 2
    node: what a real robot's handle waits for."""
    return ["docker", "exec", sandbox_name, "bash", "-c",
            "ros2 node list 2>/dev/null | grep -q ."]


def simulator_venv(backend: dict, home: Path | None = None) -> Path:
    """The simulator venv named by machine.backend.simulator.venv (see
    paths.simulator_venv for how relative names resolve)."""
    return paths.simulator_venv(backend["simulator"]["venv"], home)


def bring_up(cfg: dict, dest: Path, sim_name: str, sandbox_name: str,
             task_suite: str, task_id: int, network: str, proxy_url: str,
             mounts: tuple[str, ...], ros_domain: int, robot_log: Path,
             home: Path | None = None,
             record_cameras: tuple[str, ...] | None = None) -> tuple[Path, robot.Handle]:
    """Write the config, start the sandbox, then the robot.

    Sandbox first: the robot's ROS_STATIC_PEERS must resolve the
    sandbox's name at participant creation (mutual unicast discovery).
    The resolved config is written once as ``dest/config.yaml`` and every
    party reads that same file; files the robot opens by path are copied
    next to it first (resolve_robot_files).

    Returns the config path and the robot's handle, ready (``wait_ready``
    done). If the robot fails to come up, the sandbox this call started
    is torn down before the error propagates: the caller never inherits
    half a bring-up. ``record_cameras`` (camera names; empty = the
    profile's ``cameras.record``) makes the robot write its frames to
    ``dest/frames``; None records nothing.
    """
    resolve_robot_files(cfg, dest, home)
    config_path = record.write_config(dest, cfg)
    backend = cfg.get("machine", {}).get("backend", {})
    reach = sandbox_reachability(backend, network, sim_name, proxy_url)
    sandbox_up(
        cfg,
        dest / "workspace",
        image=backend["sandbox_image"],
        run_args=tuple(cfg.get("sandbox", {}).get("run_args", [])),
        ros_domain=ros_domain,
        name=sandbox_name,
        mounts=mounts,
        **reach,
    )
    machine = None
    try:
        # The container mounts and runs the venv by this string, and a
        # host symlink means nothing inside it: hand over the real path.
        venv = (simulator_venv(backend, home).resolve()
                if backend.get("kind") == "sim" else None)
        machine = robot.up(
            backend,
            name=sim_name,
            config_path=str(config_path),
            task_suite=task_suite,
            task_id=task_id,
            venv=str(venv) if venv else None,
            code_root=str(paths.code_root()),
            log_path=robot_log,
            moveit_log=str(robot_log.with_name("moveit.log")),
            network=network,
            static_peer=sandbox_name,
            peers_xml=peers_profile([sandbox_name]),
            ros_domain=ros_domain,
            probe_argv=graph_probe(sandbox_name),
            record=str(dest / "frames") if record_cameras is not None else None,
            record_cameras=record_cameras or (),
        )
        machine.wait_ready()
    except BaseException:
        if machine is not None:
            machine.shutdown()
        sandbox_down(sandbox_name)
        raise
    return config_path, machine


def resolve_robot_files(cfg: dict, dest: Path, home: Path | None = None) -> None:
    """Files the robot reads by path (``machine.controller_config``) are
    copied next to the config and named there by absolute path, so the
    container sees them through the one mount it has on that directory
    and the config stays self-contained. Names resolve bundled
    (``robocli/configs/robots/``), then ``<home>/robots/``, then as a
    path."""
    machine = cfg.get("machine", {})
    spec = machine.get("controller_config")
    if not spec:
        return
    p = Path(spec).expanduser()
    candidates = [p] if p.is_absolute() else [
        paths.bundled("robots") / p, paths.user_dir("robots", home) / p, p]
    src = next((c for c in candidates if c.is_file()), None)
    if src is None:
        looked = ", ".join(str(c) for c in candidates)
        raise NotFound(f"machine.controller_config {spec!r} not found (looked at: {looked})")
    dst = dest / src.name
    shutil.copyfile(src, dst)
    machine["controller_config"] = str(dst.resolve())


def ensure_internal_network(name: str = "robocli-internal") -> str:
    subprocess.run(
        ["docker", "network", "create", "--internal", name], capture_output=True
    )
    return name
