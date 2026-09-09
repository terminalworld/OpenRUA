"""Bring a robot and its sandbox up from one resolved config.

The same path serves ``openrua up`` and every trial: the config is
written once as ``<dest>/config.yaml`` and every party reads that file
(the workspace seeding, the robot's bridge, preflight). Files the robot
opens by path are copied next to it first, so the container sees them
through the one mount it has on that directory.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path
from typing import Callable

from openrua import robot
from openrua.config import paths
from openrua.errors import NotFound, UnavailableError
from openrua import proxy
from openrua.proxy.up import url_from_network as proxy_url_from_network
from openrua.runner import record
from openrua.sandbox.down import down as sandbox_down
from openrua.sandbox.up import up as sandbox_up

# ROS_STATIC_PEERS exists only from Iron on; Humble's Fast DDS ignores
# it. The portable equivalent is a Fast DDS initial-peers profile; DNS
# names resolve inside it, so container names work as-is. Both sides of
# a multicast-less network get a rendered copy (robot container and
# sandbox).
FASTDDS_PEERS_XML = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="openrua_initial_peers" is_default_profile="true">
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


# ROS_DOMAIN_ID values that keep DDS on its default ports (ROS 2 documents
# 0-101 as safe on Linux); the higher range is reserved for hand-picked ids.
DOMAINS = range(0, 102)


def domains_in_use(network: str) -> dict[int, tuple[str, str]]:
    """``{ROS_DOMAIN_ID: (started, container)}`` over the containers on
    ``network``, keeping the earliest-started container per domain.

    Read from the containers themselves, never from bookkeeping: a
    container that is up is the fact that matters, and only containers
    on this network can hear each other's DDS traffic. A daemon that
    cannot be asked is not evidence that nothing is running, so the
    answer is then "unknown" (an exception), not "free".
    """
    names = subprocess.run(
        ["docker", "ps", "--filter", f"network={network}", "--format", "{{.Names}}"],
        capture_output=True, text=True, timeout=30, check=True).stdout.split()
    if not names:
        return {}
    r = subprocess.run(["docker", "inspect", *names],
                       capture_output=True, text=True, timeout=60)
    # A container that vanished between ps and inspect (a bring-up that
    # lost its domain backing off) makes docker exit 1 while still
    # printing the rest; the rest is what matters.
    infos = json.loads(r.stdout) if r.stdout.strip() else []
    used: dict[int, tuple[str, str]] = {}
    for info in infos:
        env = dict(e.split("=", 1) for e in info["Config"].get("Env") or [] if "=" in e)
        if "ROS_DOMAIN_ID" not in env:
            continue
        try:
            domain = int(env["ROS_DOMAIN_ID"])
        except ValueError:
            continue
        claim = (_instant(info["State"]["StartedAt"]), info["Name"].lstrip("/"))
        if domain not in used or claim < used[domain]:
            used[domain] = claim
    return used


def _instant(started_at: str) -> str:
    """A container's StartedAt as a string that orders correctly: the
    engines write RFC 3339 UTC with a fraction of varying length (docker
    nine digits, podman as few as it needs), so the fraction is padded
    before two of them are compared."""
    head, _, frac = started_at.replace("+00:00", "Z").rstrip("Z").partition(".")
    return f"{head}.{frac.ljust(9, '0')}"


def free_domain(used: dict[int, tuple[str, str]], skip: set[int] = frozenset()) -> int:
    """The lowest domain no container uses, ``skip`` excluded."""
    for d in DOMAINS:
        if d not in used and d not in skip:
            return d
    raise UnavailableError(f"every ROS domain {DOMAINS.start}-{DOMAINS.stop - 1} is in use",
                           hint="wait for a robot to power off, or pass --ros-domain")


def claim_domain(network: str, requested: int | None,
                 start: Callable[[int], str], stop: Callable[[str], None]) -> int:
    """Start the first container of a bring-up on a ROS domain nobody
    else on ``network`` uses, and return the domain.

    A requested domain is taken as given. Otherwise the lowest free one
    is tried: ``start(domain)`` starts the container (returning its
    name), then the network is read again, because between reading and
    starting another bring-up may have chosen the same number. The
    earlier-started container keeps the domain; the later one is stopped
    with ``stop(name)`` and the next free domain is tried. Bounded by the
    number of domains, so two bring-ups racing each other converge.
    """
    if requested is not None:
        start(requested)
        return requested
    lost: set[int] = set()
    while True:
        domain = free_domain(domains_in_use(network), skip=lost)
        name = start(domain)
        try:
            holder = domains_in_use(network).get(domain)
        except BaseException:
            stop(name)      # never leave the container we just started behind
            raise
        if holder is None or holder[1] == name:
            print(f"[bringup] ROS_DOMAIN_ID {domain}: lowest free on {network}", flush=True)
            return domain
        print(f"[bringup] ROS_DOMAIN_ID {domain} was taken by {holder[1]} first; "
              "trying the next", flush=True)
        stop(name)
        lost.add(domain)


def bring_up(cfg: dict, dest: Path, sim_name: str, sandbox_name: str,
             task_suite: str, task_id: int, network: str, proxy_url: str,
             mounts: tuple[str, ...], ros_domain: int | None, robot_log: Path,
             home: Path | None = None,
             record_cameras: tuple[str, ...] | None = None
             ) -> tuple[Path, robot.Handle, int]:
    """Write the config, start the sandbox, then the robot.

    Sandbox first: the robot's ROS_STATIC_PEERS must resolve the
    sandbox's name at participant creation (mutual unicast discovery).
    The resolved config is written once as ``dest/config.yaml`` and every
    party reads that same file; files the robot opens by path are copied
    next to it first (resolve_robot_files).

    ``ros_domain`` None means: a simulated robot takes the lowest ROS
    domain no container on the internal network uses (claim_domain), so
    concurrent bring-ups on one machine never share a DDS graph; a real
    robot joins its own graph, domain 0 unless told otherwise.

    Returns the config path, the robot's handle, ready (``wait_ready``
    done), and the domain the pair runs on. If the robot fails to come
    up, the sandbox this call started is torn down before the error
    propagates: the caller never inherits half a bring-up.
    ``record_cameras`` (camera names; empty = the profile's
    ``cameras.record``) makes the robot write its frames to
    ``dest/frames``; None records nothing.
    """
    resolve_robot_files(cfg, dest, home)
    config_path = record.write_config(dest, cfg)
    backend = cfg.get("machine", {}).get("backend", {})
    reach = sandbox_reachability(backend, network, sim_name, proxy_url)

    def start_sandbox(domain: int) -> str:
        sandbox_up(
            cfg,
            dest / "workspace",
            image=backend["sandbox_image"],
            run_args=tuple(cfg.get("sandbox", {}).get("run_args", [])),
            ros_domain=domain,
            name=sandbox_name,
            mounts=mounts,
            **reach,
        )
        return sandbox_name

    if backend.get("kind") != "sim":
        ros_domain = 0 if ros_domain is None else ros_domain
    ros_domain = claim_domain(network, ros_domain, start_sandbox, sandbox_down)
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
            # A venv's editable installs may point into sibling checkouts
            # (a LIBERO fork's venv uses cap-x's robosuite): the whole
            # simulators directory is visible, by its real path.
            mounts=(str(paths.simulators_dir(home).resolve()),),
        )
        machine.wait_ready()
    except BaseException:
        if machine is not None:
            machine.shutdown()
        sandbox_down(sandbox_name)
        raise
    return config_path, machine, ros_domain


def resolve_robot_files(cfg: dict, dest: Path, home: Path | None = None) -> None:
    """Files the robot reads by path are copied next to the config and
    named there by absolute path, so the container sees them through
    the one mount it has on that directory and the config stays
    self-contained: ``machine.controller_config`` (a bundled name under
    ``robots/controllers/`` or a path) and ``task.loader`` when it is a
    file of your own (a bundled module path is left alone)."""
    del home
    machine = cfg.get("machine", {})
    spec = machine.get("controller_config")
    if spec:
        p = Path(spec).expanduser()
        candidates = [p] if p.is_absolute() else [
            paths.bundled("robots") / "controllers" / p, paths.bundled("robots") / p, p]
        src = next((c for c in candidates if c.is_file()), None)
        if src is None:
            looked = ", ".join(str(c) for c in candidates)
            raise NotFound(f"machine.controller_config {spec!r} not found (looked at: {looked})")
        machine["controller_config"] = _stage(src, dest)
    loader = cfg.get("task", {}).get("loader")
    if loader and loader.endswith(".py"):
        cfg["task"]["loader"] = _stage(Path(loader), dest)


def _stage(src: Path, dest: Path) -> str:
    dst = dest / src.name
    if dst.resolve() != src.resolve():
        shutil.copyfile(src, dst)
    return str(dst.resolve())


def ensure_internal_network(name: str = "openrua-internal") -> str:
    subprocess.run(
        ["docker", "network", "create", "--internal", name], capture_output=True
    )
    return name
