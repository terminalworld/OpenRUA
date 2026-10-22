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
import time
from pathlib import Path
from typing import Callable
from urllib.parse import urlparse

from openrua import robot
from openrua.config import paths
from openrua.errors import NotFound, UnavailableError
from openrua.proxy.up import url_from_network as proxy_url_from_network
from openrua.runner import record
from openrua.sandbox.down import down as sandbox_down
from openrua.sandbox.up import up as sandbox_up
from openrua.sandbox.up import write_peers as sandbox_peers

# ROS_STATIC_PEERS exists only from Iron on; Humble's Fast DDS ignores
# it. The portable equivalent is a Fast DDS initial-peers profile; DNS
# names resolve inside it, so container names work as-is. Humble's Fast
# DDS resolves them too, but only after printing an IP_LOCATOR format
# error in every process that loads the profile (each ros2 command the
# agent runs), so a simulated robot on a distro outside NAMES_RESOLVE
# gets addresses instead (bring_up). Both sides of a multicast-less
# network get a rendered copy (robot container and sandbox). Initial peers replace Fast DDS's default discovery peer, the
# multicast group, so it is listed again: without it the processes inside
# one container (bridge, robot_state_publisher, move_group; or two of the
# agent's own nodes) never discover each other.
DDS_DEFAULT_MULTICAST = "239.255.0.1"
NAMES_RESOLVE = ("jazzy",)   # distros whose Fast DDS takes a name in a locator quietly

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


# On the host network every container shares the host's addresses, so
# Fast DDS takes participants in different containers for one host and
# reaches them through shared memory, which containers do not share:
# they discover nothing. A sandbox there speaks UDP only (the drivers'
# side then answers over UDP too): Jazzy's Fast DDS through
# FASTDDS_BUILTIN_TRANSPORTS (a profile naming transports keeps it from
# discovering anything, checked on Jazzy), Humble's, which predates that
# variable, through this profile.
UDP_ONLY_TRANSPORTS = """  <transport_descriptors>
    <transport_descriptor>
      <transport_id>openrua_udp</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
"""
UDP_ONLY_PARTICIPANT = """      <userTransports>
        <transport_id>openrua_udp</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
"""


def peers_profile(peers: list[str], udp_only: bool = False) -> str:
    """The Fast DDS initial-peers profile naming every peer (container
    names or addresses); ``udp_only`` for a sandbox on the host network."""
    locators = "".join(f"          <locator><udpv4><address>{p}</address></udpv4></locator>\n"
                       for p in [DDS_DEFAULT_MULTICAST, *peers])
    profile = FASTDDS_PEERS_XML.format(locators=locators)
    if udp_only:
        profile = profile.replace("  <participant ", UDP_ONLY_TRANSPORTS + "  <participant ", 1)
        profile = profile.replace("      </builtin>\n", "      </builtin>\n" + UDP_ONLY_PARTICIPANT, 1)
    return profile


def agent_proxy_url(backend: dict, proxy_url: str) -> str:
    """The proxy URL as the sandbox can reach it: by container name on the
    internal docker network, by address from the host network a real
    robot's sandbox joins. The agent CLI started inside the sandbox must be
    given this one, not the name-based URL. Empty means no proxy."""
    if not proxy_url or backend.get("kind") != "real":
        return proxy_url
    return proxy_url_from_network(urlparse(proxy_url).hostname)


def internet_access(proxy_url: str) -> str:
    """The sandbox's way out (``sandbox.up``'s ``internet``) for a run with
    this proxy: through it, or, with none, the open internet."""
    return f"proxy:{proxy_url}" if proxy_url else "open"


def host_network_dds(distro: str, peers: list[str]) -> tuple[str | None, tuple[str, ...]]:
    """(DDS profile, environment) for a sandbox on the host network: UDP
    only, and ``peers`` as initial peers when there are any."""
    if distro == "humble":
        return peers_profile(peers, udp_only=True), ()
    return (peers_profile(peers) if peers else None), ("FASTDDS_BUILTIN_TRANSPORTS=UDPv4",)


def sandbox_reachability(backend: dict, network: str, robot_name: str,
                         internet: str) -> dict:
    """How the sandbox reaches the robot's graph and the outside world, as
    ``sandbox.up`` keyword arguments.

    A simulated robot shares the internal docker network with the sandbox;
    the two find each other by container name through a static peer. An
    open internet then means joining a routed network as well
    (INTERNET_NETWORK: OpenRUA's own, so no other container on this host
    shares the sandbox's ROS 2 traffic). A real robot is on the host's
    network, so the sandbox joins the host network and
    ``backend.discovery`` says how it finds the graph: ``network: host``
    (multicast), ``static_peers`` (unicast, rendered into the peers
    profile) or ``discovery_server`` (ROS_DISCOVERY_SERVER). On the host
    network the proxy is reached by address, not by name, and the sandbox
    speaks UDP only (UDP_ONLY_TRANSPORTS). ``internet`` is
    ``none`` (read-only graph probing), ``open`` or ``proxy:<url>``.
    """
    if backend.get("kind") != "real":
        return {"network": network, "static_peer": robot_name,
                "peers_xml": peers_profile([robot_name]), "internet": internet,
                "networks": (INTERNET_NETWORK,) if internet == "open" else (), "env": ()}
    if internet.startswith("proxy:"):
        internet = "proxy:" + agent_proxy_url(backend, internet.split(":", 1)[1])
    discovery = backend.get("discovery", {})
    peers = list(discovery.get("static_peers") or [])
    profile, env = host_network_dds(backend.get("ros_distro", "jazzy"), peers)
    out = {"network": "host", "static_peer": ",".join(peers) or None, "peers_xml": profile,
           "internet": internet, "networks": (), "env": env}
    if discovery.get("discovery_server"):
        out["env"] += (f"ROS_DISCOVERY_SERVER={discovery['discovery_server']}",)
    return out


def start_episode(machine: robot.Handle, init_state_id: int) -> dict:
    """Reset a simulated robot to an initial state and ask its bridge
    what the task is. Returns the bridge's task_info answer (``language``
    and whatever else the loader records about this episode's world)."""
    r = machine.rpc({"cmd": "reset", "init_state_id": init_state_id})
    if not r.get("ok"):
        raise RuntimeError(f"reset failed: {r}")
    return machine.rpc({"cmd": "task_info"})


def address_on(container: str, network: str, timeout_s: float = 30.0) -> str:
    """``container``'s IPv4 address on ``network``, once docker has given
    it one (a container started a moment ago may not have it yet)."""
    form = f'{{{{with index .NetworkSettings.Networks "{network}"}}}}{{{{.IPAddress}}}}{{{{end}}}}'
    deadline = time.monotonic() + timeout_s
    while True:
        r = subprocess.run(["docker", "inspect", "--format", form, container],
                           capture_output=True, text=True)
        if r.returncode == 0 and r.stdout.strip():
            return r.stdout.strip()
        if time.monotonic() > deadline:
            raise UnavailableError(f"{container} has no address on {network}",
                                   hint=r.stderr.strip() or f"docker inspect {container}")
        time.sleep(0.2)


def graph_probe(sandbox_name: str) -> list[str]:
    """A command that exits 0 once the sandbox sees at least one ROS 2
    node: what a real robot's handle waits for."""
    return ["docker", "exec", sandbox_name, "bash", "-c",
            "ros2 node list 2>/dev/null | grep -q ."]


# The routed network a simulated robot's sandbox joins for an open
# internet; only OpenRUA's sandboxes are on it.
INTERNET_NETWORK = "openrua-internet"

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
             task_suite: str | None, task_id: int | None, network: str, internet: str,
             mounts: tuple[str, ...], ros_domain: int | None, robot_log: Path,
             home: Path | None = None,
             record_cameras: tuple[str, ...] | None = None,
             record_every: int = 1, record_size: str | None = None,
             workspace: Path | None = None, seed_workspace: bool = True
             ) -> tuple[Path, robot.Handle, int]:
    """Write the config, start the sandbox, then the robot.

    Sandbox first: the robot's ROS_STATIC_PEERS must resolve the
    sandbox's name at participant creation (mutual unicast discovery).
    The resolved config is written once as ``dest/config.yaml`` and every
    party reads that same file; files the robot opens by path are copied
    next to it first (resolve_robot_files).

    ``internet`` is the sandbox's way out (``internet_access``).
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
    ``dest/frames``, one step in ``record_every``, at ``record_size``
    (WxH; default the profile's camera resolution); None records nothing.
    """
    resolve_robot_files(cfg, dest, home)
    config_path = record.write_config(dest, cfg)
    backend = cfg.get("machine", {}).get("backend", {})
    reach = sandbox_reachability(backend, network, sim_name, internet)
    for routed in reach["networks"]:
        ensure_network(routed, internal=False)

    def start_sandbox(domain: int) -> str:
        sandbox_up(
            cfg,
            workspace if workspace is not None else dest / "workspace",
            image=backend["sandbox_image"],
            run_args=tuple(cfg.get("sandbox", {}).get("run_args", [])),
            ros_domain=domain,
            name=sandbox_name,
            mounts=mounts,
            seed_workspace=seed_workspace,
            preserve_permissions=workspace is not None,
            **reach,
        )
        return sandbox_name

    if backend.get("kind") != "sim":
        ros_domain = 0 if ros_domain is None else ros_domain
    ros_domain = claim_domain(network, ros_domain, start_sandbox, sandbox_down)
    # Addresses instead of names where Fast DDS complains about names: the
    # robot's profile gets the sandbox's (up already), the sandbox's is
    # rewritten with the robot's once its container exists, before
    # anything in the sandbox starts a ros2 process.
    by_address = backend.get("kind") == "sim" and \
        backend.get("ros_distro", "jazzy") not in NAMES_RESOLVE
    machine = None
    try:
        sandbox_peer = address_on(sandbox_name, network) if by_address else sandbox_name
        machine = robot.up(
            backend,
            name=sim_name,
            config_path=str(config_path),
            task_suite=task_suite,
            task_id=task_id,
            log_path=robot_log,
            moveit_log=str(robot_log.with_name("moveit.log")),
            network=network,
            static_peer=sandbox_name,
            peers_xml=peers_profile([sandbox_peer]),
            ros_domain=ros_domain,
            probe_argv=graph_probe(sandbox_name),
            record=str(dest / "frames") if record_cameras is not None else None,
            record_cameras=record_cameras or (),
            record_every=record_every,
            record_size=record_size,
        )
        if by_address:
            sandbox_peers(sandbox_name, peers_profile([address_on(sim_name, network)]))
        machine.wait_ready()
    except BaseException:
        try:
            if machine is not None:
                machine.shutdown()
        finally:
            sandbox_down(sandbox_name)
        raise
    return config_path, machine, ros_domain


def resolve_robot_files(cfg: dict, dest: Path, home: Path | None = None) -> None:
    """Files the robot reads by path are copied next to the config and
    named there by absolute path, so the container sees them through
    the one mount it has on that directory and the config stays
    self-contained: ``machine.controller_config`` (a bundled name under
    ``robots/controllers/`` or a path), and ``task.loader`` and
    ``machine.backend.simulator.engine`` when they are files of your own
    (a bundled module path is left alone)."""
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
    simulator = machine.get("backend", {}).get("simulator", {})
    engine = simulator.get("engine")
    if engine and engine.endswith(".py"):
        simulator["engine"] = _stage(Path(engine), dest)


def _stage(src: Path, dest: Path) -> str:
    dst = dest / src.name
    if dst.resolve() != src.resolve():
        shutil.copyfile(src, dst)
    return str(dst.resolve())


def ensure_network(name: str, internal: bool) -> str:
    """Create the docker network unless it exists; returns its name."""
    subprocess.run(["docker", "network", "create", *(["--internal"] if internal else []), name],
                   capture_output=True)
    return name


def ensure_internal_network(name: str = "openrua-internal") -> str:
    return ensure_network(name, internal=True)
