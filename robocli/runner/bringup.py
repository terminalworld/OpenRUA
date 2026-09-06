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
from robocli.runner import record
from robocli.sandbox.down import down as sandbox_down
from robocli.sandbox.up import up as sandbox_up

# ROS_STATIC_PEERS exists only from Iron on; Humble's Fast DDS ignores
# it. The portable equivalent is a Fast DDS initial-peers profile; DNS
# names resolve inside it, so container names work as-is. BOTH sides of
# a multicast-less network get a rendered copy (sim container and
# sandbox).
FASTDDS_PEERS_XML = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="robocli_initial_peers" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator><udpv4><address>{peer}</address></udpv4></locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
"""


def simulator_venv(backend: dict, home: Path | None = None) -> Path:
    """The simulator venv named by machine.backend.simulator.venv (see
    paths.simulator_venv for how relative names resolve)."""
    return paths.simulator_venv(backend["simulator"]["venv"], home)


def bring_up(cfg: dict, dest: Path, sim_name: str, sandbox_name: str,
             task_suite: str, task_id: int, network: str, proxy_url: str,
             mounts: tuple[str, ...], ros_domain: int, robot_log: Path,
             home: Path | None = None) -> tuple[Path, robot.Handle]:
    """Resolve, sandbox, robot, in that order, from one resolved config.

    Sandbox first: the body's ROS_STATIC_PEERS must resolve the sandbox's
    name at participant creation (mutual unicast discovery). The resolved
    config is written ONCE as ``dest/config.yaml`` and every party reads
    that same file: the manual seeding, the body's boot, the machine
    itself (ruling 2026-08-16). Files the body opens by path are copied
    next to it first (resolve_body_files).

    Returns the assembly path and the machine's control line, ready
    (``wait_ready`` done). If the body fails to come up, the sandbox this
    call started is torn down before the error propagates: the caller
    never inherits half a bring-up.
    """
    resolve_body_files(cfg, dest, home)
    config_path = record.write_config(dest, cfg)
    backend = cfg.get("machine", {}).get("backend", {})
    sandbox_up(
        config=config_path,
        workspace=dest / "workspace",
        image=backend.get("sandbox_image"),
        network=network,
        static_peer=sim_name,
        peers_xml=FASTDDS_PEERS_XML.format(peer=sim_name),
        ros_domain=ros_domain,
        internet=f"proxy:{proxy_url}",
        name=sandbox_name,
        mounts=mounts,
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
            peers_xml=FASTDDS_PEERS_XML.format(peer=sandbox_name),
            ros_domain=ros_domain,
        )
        machine.wait_ready()
    except BaseException:
        if machine is not None:
            machine.shutdown()
        sandbox_down(sandbox_name)
        raise
    return config_path, machine


def resolve_body_files(cfg: dict, dest: Path, home: Path | None = None) -> None:
    """Files the body reads by path (today: ``machine.controller_config``)
    are copied next to the assembly and named there by absolute path, so
    the container sees them through the one mount it has on that
    directory and the assembly stays self-contained. Names resolve
    bundled (``robocli/robots/``), then ``<home>/robots/``, then as a
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
