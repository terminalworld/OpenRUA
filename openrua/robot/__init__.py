"""The robot: the machine the agent operates, provided by a backend.

- ``base.py``   the contract: ``Handle`` (wait_ready, rpc, shutdown)
- ``sim/``      a simulated robot: image build, container up/down, the
                host-side client, and ``bridge/`` (the robot's own
                software, runs inside the container)
- ``real/``     a real robot: an optional launch command and a handle
                that waits for the ROS 2 graph

``up()`` and ``down()`` dispatch on ``backend["kind"]``. This package
imports ``openrua.errors`` and nothing else from openrua; every value it
needs (the resolved config path, the simulator venv, the rendered peers
profile) arrives as a parameter.
"""

from __future__ import annotations

from pathlib import Path

from openrua.errors import ConfigError
from openrua.robot.base import Handle  # noqa: F401  re-exported
from openrua.robot.real.down import down as real_down
from openrua.robot.real.up import up as real_up
from openrua.robot.sim.down import down as sim_down
from openrua.robot.sim.up import up as sim_up

KINDS = ("sim", "real")


def up(backend: dict, *, name: str, config_path: str, task_suite: str,
       task_id: int, log_path: Path, network: str | None = None,
       ros_domain: int = 0, static_peer: str | None = None,
       peers_xml: str | None = None, venv: str | None = None,
       code_root: str | None = None, moveit_log: str | None = None,
       probe_argv: list[str] | None = None, record: str | None = None,
       record_cameras: tuple[str, ...] = ()) -> Handle:
    """Bring the robot up and return its handle (not yet waited for).

    ``venv`` and ``code_root`` are the simulated backend's (the resolved
    simulator venv and the directory holding this package); ``probe_argv``
    is the real backend's (a command that succeeds once the graph is
    visible). ``record`` names a directory for the simulated robot's
    camera frames; a real robot has no renderer and refuses it. The
    other keyword arguments apply to both."""
    kind = backend.get("kind")
    if record and kind != "sim":
        raise ConfigError("only a simulated robot can record camera frames "
                          f"(machine.backend.kind is {kind!r})")
    if kind == "sim":
        if not venv or not code_root:
            raise ValueError("a simulated robot needs venv and code_root")
        return sim_up(
            name=name, image=backend["image"],
            config_path=config_path, task_suite=task_suite, task_id=task_id,
            simulator=str(Path(venv).parent), venv=venv, code_root=code_root,
            log_path=log_path, moveit_log=moveit_log, network=network,
            static_peer=static_peer, peers_xml=peers_xml, ros_domain=ros_domain,
            gpus=bool(backend.get("gpus", False)), resources=backend.get("resources"),
            record=record, record_cameras=record_cameras)
    if kind == "real":
        return real_up(name=name, launch=backend.get("launch"), log_path=log_path,
                       probe_argv=probe_argv, image=backend.get("image"))
    raise ConfigError(f"machine.backend.kind must be one of {KINDS}, got {kind!r}")


def down(name: str, kind: str = "sim") -> None:
    """Force the robot down from outside, when its handle is gone or dead."""
    if kind == "sim":
        sim_down(name)
    elif kind == "real":
        real_down(name)
    else:
        raise ConfigError(f"machine.backend.kind must be one of {KINDS}, got {kind!r}")
