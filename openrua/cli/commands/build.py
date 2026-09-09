"""``openrua build robot | sandbox | proxy``: the three images.

The sandbox and proxy images are agent-parameterised: the install line
and the whitelist come from the manifests of the agents named with
``--agent`` (default: the configured default agent), so a manifest is
the only place an agent's install and hosts are written down.
"""

from __future__ import annotations

from pathlib import Path

from openrua import agents, config, proxy
from openrua.config import paths
from openrua.proxy.build import build as build_proxy
from openrua.robot.sim.build import build as build_robot
from openrua.sandbox.build import build as build_sandbox


def default_agent(home: Path | None) -> str:
    """The agent name the defaults files resolve to: the user's file over
    the package's."""
    return config.default_agent_name(
        config.load_user_config(paths.package_config_path()),
        config.load_user_config(paths.config_path(home)))


def manifests_for(names: list[str] | None, home: Path | None) -> list[agents.Manifest]:
    return [agents.manifest(n) for n in (names or [default_agent(home)])]


def preinstall_for(names: list[str] | None, home: Path | None) -> str:
    return agents.preinstall(manifests_for(names, home))


def whitelist_for(names: list[str] | None, home: Path | None) -> str:
    return "\n".join(agents.whitelist(manifests_for(names, home)))


def agent_labels(manifests: list[agents.Manifest], kind: str) -> dict[str, str]:
    """One label per agent baked into an image: the hash of its install
    line (``install``) or whitelist (``whitelist``), so doctor can tell
    which agents an image carries and whether their manifests changed."""
    return {f"openrua.agent.{m.name}.{kind}_sha256": agents.fact_sha256(m, kind)
            for m in manifests}


def run(args) -> int:
    if args.unit is None:
        # bare `openrua build`: the three images with their defaults
        built = [build_robot(distro="jazzy", tag=None),
                 _sandbox(None, None, "jazzy", None, None, args.home),
                 _proxy(None, None, proxy.IMAGE, proxy.PORT, args.home)]
    elif args.unit == "robot":
        built = [build_robot(distro=args.distro, tag=args.tag)]
    elif args.unit == "sandbox":
        built = [_sandbox(args.agent, args.preinstall, args.distro, args.robot_uid,
                          args.tag, args.home)]
    else:
        built = [_proxy(args.agent, args.whitelist, args.tag, args.port, args.home)]
    for tag, digest in built:
        print(f"{tag} {digest}")
    return 0


def _sandbox(agent, preinstall, distro, robot_uid, tag, home):
    chosen = manifests_for(agent, home)
    preinstall = preinstall if preinstall is not None else agents.preinstall(chosen)
    return build_sandbox(preinstall=preinstall, distro=distro, robot_uid=robot_uid,
                         tag=tag, labels=agent_labels(chosen, "install"))


def _proxy(agent, whitelist, tag, port, home):
    chosen = manifests_for(agent, home)
    whitelist = whitelist if whitelist is not None else "\n".join(agents.whitelist(chosen))
    return build_proxy(whitelist=whitelist, tag=tag, port=port,
                       labels=agent_labels(chosen, "whitelist"))


def add_parser(sub) -> None:
    p = sub.add_parser("build", help="build the robot, sandbox and proxy images",
                       description="Build the three images (bare `openrua build`: all "
                       "of them with their defaults), or one of them with its options. "
                       "The sandbox and proxy images take their install line and "
                       "whitelist from the manifests of the agents named with --agent.")
    units = p.add_subparsers(dest="unit", metavar="[<unit>]", required=False)

    r = units.add_parser("robot", help="the simulated robot image (Dockerfile.<distro>)")
    r.add_argument("--distro", default="jazzy", help="ROS 2 distro: jazzy | humble")
    r.add_argument("--tag", default=None, help="image tag (default: openrua-sim-<distro>)")

    s = units.add_parser("sandbox", help="the agent terminal image")
    s.add_argument("--agent", action="append", default=None, metavar="NAME[@VERSION]",
                   help="agent(s) to install, repeatable; @VERSION pins the CLI, "
                   "otherwise the current release (default: the configured default)")
    s.add_argument("--preinstall", default=None,
                   help="install line to bake instead of the agents' manifests")
    s.add_argument("--distro", default="jazzy", help="ROS 2 distro: jazzy | humble "
                   "(the robot's; profiles name it as ros_distro)")
    s.add_argument("--robot-uid", type=int, default=None,
                   help="container uid (default: the current user)")
    s.add_argument("--tag", default=None, help="image tag (default: openrua-sandbox-<distro>)")

    x = units.add_parser("proxy", help="the whitelist proxy image")
    x.add_argument("--agent", action="append", default=None,
                   help="agent(s) whose hosts to allow; repeatable (default: the "
                   "configured default)")
    x.add_argument("--whitelist", default=None,
                   help="regexes, one per line, instead of the agents' manifests")
    x.add_argument("--tag", default=proxy.IMAGE)
    x.add_argument("--port", type=int, default=proxy.PORT, help="listen port, baked in and labelled")
    p.set_defaults(fn=run)
