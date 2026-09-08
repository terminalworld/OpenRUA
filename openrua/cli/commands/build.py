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
    layered = config.layer_agent(
        {}, config.load_user_config(paths.package_config_path()),
        config.load_user_config(paths.config_path(home)))
    return layered["name"]


def manifests_for(names: list[str] | None, home: Path | None) -> list[agents.Manifest]:
    return [agents.manifest(n, home) for n in (names or [default_agent(home)])]


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
    if args.unit == "robot":
        tag, digest = build_robot(distro=args.distro, tag=args.tag)
    elif args.unit == "sandbox":
        chosen = manifests_for(args.agent, args.home)
        preinstall = (args.preinstall if args.preinstall is not None
                      else agents.preinstall(chosen))
        tag, digest = build_sandbox(preinstall=preinstall, distro=args.distro,
                            robot_uid=args.robot_uid, tag=args.tag,
                            labels=agent_labels(chosen, "install"))
    else:
        chosen = manifests_for(args.agent, args.home)
        whitelist = (args.whitelist if args.whitelist is not None
                     else "\n".join(agents.whitelist(chosen)))
        tag, digest = build_proxy(whitelist=whitelist, tag=args.tag, port=args.port,
                            labels=agent_labels(chosen, "whitelist"))
    print(f"{tag} {digest}")
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("build", help="build the robot / sandbox / proxy image",
                       description="Build one of the three images. The sandbox and "
                       "proxy images take their install line and whitelist from the "
                       "manifests of the agents named with --agent.")
    units = p.add_subparsers(dest="unit", metavar="<unit>", required=True)

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
