"""``openrua build [--bench B ...] [--sim S ...] [--all]`` and
``openrua build sandbox | proxy | base``: the images.

A simulated robot's image is rendered from a declaration: the
simulator's ``install:`` with the benchmark's over it, one image per
declaration that writes install keys (``openrua-sim-<name>``). Bare
``openrua build`` makes what the defaults need: the sandbox and proxy
images for the default agent and the simulator image of the default
benchmark; ``--all`` builds every bundled benchmark's.

The sandbox and proxy images are agent-parameterised: the install line
and the whitelist come from the manifests of the agents named with
``--agent`` (default: the configured default agent), so a manifest is
the only place an agent's install and hosts are written down.
"""

from __future__ import annotations

from pathlib import Path

from openrua import agents, config, proxy
from openrua.config import paths
from openrua.config.schema import sim_image
from openrua.proxy.build import build as build_proxy
from openrua.robot.sim.build import build_base, build_simulator
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


def simulator_targets(benches: list[str], sims: list[str], every: bool,
                      home: Path | None) -> list[tuple[str, dict, str]]:
    """``(name, install, tag)`` per simulator image to build, deduplicated:
    two benchmarks that add nothing to their simulator share its image."""
    if every:
        benches = [e.name for e in paths.available("benchmarks")]
    if not benches and not sims:
        user = config.load_user_config(paths.config_path(home))
        defaults = config.load_user_config(paths.package_config_path())
        bench = user.benchmark or defaults.benchmark
        if not bench:
            return []
        benches = [bench]
    seen: dict[str, tuple[str, dict, str]] = {}
    for b in benches:
        _, decl = config.load_benchmark(b)
        owner = config.image_owner(decl["simulator"], b)
        seen.setdefault(owner, (owner, config.install_for(decl["simulator"], b), sim_image(owner)))
    for s in sims:
        owner = config.image_owner(s)
        seen.setdefault(owner, (owner, config.install_for(s), sim_image(owner)))
    return list(seen.values())


def run(args) -> int:
    unit = getattr(args, "unit", None)
    if unit is None:
        built = []
        if not (args.bench or args.sim or args.all):
            # bare `openrua build`: the agent images too
            built += [_sandbox(None, None, "jazzy", None, None, args.home),
                      _proxy(None, None, proxy.IMAGE, proxy.PORT, args.home)]
        targets = simulator_targets(args.bench or [], args.sim or [], args.all, args.home)
        if not targets:
            print("[build] no default benchmark, so no simulator image: openrua build "
                  "--bench <name> builds one (openrua benchmarks lists them; openrua config "
                  "set --bench <name> makes one the default)", flush=True)
        for name, install, tag in targets:
            print(f"[build] {tag} from the {name} declaration", flush=True)
            built.append(build_simulator(install, name, paths.code_root(), tag=tag))
    elif unit == "base":
        built = [build_base(distro=args.distro, tag=args.tag)]
    elif unit == "sandbox":
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
    p = sub.add_parser("build", help="build the images: simulators, sandbox, proxy",
                       description="Build simulator images from their declarations "
                       "(--bench / --sim, repeatable; --all: every bundled benchmark), "
                       "or one of the agent-side images (sandbox, proxy) or the ROS "
                       "base. Bare `openrua build` makes the sandbox and proxy images "
                       "for the default agent and the default benchmark's simulator "
                       "image. The sandbox and proxy take their install line and "
                       "whitelist from the manifests of the agents named with --agent.")
    p.add_argument("--bench", action="append", default=None, metavar="NAME",
                   help="benchmark whose simulator image to build (openrua benchmarks); "
                        "repeatable")
    p.add_argument("--sim", action="append", default=None, metavar="NAME",
                   help="simulator whose image to build, for its native scene "
                        "(openrua simulators); repeatable")
    p.add_argument("--all", action="store_true", help="every bundled benchmark's image")
    units = p.add_subparsers(dest="unit", metavar="[<unit>]", required=False)

    b = units.add_parser("base", help="the ROS base image simulator images build on")
    b.add_argument("--distro", default="jazzy", help="ROS 2 distro: jazzy | humble")
    b.add_argument("--tag", default=None, help="image tag (default: openrua-sim-base-<distro>)")

    s = units.add_parser("sandbox", help="the agent terminal image")
    s.add_argument("--agent", action="append", default=None, metavar="NAME[@VERSION]",
                   help="agent(s) to install, repeatable; @VERSION pins the CLI, "
                   "otherwise the current release (default: the configured default)")
    s.add_argument("--preinstall", default=None,
                   help="install line to bake instead of the agents' manifests")
    s.add_argument("--distro", default="jazzy", help="ROS 2 distro: jazzy | humble "
                   "(the robot's; declarations name it as ros_distro)")
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
