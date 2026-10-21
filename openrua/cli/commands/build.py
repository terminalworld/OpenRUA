"""``openrua build [--bench B ...] [--sim S ...] [--all]`` and
``openrua build sandbox | proxy | base | driver``: the images.

A simulated robot's image is rendered from a declaration: the
simulator's ``install:`` with the benchmark's over it, one image per
declaration that writes install keys (``openrua-sim-<name>``). Bare
``openrua build`` makes what the defaults need: the sandbox and proxy
images for the default agent and the simulator image of the default
benchmark; ``--all`` builds every bundled benchmark's.

The sandbox image is agent-parameterised: the install line comes from
the manifests of the agents named with ``--agent`` (default: the
configured default agent), so a manifest is the only place an agent's
install is written down. The proxy image holds no whitelist; each proxy
gets its agent's hosts when it starts.
"""

from __future__ import annotations

import hashlib
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from openrua import agents, config, proxy
from openrua.errors import UnavailableError, UsageError
from openrua.config import paths
from openrua.config.schema import sim_image
from openrua.proxy.build import build as build_proxy
from openrua.robot.real.build import DriverBuildError, build as build_driver
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


def agent_labels(manifests: list[agents.Manifest], kind: str) -> dict[str, str]:
    """One label per agent baked into an image: the hash of its install
    line, so doctor can tell which agents an image carries and whether
    their manifests changed."""
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
                      build_proxy(proxy.IMAGE, proxy.PORT)]
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
                          args.tag, args.home, getattr(args, "robot", None))]
    elif unit == "driver":
        built = _drivers(args.robot, args.image, args.home)
    else:
        built = [build_proxy(args.tag, args.port)]
    for tag, digest in built:
        print(f"{tag} {digest}")
    return 0


def robot_install(robot: str, home: Path | None) -> dict:
    """The image facts a real robot's config declares: its distro, its
    sandbox image, the install chain for its interface packages and the
    driver images it builds."""
    composed = config.compose(robot, "", "", home)
    backend = composed.cfg["machine"]["backend"]
    if backend["kind"] != "real":
        raise UsageError(f"{robot} is not a real robot instance; --robot names one",
                         hint="openrua build sandbox --distro <distro> builds the shared image")
    return backend


def _drivers(robot: str, image: str | None, home: Path | None) -> list[tuple[str, str]]:
    """Build the images a real robot's config builds itself (``backend.build``),
    or the one named."""
    images = robot_install(robot, home).get("build") or {}
    if image is not None and image not in images:
        raise UsageError(f"{robot} builds no image {image!r}",
                         hint=f"its backend.build names: {', '.join(images) or 'none'}")
    if not images:
        raise UsageError(f"{robot} declares no images to build (backend.build)")
    built = []
    for name, context in images.items():
        if image in (None, name):
            print(f"[build] {name} from {context}", flush=True)
            try:
                built.append(build_driver(name, context))
            except DriverBuildError as e:
                raise UnavailableError(str(e), hint=f"fix {context}/Dockerfile, then "
                                       f"openrua build driver --robot {robot}") from e
    return built


def _sandbox(agent, preinstall, distro, robot_uid, tag, home, robot=None):
    chosen = manifests_for(agent, home)
    labels = agent_labels(chosen, "install")
    chain = preinstall if preinstall is not None else agents.preinstall(chosen)
    if robot:
        backend = robot_install(robot, home)
        distro, tag = backend["ros_distro"], tag or backend["sandbox_image"]
        machine = backend.get("sandbox_install") or ""
        chain = " && ".join(c for c in (chain, machine) if c)
        labels["openrua.machine_install_sha256"] = hashlib.sha256(machine.encode()).hexdigest()
    # An unpinned agent installs whatever is current: a fresh stamp keeps
    # docker from reusing an older build's layer for it.
    fresh = preinstall is None and any(not m.fields.get("version") for m in chosen)
    stamp = datetime.now(timezone.utc).isoformat() if fresh else ""
    # Built under a candidate tag and checked before it takes the real one:
    # a build whose CLI is too old must not replace a sandbox that works.
    tag = tag or config.schema.sandbox_image(distro)
    candidate = f"{tag}-candidate"
    _, digest = build_sandbox(preinstall=chain, distro=distro, robot_uid=robot_uid,
                              tag=candidate, labels=labels, stamp=stamp)
    try:
        _report_versions(candidate, chosen)
        subprocess.run(["docker", "tag", candidate, tag], check=True)
    finally:
        subprocess.run(["docker", "rmi", candidate], capture_output=True)
    return tag, digest


def _report_versions(image: str, chosen: list[agents.Manifest]) -> None:
    """Say which CLI each agent got, and refuse an image whose CLI is
    older than its adapter's flags need: better at build time than as a
    disconnected agent at the first session."""
    for m in chosen:
        agent = agents.get(m.name, version=m.fields.get("version"))
        if not agent.version_argv:
            continue
        found = subprocess.run(["docker", "run", "--rm", "--entrypoint", "", image,
                                *agent.version_argv], capture_output=True, text=True)
        print(f"[build] {m.name}: {found.stdout.strip() or found.stderr.strip()}", flush=True)
        check = agent.sandbox_cli_check()
        if check and subprocess.run(["docker", "run", "--rm", "--entrypoint", "", image,
                                     "bash", "-c", check[1]]).returncode:
            want = agent.version or f"{agent.min_version} or newer"
            raise UnavailableError(f"{image} carries {m.name} {found.stdout.strip()}, "
                                   f"not {want}",
                                   hint=f"pin a release with --agent {m.name}@<version>")


def add_parser(sub) -> None:
    p = sub.add_parser("build", help="build the images: simulators, sandbox, proxy",
                       description="Build simulator images from their declarations "
                       "(--bench / --sim, repeatable; --all: every bundled benchmark), "
                       "or one of the agent-side images (sandbox, proxy), the ROS "
                       "base, or a real robot's driver images. Bare `openrua build` makes the sandbox and proxy images "
                       "for the default agent and the default benchmark's simulator "
                       "image. The sandbox takes its install line from the manifests "
                       "of the agents named with --agent.")
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
    s.add_argument("--robot", default=None, metavar="NAME|PATH",
                   help="a real robot whose config declares backend.sandbox_install: "
                   "its distro, image name and install chain come from there")

    d = units.add_parser("driver", help="the images a real robot's launch command uses, "
                         "from the directories its config names (backend.build)")
    d.add_argument("--robot", required=True, metavar="NAME|PATH", help="the real robot")
    d.add_argument("--image", default=None, help="only this one of them")

    x = units.add_parser("proxy", help="the whitelist proxy image (runs that limit the "
                         "agent to its model service start one per agent's hosts)")
    x.add_argument("--tag", default=proxy.IMAGE)
    x.add_argument("--port", type=int, default=proxy.PORT, help="listen port, baked in and labelled")
    p.set_defaults(fn=run)
