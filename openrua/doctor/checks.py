"""The checks: each takes the resolved context and returns rows; order = report order.

Checks read the artifacts, not the code: the sandbox and proxy images
carry a label with the hash of what was baked in, and doctor compares
it with what the selected agents' manifests say today. Nothing here
changes the machine; a check that itself crashes becomes an error row.
"""

from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path


from openrua import agents, config, proxy
from openrua.config import compose, paths
from openrua.doctor.report import CheckResult, Report
from openrua.robot.sim import build as sim_build
from openrua.robot.sim import install as installer


# ------------------------------------------------------------ docker access
# One seam for everything that asks docker, so tests can stand in for it.

def docker_inspect(kind: str, name: str, fmt: str) -> str | None:
    """``docker <kind> inspect --format <fmt> <name>``, or None when the
    object does not exist (or docker is missing)."""
    try:
        r = subprocess.run(["docker", kind, "inspect", "--format", fmt, name],
                           capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return r.stdout.strip() if r.returncode == 0 else None


def artifact_label(image: str, label: str, *, object_kind: str = "image") -> str | None:
    value = docker_inspect(object_kind, image, f'{{{{index .Config.Labels "{label}"}}}}')
    return None if value in (None, "", "<no value>") else value


def _sha(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()


# --------------------------------------------------------------- the checks
# Each takes the resolved context and returns rows. Order = report order.

@dataclass
class Context:
    home: Path
    agents: list[agents.Agent]
    cfg: dict | None            # the robot's assembled config, when one was named
    robot: str | None
    install: dict | None = None  # the simulator install the config declares
    sim: str | None = None       # what was named on the command line, for the hints
    bench: str | None = None
    owner: str | None = None     # the declaration the robot image is rendered from
    login_directories: dict[agents.Agent, Path] = field(default_factory=dict)


def engine_version() -> str:
    """What ``docker --version`` prints: Docker Engine, or a podman that
    provides the docker command (podman-docker)."""
    try:
        r = subprocess.run(["docker", "--version"], capture_output=True, text=True,
                           timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return r.stdout.strip() if r.returncode == 0 else ""


def engine_rootless() -> bool:
    """podman's own answer; Docker Engine has no such field and the
    template errors, which reads as not rootless."""
    try:
        r = subprocess.run(["docker", "info", "--format", "{{.Host.Security.Rootless}}"],
                           capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return r.returncode == 0 and r.stdout.strip() == "true"


def engine_error() -> str:
    """Report access to the daemon, not just the installed CLI version."""
    try:
        result = subprocess.run(['docker', 'info', '--format', '{{.ServerVersion}}'],
                                capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return str(exc)
    return (result.stderr.strip() or result.stdout.strip() or 'Docker daemon unavailable') if result.returncode else ''


def check_docker(ctx: Context) -> list[CheckResult]:
    if not shutil.which("docker"):
        return [CheckResult("docker", "docker not found", "error",
                            hint="install Docker Engine: https://docs.docker.com/engine/install/"
                                 " (or rootless podman with podman-docker: docs/podman.md)")]
    version = engine_version()
    problem = engine_error()
    if problem:
        return [CheckResult('docker', 'Docker daemon unavailable', 'error', detail=problem,
                            hint='start Docker or restore access to its socket, then rerun openrua doctor')]
    rows = [CheckResult("docker", "docker on PATH", detail=version)]
    if "podman" in version.lower() and engine_rootless():
        run_args = config.load_user_config(paths.config_path(ctx.home)).sandbox.run_args
        if "--userns=keep-id" in run_args:
            rows.append(CheckResult("sandbox-userns", "rootless podman: sandbox keeps your uid"))
        else:
            rows.append(CheckResult(
                "sandbox-userns", "rootless podman without --userns=keep-id", "error",
                detail="the sandbox user robot would map to a subordinate uid and could "
                       "not write /workspace",
                hint=f"add to {paths.config_path(ctx.home)}:\n"
                     "sandbox:\n  run_args: [\"--userns=keep-id\"]"))
    return rows


def check_home(ctx: Context) -> list[CheckResult]:
    h = ctx.home
    if not h.exists():
        return [CheckResult("home", f"user directory {h}", "ok",
                            detail="not created yet; made on first use")]
    if not os.access(h, os.W_OK):
        return [CheckResult("home", f"user directory {h} is not writable", "error",
                            hint=f"chmod u+w {h}")]
    return [CheckResult("home", f"user directory {h}")]


def check_bundled_agents(ctx: Context) -> list[CheckResult]:
    """Every bundled agent composes (its entry point imports)."""
    out: list[CheckResult] = []
    for a in agents.available():
        if a.agent is None:
            out.append(CheckResult(f"agents-{a.name}-broken",
                                   f"agent {a.name}: {a.path} failed to load", "error",
                                   detail=a.error or "", hint="fix the module; it must expose HOOKS"))
    return out


def _agents_in_artifact(image: str, ctx: Context, kind: str, *, object_kind: str = "image") -> tuple[str, str]:
    """(severity, detail) for the agents an image or container should carry. Each
    agent's label is compared with its manifest; an image built before
    per-agent labels existed falls back to the aggregate label of the
    selected set."""
    missing, stale, ok = [], [], []
    for a in ctx.agents:
        have = artifact_label(image, f"openrua.agent.{a.name}.{kind}_sha256", object_kind=object_kind)
        if have:
            (ok if have == agents.fact_sha256(a, kind) else stale).append(a.name)
        else:
            missing.append(a.name)
    if not missing:
        if stale:
            return "warning", f"{', '.join(stale)}: the manifest changed since the image was built"
        return "ok", ""
    aggregate = artifact_label(image, "openrua.preinstall_sha256" if kind == "install"
                            else "openrua.whitelist_sha256", object_kind=object_kind)
    if not aggregate:
        return "ok", "unlabelled: built before labels existed"
    want = _sha(agents.preinstall(ctx.agents) if kind == "install"
                else "\n".join(agents.whitelist(ctx.agents)))
    if aggregate == want:
        return "ok", ""
    what = "agent install" if kind == "install" else "whitelist"
    return "warning", (f"its {what} differs from what the selected agents need "
                       f"({', '.join(missing)} not labelled on it)")


def check_proxy_image(ctx: Context) -> list[CheckResult]:
    image = proxy.IMAGE
    names = " ".join(f"--agent {a.name}" for a in ctx.agents)
    build = f"openrua build proxy {names}"
    if docker_inspect("image", image, "{{.Id}}") is None:
        return [CheckResult("proxy-image", f"proxy image {image} missing", "error", hint=build)]
    severity, detail = _agents_in_artifact(image, ctx, "whitelist")
    return [CheckResult("proxy-image", f"proxy image {image} present", severity,
                        detail=detail, hint=build if severity != "ok" else "")]


def check_proxy_container(ctx: Context) -> list[CheckResult]:
    """Check the standing container too: rebuilding its tag does not replace it."""
    if docker_inspect("container", proxy.NAME, "{{.Id}}") is None:
        return []
    severity, detail = _agents_in_artifact(proxy.NAME, ctx, "whitelist", object_kind="container")
    return [CheckResult("proxy-container", f"standing proxy {proxy.NAME} present", severity,
                        detail=detail, hint=(
                            "rebuilding the proxy image does not update this container; "
                            "end sessions using it before removing it, then restart your session "
                            "(see docs/install.md#updating-a-shared-proxy)"
                        ) if severity != "ok" else "")]


def _build_hint(ctx: Context) -> str:
    return f"openrua build --bench {ctx.bench}" if ctx.bench else f"openrua build --sim {ctx.sim}"


def check_robot_images(ctx: Context) -> list[CheckResult]:
    """The simulated robot's image exists and was built from the
    declaration as it reads today: the image carries the fingerprint of
    the Dockerfile and files it was rendered from, and the same
    rendering is hashed here without building anything."""
    if ctx.cfg is None:
        return []
    backend = ctx.cfg["machine"].get("backend", {})
    out: list[CheckResult] = []
    if backend.get("kind") == "sim":
        sim = backend["image"]
        if docker_inspect("image", sim, "{{.Id}}") is None:
            out.append(CheckResult("robot-image", f"robot image {sim} missing", "error",
                                   hint=_build_hint(ctx)))
        else:
            have = artifact_label(sim, sim_build.LABEL_FINGERPRINT)
            want = None
            if ctx.install and ctx.owner:
                dockerfile, files = installer.render(ctx.install, ctx.owner, paths.code_root())
                want = installer.fingerprint(dockerfile, files)
            if want and have and have != want:
                out.append(CheckResult("robot-image", f"robot image {sim} present", "warning",
                                       detail="the install declaration changed since it was built",
                                       hint=_build_hint(ctx)))
            else:
                out.append(CheckResult("robot-image", f"robot image {sim} present",
                                       detail=f"openrua {artifact_label(sim, sim_build.LABEL_VERSION) or '?'}"))
    sandbox = backend["sandbox_image"]
    names = " ".join(f"--agent {a.name}" for a in ctx.agents)
    tag = "" if sandbox == config.schema.sandbox_image(backend["ros_distro"]) else f" --tag {sandbox}"
    build = f"openrua build sandbox --distro {backend['ros_distro']}{tag} {names}"
    if docker_inspect("image", sandbox, "{{.Id}}") is None:
        out.append(CheckResult("sandbox-image", f"sandbox image {sandbox} missing", "error",
                               hint=build))
    else:
        severity, detail = _agents_in_artifact(sandbox, ctx, "install")
        out.append(CheckResult("sandbox-image", f"sandbox image {sandbox} present", severity,
                               detail=detail, hint=build if severity != "ok" else ""))
    return out


def check_login(ctx: Context) -> list[CheckResult]:
    out: list[CheckResult] = []
    for a in ctx.agents:
        if a.credentials is None:
            out.append(CheckResult(f"login-{a.name}", f"{a.name}: no profile login to check"))
            continue
        home = ctx.login_directories.get(a, paths.credentials_dir(ctx.home) / a.name)
        credential = home / a.credentials.filename
        try:
            if not credential.is_file():
                raise ValueError('credential file is missing or is not a regular file')
            with credential.open('rb') as stream:
                if not stream.read(1):
                    raise ValueError('credential file is empty')
        except (OSError, ValueError) as exc:
            out.append(CheckResult(f"login-{a.name}", f"{a.name} not logged in at {home}",
                                   "error", detail=str(exc), hint=a.login_hint(home) + (
                                       f"; or pass --token-file ({a.token_hint('<file>')})"
                                       if a.token_env else "")))
        else:
            out.append(CheckResult(f"login-{a.name}", f"{a.name} login file at {home}",
                                   detail=f'source: {home.resolve()}; readable and nonempty; authentication and quota are checked by the native agent'))
    return out


CHECKS: tuple[Callable[[Context], list[CheckResult]], ...] = (
    check_docker, check_home, check_bundled_agents, check_proxy_image, check_proxy_container,
    check_robot_images, check_login,
)


# ---------------------------------------------------------------- the run

def run(robot: str | None = None, agent_names: list[str] | None = None,
        home: Path | None = None, checks=CHECKS, sim: str | None = None,
        bench: str | None = None) -> Report:
    home = paths.home(home)
    cfg = install = owner = None
    report = Report()
    try:
        defaults = config.load_user_config(paths.package_config_path())
        user = config.load_user_config(paths.config_path(home))
    except Exception as exc:  # noqa: BLE001
        report.checks.append(CheckResult("configuration", "invalid configuration", "error",
                                         detail=str(exc), hint="openrua config schema lists supported keys"))
        return report
    if robot or bench:
        try:
            composed = compose(robot, sim, bench, home)
            cfg, install = composed.cfg, composed.install
            if composed.simulator:
                owner = config.image_owner(composed.simulator, bench)
        except Exception as exc:  # noqa: BLE001
            what = " ".join(x for x in (robot, sim and f"--sim {sim}", bench and f"--bench {bench}") if x)
            report.checks.append(CheckResult("robot-profile", f"{what}: {exc}", "error",
                                             hint="openrua robots / simulators / benchmarks "
                                                  "list what there is"))
    configured = (cfg or {}).get("agent", {})
    names = agent_names or [configured.get("name") or config.default_agent_name(defaults, user)]
    chosen: list[agents.Agent] = []
    login_directories = {}
    for n in names:
        try:
            spec, _ = agents.split_pin(n)
            settings = config.layer_agent(configured, defaults, user, agent=spec)
            adapter = agents.get(n, home, version=settings.get("version"))
            chosen.append(adapter)
            login_directories[adapter] = agents.resolve_profile(
                adapter, settings.get('credentials_dir'), paths.credentials_dir(home) / adapter.name,
                user_home=Path.home(), environment=os.environ)
        except Exception as exc:  # noqa: BLE001
            report.checks.append(CheckResult(f"agent-{n}", f"agent {n}: {exc}", "error",
                                             hint="openrua agents lists the agents"))
    ctx = Context(home=home, agents=chosen, cfg=cfg, robot=robot, install=install,
                  sim=sim, bench=bench, owner=owner, login_directories=login_directories)
    for check in checks:
        try:
            report.checks.extend(check(ctx))
        except Exception as exc:  # noqa: BLE001  a crashing check is itself a finding
            report.checks.append(CheckResult(check.__name__, f"{check.__name__} crashed",
                                             "error", detail=repr(exc)))
    return report
