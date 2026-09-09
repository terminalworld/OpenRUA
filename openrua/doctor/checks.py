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
from dataclasses import dataclass
from pathlib import Path


from openrua import agents, config, proxy
from openrua.config import compose, paths
from openrua.doctor.report import CheckResult, Report


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


def image_label(image: str, label: str) -> str | None:
    return docker_inspect("image", image, f'{{{{index .Config.Labels "{label}"}}}}')


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


def check_docker(ctx: Context) -> list[CheckResult]:
    if not shutil.which("docker"):
        return [CheckResult("docker", "docker not found", "error",
                            hint="install Docker Engine: https://docs.docker.com/engine/install/"
                                 " (or rootless podman with podman-docker: docs/podman.md)")]
    version = engine_version()
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


def _agents_in_image(image: str, ctx: Context, kind: str) -> tuple[str, str]:
    """(severity, detail) for the agents an image should carry. Each
    agent's label is compared with its manifest; an image built before
    per-agent labels existed falls back to the aggregate label of the
    selected set."""
    missing, stale, ok = [], [], []
    for a in ctx.agents:
        have = image_label(image, f"openrua.agent.{a.name}.{kind}_sha256")
        if have:
            (ok if have == agents.fact_sha256(a, kind) else stale).append(a.name)
        else:
            missing.append(a.name)
    if not missing:
        if stale:
            return "warning", f"{', '.join(stale)}: the manifest changed since the image was built"
        return "ok", ""
    aggregate = image_label(image, "openrua.preinstall_sha256" if kind == "install"
                            else "openrua.whitelist_sha256")
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
    severity, detail = _agents_in_image(image, ctx, "whitelist")
    return [CheckResult("proxy-image", f"proxy image {image} present", severity,
                        detail=detail, hint=build if severity != "ok" else "")]


def check_robot_images(ctx: Context) -> list[CheckResult]:
    if ctx.cfg is None:
        return []
    backend = ctx.cfg["machine"].get("backend", {})
    out: list[CheckResult] = []
    if backend.get("kind") == "sim":
        sim = backend["image"]
        if docker_inspect("image", sim, "{{.Id}}") is None:
            out.append(CheckResult("robot-image", f"robot image {sim} missing", "error",
                                   hint=f"openrua build robot --distro {backend['ros_distro']}"))
        else:
            out.append(CheckResult("robot-image", f"robot image {sim} present"))
    sandbox = backend["sandbox_image"]
    names = " ".join(f"--agent {a.name}" for a in ctx.agents)
    tag = "" if sandbox == config.schema.sandbox_image(backend["ros_distro"]) else f" --tag {sandbox}"
    build = f"openrua build sandbox --distro {backend['ros_distro']}{tag} {names}"
    if docker_inspect("image", sandbox, "{{.Id}}") is None:
        out.append(CheckResult("sandbox-image", f"sandbox image {sandbox} missing", "error",
                               hint=build))
    else:
        severity, detail = _agents_in_image(sandbox, ctx, "install")
        out.append(CheckResult("sandbox-image", f"sandbox image {sandbox} present", severity,
                               detail=detail, hint=build if severity != "ok" else ""))
    return out


def check_simulator(ctx: Context) -> list[CheckResult]:
    if ctx.cfg is None:
        return []
    backend = ctx.cfg["machine"].get("backend", {})
    if backend.get("kind") != "sim":
        return [CheckResult("simulator", "real robot: no simulator")]
    venv = paths.simulator_venv(backend["simulator"]["venv"], ctx.home)
    if venv.is_dir():
        return [CheckResult("simulator", f"simulator venv {venv}")]
    return [CheckResult("simulator", f"simulator venv {venv} missing", "error",
                        hint="openrua install --bench <benchmark> (or --sim <engine>) builds it "
                        f"under {paths.simulators_dir(ctx.home)} (docs/simulation.md)")]


def check_login(ctx: Context) -> list[CheckResult]:
    out: list[CheckResult] = []
    configured = (ctx.cfg or {}).get("agent", {}).get("credentials_dir")
    for a in ctx.agents:
        if a.credentials is None:
            out.append(CheckResult(f"login-{a.name}", f"{a.name}: no profile login to check"))
            continue
        home = Path(configured or paths.credentials_dir(ctx.home) / a.name).expanduser()
        if (home / a.credentials.filename).exists():
            out.append(CheckResult(f"login-{a.name}", f"{a.name} login at {home}"))
        else:
            out.append(CheckResult(f"login-{a.name}", f"{a.name} not logged in at {home}",
                                   "error", hint=a.login_hint(home) + (
                                       f"; or pass --token-file ({a.token_hint('<file>')})"
                                       if a.token_env else "")))
    return out


CHECKS: tuple[Callable[[Context], list[CheckResult]], ...] = (
    check_docker, check_home, check_bundled_agents, check_proxy_image,
    check_robot_images, check_simulator, check_login,
)


# ---------------------------------------------------------------- the run

def run(robot: str | None = None, agent_names: list[str] | None = None,
        home: Path | None = None, checks=CHECKS, sim: str | None = None,
        bench: str | None = None) -> Report:
    home = paths.home(home)
    cfg = None
    report = Report()
    if robot or bench:
        try:
            cfg = compose(robot, sim, bench, home).cfg
        except Exception as exc:  # noqa: BLE001
            what = " ".join(x for x in (robot, sim and f"--sim {sim}", bench and f"--bench {bench}") if x)
            report.checks.append(CheckResult("robot-profile", f"{what}: {exc}", "error",
                                             hint="openrua robots / simulators / benchmarks "
                                                  "list what there is"))
    names = agent_names or [(cfg or {}).get("agent", {}).get("name")
                     or config.load_user_config(paths.package_config_path()).agent.name]
    pin = (cfg or {}).get("agent", {}).get("version")
    chosen: list[agents.Agent] = []
    for n in names:
        try:
            chosen.append(agents.get(n, home, version=pin))
        except Exception as exc:  # noqa: BLE001
            report.checks.append(CheckResult(f"agent-{n}", f"agent {n}: {exc}", "error",
                                             hint="openrua agents lists the agents"))
    ctx = Context(home=home, agents=chosen, cfg=cfg, robot=robot)
    for check in checks:
        try:
            report.checks.extend(check(ctx))
        except Exception as exc:  # noqa: BLE001  a crashing check is itself a finding
            report.checks.append(CheckResult(check.__name__, f"{check.__name__} crashed",
                                             "error", detail=repr(exc)))
    return report
