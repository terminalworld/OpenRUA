"""``robocli doctor``: is this machine ready to bring a robot up?

Every check is a function returning ``CheckResult`` rows (``id``,
``label``, ``severity``, ``detail``, ``hint``); the runner collects
them in order, isolating a check that itself crashes as an error row.
One report renders two ways with no divergence: lines with a mark per
row and the fix under every error, or JSON (``--json``, or whenever
stdout is not a terminal) for scripts. The exit status is 1 exactly
when an error row exists; warnings never fail. Nothing here changes
the machine.

Checks read the artifacts, not the code: the sandbox and proxy images
carry a label with the hash of what was baked in, and doctor compares
it with what the selected agents would emit today.
"""

from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable

import yaml

from robocli import agents, config
from robocli.config import paths

SEVERITIES = ("ok", "warning", "error")
MARK = {"ok": "ok", "warning": "??", "error": "!!"}


@dataclass
class CheckResult:
    id: str
    label: str
    severity: str = "ok"
    detail: str = ""
    hint: str = ""


@dataclass
class Report:
    checks: list[CheckResult] = field(default_factory=list)

    @property
    def summary(self) -> dict[str, int]:
        return {s: sum(1 for c in self.checks if c.severity == s) for s in SEVERITIES}

    @property
    def ok(self) -> bool:
        return self.summary["error"] == 0

    def to_json(self) -> str:
        return json.dumps({"ok": self.ok, "summary": self.summary,
                           "checks": [asdict(c) for c in self.checks]}, indent=2)

    def render(self) -> str:
        lines = ["robocli doctor"]
        for c in self.checks:
            detail = f"  ({c.detail})" if c.detail else ""
            lines.append(f"  [{MARK[c.severity]}] {c.label}{detail}")
            if c.severity == "error" and c.hint:
                lines.append(f"       fix: {c.hint}")
        s = self.summary
        lines.append(f"{s['ok']} ok, {s['warning']} warnings, {s['error']} errors")
        return "\n".join(lines)


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


def check_docker(ctx: Context) -> list[CheckResult]:
    if shutil.which("docker"):
        return [CheckResult("docker", "docker on PATH")]
    return [CheckResult("docker", "docker not found", "error",
                        hint="install Docker Engine: https://docs.docker.com/engine/install/")]


def check_home(ctx: Context) -> list[CheckResult]:
    h = ctx.home
    if not h.exists():
        return [CheckResult("home", f"user directory {h}", "ok",
                            detail="not created yet; made on first use")]
    import os
    if not os.access(h, os.W_OK):
        return [CheckResult("home", f"user directory {h} is not writable", "error",
                            hint=f"chmod u+w {h}")]
    return [CheckResult("home", f"user directory {h}")]


def check_user_entries(ctx: Context) -> list[CheckResult]:
    """User-directory robots, benchmarks and agents: shadowed names are
    warnings (the bundled one wins), unreadable files are errors."""
    out: list[CheckResult] = []
    for kind in ("robots", "benchmarks"):
        for e in paths.available(kind, ctx.home):
            if e.shadowed_by:
                out.append(CheckResult(
                    f"{kind}-{e.name}-shadowed", f"{kind[:-1]} {e.name}: {e.shadowed_by} "
                    "has a bundled name and is ignored", "warning",
                    hint=f"rename {e.shadowed_by} to use it"))
            if e.source == "user":
                try:
                    yaml.safe_load(e.path.read_text())
                except yaml.YAMLError as exc:
                    out.append(CheckResult(f"{kind}-{e.name}-unreadable",
                                           f"{kind[:-1]} {e.name}: {e.path} is not valid YAML",
                                           "error", detail=str(exc).splitlines()[0]))
    for a in agents.available(ctx.home):
        if a.agent is None:
            out.append(CheckResult(f"agents-{a.name}-broken",
                                   f"agent {a.name}: {a.path} failed to load", "error",
                                   detail=a.error or "", hint="fix the module; it must expose AGENT"))
        elif a.shadowed_by:
            out.append(CheckResult(f"agents-{a.name}-shadowed",
                                   f"agent {a.name}: {a.shadowed_by} has a bundled name and is ignored",
                                   "warning", hint=f"rename {a.shadowed_by} to use it"))
    return out


def check_proxy_image(ctx: Context) -> list[CheckResult]:
    image = "robocli-proxy"
    names = " ".join(f"--agent {a.name}" for a in ctx.agents)
    build = (f'robocli build proxy --whitelist "$(python -m robocli.agents whitelist {names})"')
    if docker_inspect("image", image, "{{.Id}}") is None:
        return [CheckResult("proxy-image", f"proxy image {image} missing", "error", hint=build)]
    want = _sha("\n".join(agents.whitelist(ctx.agents)))
    have = image_label(image, "robocli.whitelist_sha256")
    if have and have != want:
        return [CheckResult("proxy-image", f"proxy image {image} present",
                            "warning", detail="its whitelist differs from what the selected "
                            "agents need", hint=build)]
    return [CheckResult("proxy-image", f"proxy image {image} present",
                        detail="" if have else "unlabelled: built before labels existed")]


def check_robot_images(ctx: Context) -> list[CheckResult]:
    if ctx.cfg is None:
        return []
    backend = ctx.cfg["machine"].get("backend", {})
    out: list[CheckResult] = []
    sim = backend.get("image", "robocli-sim-jazzy")
    if docker_inspect("image", sim, "{{.Id}}") is None:
        out.append(CheckResult("robot-image", f"robot image {sim} missing", "error",
                               hint="robocli build robot"))
    else:
        out.append(CheckResult("robot-image", f"robot image {sim} present"))
    sandbox = backend.get("sandbox_image", "robocli-sandbox")
    names = " ".join(f"--agent {a.name}" for a in ctx.agents)
    build = (f'robocli build sandbox --tag {sandbox} --preinstall '
             f'"$(python -m robocli.agents preinstall {names})"')
    if docker_inspect("image", sandbox, "{{.Id}}") is None:
        out.append(CheckResult("sandbox-image", f"sandbox image {sandbox} missing", "error",
                               hint=build))
    else:
        want = _sha(agents.preinstall(ctx.agents))
        have = image_label(sandbox, "robocli.preinstall_sha256")
        if have and have != want:
            out.append(CheckResult("sandbox-image", f"sandbox image {sandbox} present", "warning",
                                   detail="its agent install differs from what the selected "
                                   "agents need", hint=build))
        else:
            out.append(CheckResult("sandbox-image", f"sandbox image {sandbox} present",
                                   detail="" if have else "unlabelled: built before labels existed"))
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
                        hint=f"build it under {paths.simulators_dir(ctx.home)} (docs/simulation.md) "
                        "or point machine.backend.simulator.venv at it")]


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
    check_docker, check_home, check_user_entries, check_proxy_image,
    check_robot_images, check_simulator, check_login,
)


# ---------------------------------------------------------------- the run

def run(robot: str | None = None, agent_names: list[str] | None = None,
        home: Path | None = None, checks=CHECKS) -> Report:
    home = paths.home(home)
    cfg = None
    report = Report()
    if robot:
        from robocli.bench.run import compose   # the same assembly `up` does
        try:
            cfg, _, _ = compose(robot, None, home)
        except Exception as exc:  # noqa: BLE001
            report.checks.append(CheckResult("robot-profile", f"robot {robot}: {exc}", "error",
                                             hint="robocli robots lists the profiles"))
    names = agent_names or [(cfg or {}).get("agent", {}).get("name")
                     or config.load_user_config(paths.package_config_path()).agent.name]
    chosen: list[agents.Agent] = []
    for n in names:
        try:
            chosen.append(agents.get(n, home))
        except Exception as exc:  # noqa: BLE001
            report.checks.append(CheckResult(f"agent-{n}", f"agent {n}: {exc}", "error",
                                             hint="robocli agents lists the agents"))
    ctx = Context(home=home, agents=chosen, cfg=cfg, robot=robot)
    for check in checks:
        try:
            report.checks.extend(check(ctx))
        except Exception as exc:  # noqa: BLE001  a crashing check is itself a finding
            report.checks.append(CheckResult(check.__name__, f"{check.__name__} crashed",
                                             "error", detail=repr(exc)))
    return report


def main_report(report: Report, as_json: bool | None = None) -> int:
    """Print the report the right way for the reader and return the
    exit status: ``--json`` forces JSON, and so does a non-terminal
    stdout; errors, not warnings, make it 1."""
    if as_json is None:
        as_json = not sys.stdout.isatty()
    print(report.to_json() if as_json else report.render())
    return 0 if report.ok else 1
