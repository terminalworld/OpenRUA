"""Finding agents: manifests under configs/agents, hooks under plugins/agents.

A manifest (``<name>.yaml``) is read and validated without importing any
code; the hooks module it names is imported only when an ``Agent`` is
actually needed. Both halves are looked up bundled first, then in the
user directory (``~/.openrua/agents/<name>.yaml``,
``~/.openrua/plugins/agents/<hooks>.py``). A third source, pip entry
points, would come after the user directory; not implemented.
"""

from __future__ import annotations

import hashlib
import importlib
import importlib.util
import logging
import sys
from dataclasses import dataclass
from pathlib import Path

from openrua import config
from openrua.agents.base import Agent, Credentials
from openrua.config import paths
from openrua.errors import NotFound

_log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Manifest:
    """One validated manifest and where it came from."""
    name: str
    path: Path
    source: str                        # "bundled" | "user"
    fields: dict                       # the validated manifest as a dict
    shadowed_by: Path | None = None

    @property
    def install(self) -> str:
        return self.fields["install"]

    @property
    def whitelist(self) -> tuple[str, ...]:
        return tuple(self.fields["whitelist"])


def split_pin(spec: str) -> tuple[str, str | None]:
    """``name`` or ``name@version`` -> (name, version or None)."""
    name, _, version = spec.partition("@")
    return name, (version or None)


def _read(path: Path, source: str, shadowed_by: Path | None = None,
          version: str | None = None) -> Manifest:
    """Validate one manifest. ``version`` pins the CLI (from ``name@version``
    or a config's ``agent.version``); without a pin the install line
    installs whatever is current, and no version check is minted."""
    m = config.validate(config.AgentManifest, config.load_yaml(path), path)
    fields = config.dump(m)
    version = version or m.version
    fields["version"] = version
    install = fields["install"]
    fields["install"] = (install.replace("{version}", version) if version
                         else install.replace("@{version}", "").replace("{version}", ""))
    return Manifest(m.name, path, source, fields, shadowed_by)


def manifests(home: Path | None = None) -> list[Manifest]:
    """Every agent manifest, bundled then user, validated. A manifest that
    fails validation is logged and skipped; it hides nothing else."""
    out: list[Manifest] = []
    for e in paths.available("agents", home):
        try:
            out.append(_read(e.path, e.source, e.shadowed_by))
        except Exception as exc:  # noqa: BLE001
            _log.warning("agent manifest %s skipped: %s", e.path, exc)
    return out


def manifest(spec: str, home: Path | None = None, version: str | None = None) -> Manifest:
    """The manifest for ``name`` or ``name@version``; unknown names list
    what exists."""
    if not spec:
        raise ValueError("agent name is empty; a resolved config always carries agent.name")
    name, pin = split_pin(spec)
    path = paths.find("agents", name, home)
    source = "user" if path.is_relative_to(paths.user_dir("agents", home)) else "bundled"
    return _read(path, source, version=pin or version)


def _load_hooks(hooks: str, home: Path | None) -> type[Agent]:
    """The ``HOOKS`` class of a hooks module: bundled ``openrua.plugins.agents.<hooks>``,
    then ``<home>/plugins/agents/<hooks>.py``."""
    bundled_name = f"openrua.plugins.agents.{hooks}"
    try:
        module = importlib.import_module(bundled_name)
    except ModuleNotFoundError as e:
        if e.name != bundled_name:
            raise
        user_file = paths.user_plugins("agents", home) / f"{hooks}.py"
        if not user_file.is_file():
            raise NotFound(f"hooks module {hooks!r} not found",
                           hint=f"bundled as {bundled_name} or a file at {user_file}") from None
        module = _import_file(user_file)
    cls = getattr(module, "HOOKS", None)
    if not (isinstance(cls, type) and issubclass(cls, Agent)):
        raise TypeError(f"{module.__name__} exposes no HOOKS (an Agent subclass)")
    return cls


def _import_file(path: Path):
    """Import one user-directory module under a private name, so two
    users' files (or a user's and ours) never collide. A failure removes
    the half-initialised module so a retry starts clean."""
    modname = f"_openrua_user_hooks_{path.stem}"
    if modname in sys.modules:
        return sys.modules[modname]
    spec = importlib.util.spec_from_file_location(modname, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[modname] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(modname, None)
        raise
    return module


def compose(m: Manifest, home: Path | None = None) -> Agent:
    """Manifest + hooks -> Agent. Without ``hooks:`` the plain contract
    class is used; it has no launch command, which check_agent reports."""
    fields = dict(m.fields)
    hooks = fields.pop("hooks", None)
    cls = _load_hooks(hooks, home) if hooks else Agent
    if fields.get("credentials") is not None:
        fields["credentials"] = Credentials(**fields["credentials"])
    fields["whitelist"] = tuple(fields.get("whitelist", ()))
    if fields.get("version_argv") is not None:
        fields["version_argv"] = tuple(fields["version_argv"])
    return cls(**fields)


def get(spec: str, home: Path | None = None, version: str | None = None) -> Agent:
    """The Agent for an ``agent.name`` (or ``name@version``): manifest
    found, hooks imported. ``version`` is a config's ``agent.version``."""
    return compose(manifest(spec, home, version), home)


@dataclass(frozen=True)
class Listed:
    name: str
    source: str            # "bundled" | "user"
    path: Path
    agent: Agent | None    # None when the hooks failed to load
    error: str | None = None
    shadowed_by: Path | None = None


def available(home: Path | None = None) -> list[Listed]:
    """Every agent, bundled then user, each composed in isolation: a
    broken hooks module is listed with its error and hides nothing else."""
    out: list[Listed] = []
    for m in manifests(home):
        try:
            out.append(Listed(m.name, m.source, m.path, compose(m, home),
                              shadowed_by=m.shadowed_by))
        except Exception as exc:  # noqa: BLE001
            _log.warning("agent %s failed to load: %s", m.path, exc)
            out.append(Listed(m.name, m.source, m.path, None, error=str(exc),
                              shadowed_by=m.shadowed_by))
    return out


def fact_sha256(agent, kind: str) -> str:
    """The hash of one agent's baked-in fact: its install line
    (``install``) or its whitelist (``whitelist``). Takes an Agent or a
    Manifest."""
    text = agent.install if kind == "install" else "\n".join(agent.whitelist)
    return hashlib.sha256(text.encode()).hexdigest()


def preinstall(agents) -> str:
    """One shell chain installing every agent's CLI (the sandbox image's
    PREINSTALL slot). Takes Agents or Manifests; nothing to install
    contributes nothing."""
    return " && ".join(a.install for a in agents if a.install)


def whitelist(agents) -> tuple[str, ...]:
    """The union of the agents' proxy whitelists, first occurrence order.
    Takes Agents or Manifests."""
    seen: list[str] = []
    for a in agents:
        for line in a.whitelist:
            if line not in seen:
                seen.append(line)
    return tuple(seen)
