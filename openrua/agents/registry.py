"""Finding agents: manifests under configs/agents, code behind ``entry_point``.

A manifest (``<name>.yaml``) is read and validated without importing any
code; the module its ``entry_point`` names is imported only when an
``Agent`` is actually needed. A bundled name resolves under
``openrua/plugins/agents/``; a manifest of your own is passed as a path
and its entry_point is a path relative to it (``./my_agent.py``). Both
expose ``HOOKS``, an Agent subclass.
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
    fields: dict                       # the validated manifest as a dict

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


def _read(path: Path, version: str | None = None) -> Manifest:
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
    if fields.get("entry_point"):
        fields["entry_point"] = paths.entry_point("agents", fields["entry_point"], path)
    return Manifest(m.name, path, fields)


def manifests() -> list[Manifest]:
    """Every bundled agent manifest, validated. A manifest that fails
    validation is logged and skipped; it hides nothing else."""
    out: list[Manifest] = []
    for e in paths.available("agents"):
        try:
            out.append(_read(e.path))
        except Exception as exc:  # noqa: BLE001
            _log.warning("agent manifest %s skipped: %s", e.path, exc)
    return out


def manifest(spec: str, version: str | None = None) -> Manifest:
    """The manifest for ``name`` or ``name@version`` (a bundled name or a
    path); unknown names list what exists."""
    if not spec:
        raise ValueError("agent name is empty; a resolved config always carries agent.name")
    name, pin = split_pin(spec)
    return _read(paths.find("agents", name), version=pin or version)


def _load_hooks(entry_point: str) -> type[Agent]:
    """The ``HOOKS`` class of a resolved entry point: a module path
    (``openrua.plugins.agents.<name>``) or an absolute file path."""
    if entry_point.endswith(".py"):
        module = _import_file(Path(entry_point))
    else:
        try:
            module = importlib.import_module(entry_point)
        except ModuleNotFoundError as e:
            if e.name != entry_point:
                raise
            raise NotFound(f"entry_point module {entry_point!r} not found") from None
    cls = getattr(module, "HOOKS", None)
    if not (isinstance(cls, type) and issubclass(cls, Agent)):
        raise TypeError(f"{module.__name__} exposes no HOOKS (an Agent subclass)")
    return cls


def _import_file(path: Path):
    """Import one module by file path under a private name, so two files
    of the same stem never collide. A failure removes the
    half-initialised module so a retry starts clean."""
    modname = f"_openrua_entry_point_{path.stem}_{abs(hash(str(path)))}"
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


def compose(m: Manifest) -> Agent:
    """Manifest + entry point -> Agent. Without ``entry_point:`` the plain
    contract class is used; it has no launch command, which check_agent
    reports."""
    fields = dict(m.fields)
    entry = fields.pop("entry_point", None)
    cls = _load_hooks(entry) if entry else Agent
    if fields.get("credentials") is not None:
        fields["credentials"] = Credentials(**fields["credentials"])
    fields["whitelist"] = tuple(fields.get("whitelist", ()))
    if fields.get("version_argv") is not None:
        fields["version_argv"] = tuple(fields["version_argv"])
    return cls(**fields)


def get(spec: str, home: Path | None = None, version: str | None = None) -> Agent:
    """The Agent for an ``agent.name`` (or ``name@version``, or a manifest
    path): manifest found, entry point imported. ``version`` is a
    config's ``agent.version``. ``home`` is accepted for call-site
    symmetry and unused: agents are bundled or passed as paths."""
    del home
    return compose(manifest(spec, version))


@dataclass(frozen=True)
class Listed:
    name: str
    path: Path
    agent: Agent | None    # None when the entry point failed to load
    error: str | None = None


def available() -> list[Listed]:
    """Every bundled agent, each composed in isolation: a broken entry
    point is listed with its error and hides nothing else."""
    out: list[Listed] = []
    for m in manifests():
        try:
            out.append(Listed(m.name, m.path, compose(m)))
        except Exception as exc:  # noqa: BLE001
            _log.warning("agent %s failed to load: %s", m.path, exc)
            out.append(Listed(m.name, m.path, None, error=str(exc)))
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
