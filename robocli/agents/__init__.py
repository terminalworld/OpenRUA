"""Agents: the occupant package (adapters + launcher + opening prompt).

The harness is agent-agnostic by construction: every fact about a
particular coding agent lives in one adapter (a ``base.Agent``
subclass), and every consumer reaches it through ``get()``. Adding an
agent is one module exposing ``AGENT``: bundled here, or in the user
directory (``~/.robocli/agents/<name>.py``), looked up in that order.
A third source, pip entry points (group ``robocli.agents``), would come
after the user directory; not implemented.

Selection: configs carry ``agent.name`` (the package default lives in
``configs/config.yaml``), recorded per trial in ``operator_meta.agent`` so
post-hoc tools resolve the adapter the trial actually ran.
"""

from __future__ import annotations

import importlib
import importlib.util
import logging
import shutil as _shutil
import subprocess as _subprocess
import sys
import tempfile as _tempfile
from dataclasses import dataclass
from pathlib import Path as _Path

from robocli.config import paths
from robocli.agents.base import HOOKS, Agent, Credentials  # noqa: F401  re-exported

_log = logging.getLogger(__name__)

# Modules in this package that are not adapters.
_NOT_ADAPTERS = {"base", "launcher"}


def _module_name(name: str) -> str:
    return name.replace("-", "_")


def _load_user_module(path: _Path):
    """Import one user-directory adapter file under a private module
    name, so two users' files (or a user's and ours) never collide.
    A failure is logged and re-raised; the half-initialised module is
    removed so a retry starts clean."""
    modname = f"_robocli_user_agent_{path.stem}"
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


def _agent_of(module, source: str) -> Agent:
    agent = getattr(module, "AGENT", None)
    if not isinstance(agent, Agent):
        raise TypeError(f"{source} exposes no AGENT (an robocli.agents.Agent instance)")
    return agent


def get(name: str, home: _Path | None = None) -> Agent:
    """Resolve an adapter by its ``agent.name``: bundled first, then
    ``<home>/agents/<name>.py``. Unknown names list what exists."""
    if not name:
        raise ValueError("agent name is empty; a resolved config always carries agent.name")
    mod = _module_name(name)
    if mod not in _NOT_ADAPTERS:
        try:
            return _agent_of(importlib.import_module(f"robocli.agents.{mod}"),
                             f"robocli.agents.{mod}")
        except ModuleNotFoundError as e:
            if e.name != f"robocli.agents.{mod}":
                raise
    for candidate in (paths.agents_dir(home) / f"{name}.py",
                      paths.agents_dir(home) / f"{mod}.py"):
        if candidate.is_file():
            return _agent_of(_load_user_module(candidate), str(candidate))
    names = ", ".join(e.name for e in available(home))
    raise KeyError(f"unknown agent {name!r} (available: {names}); "
                   f"add one under {paths.agents_dir(home)}/ exposing AGENT")


@dataclass(frozen=True)
class Listed:
    name: str
    source: str            # "bundled" | "user"
    path: _Path
    agent: Agent | None    # None when the module failed to load
    error: str | None = None
    shadowed_by: _Path | None = None


def available(home: _Path | None = None) -> list[Listed]:
    """Every adapter, bundled then user, each loaded in isolation: a
    broken user file is listed with its error and hides nothing else."""
    out: list[Listed] = []
    for e in paths.available("agents", home):
        if e.source == "bundled" and e.name in _NOT_ADAPTERS:
            continue
        try:
            if e.source == "bundled":
                agent = _agent_of(importlib.import_module(f"robocli.agents.{e.name}"),
                                  str(e.path))
            else:
                agent = _agent_of(_load_user_module(e.path), str(e.path))
            out.append(Listed(agent.name, e.source, e.path, agent,
                              shadowed_by=e.shadowed_by))
        except Exception as exc:  # noqa: BLE001  one bad adapter must not hide the rest
            _log.warning("agent adapter %s failed to load: %s", e.path, exc)
            out.append(Listed(e.name, e.source, e.path, None, error=str(exc),
                              shadowed_by=e.shadowed_by))
    return out


def preinstall(agents: list[Agent]) -> str:
    """One shell chain installing every agent's CLI (the sandbox image's
    PREINSTALL slot). Adapters with nothing to install contribute nothing."""
    return " && ".join(a.install for a in agents if a.install)


def whitelist(agents: list[Agent]) -> tuple[str, ...]:
    """The union of the agents' proxy whitelists, first occurrence order."""
    seen: list[str] = []
    for a in agents:
        for line in a.whitelist:
            if line not in seen:
                seen.append(line)
    return tuple(seen)


# THE opening prompt, inline by ruling 2026-08-15: it is stable skin, a
# separate document earns nothing. Single source shared by the launcher
# (what the agent sees) and the evaluator's provenance hash (audit
# 2026-08-14 F18: divergent sources would pin the hash of a prompt the
# agent never saw). {task} is the only placeholder (guarded by test).
PROMPT = """\
You are working on a robot's onboard computer.

Your task: {task}

Survey the machine yourself to find out what robot this is and what it
can do; work until the task is physically done, verify it your own way,
then finish.

The workspace contains starter docs and tools you can use.
"""

# The only thing said to an agent whose trial was suspended at a quota wall
# and is being resumed (ruling 2026-08-20). Deliberately content-free: the
# session already holds the task, the workspace, and everything the agent
# has done, so restating any of it would hand a resumed trial context an
# uninterrupted one never got. It cannot be made to disappear entirely -- a
# resumed trial receives one more user turn than an uninterrupted one, and
# that structural difference is disclosed rather than hidden.
RESUME_PROMPT = "Continue where you left off."


def prepare_profile(creds_home: _Path, adapter: Agent,
                    require_credentials: bool = True
                    ) -> tuple[_Path, _Path | None]:
    """Stage the occupant's luggage: auth profile for one sandbox entry.

    The profile dir (settings and other non-rotating state) is always a
    fresh per-entry copy. What happens to the credentials file depends on
    how the sandbox authenticates:

    - ``require_credentials=True`` is the shared-file arrangement
      (post-incident 2026-08-08): OAuth refresh tokens ROTATE, so every
      consumer must read/write the SAME file or the forks kill each other.
      The file is returned as a path for bind-mounting, never copied.
    - ``require_credentials=False`` is the minted-token arrangement
      (2026-09-02): the sandbox carries its own token, so no credentials
      file is needed or wanted, and the second element comes back None.

    An adapter without a profile-directory login gets an empty copy dir
    and None. Returns (profile_copy_dir, shared_file_or_None); the caller
    owns deleting the copy dir.
    """
    cfg_dir = _Path(_tempfile.mkdtemp(prefix="robocli-agentcfg-"))
    creds = adapter.credentials
    if creds is None:
        return cfg_dir, None
    creds_file: _Path | None = creds_home / creds.filename
    if not require_credentials:
        creds_file = None
    elif not creds_file.exists():
        raise RuntimeError(
            f"credentials missing: {adapter.login_hint(creds_home)}")
    for pattern in ("*.json", ".*.json"):
        for f in creds_home.glob(pattern):
            if f.name != creds.filename:
                _shutil.copy2(f, cfg_dir / f.name)
    _subprocess.run(["chmod", "-R", "777", str(cfg_dir)])
    return cfg_dir, creds_file
