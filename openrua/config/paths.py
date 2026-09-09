"""Where things live: the bundled files, the user directory, entry points.

Bundled files ship inside the installed package: declarative files under
``openrua/configs/<kind>/`` (``robots/``, ``simulators/``, ``benchmarks/``,
``agents/``) and, for the kinds that need code, one module per entry in
the package ``ENTRY_POINT_PACKAGES`` names. A short name resolves there
and nowhere else; anything else (a slash or a suffix) is a path to a
file of your own. There is no lookup in the user directory: extensions
enter the repository or travel as a path.

The user directory, ``~/.openrua`` unless overridden, holds only what
the tool writes: ``config.yaml`` (openrua config set), ``credentials/``
(logins), ``simulators/`` (openrua install), ``workspaces/``, ``state/``.
It reaches this module as a parameter (``home=``); the CLI entry point
reads ``OPENRUA_HOME`` and ``--home`` once and passes the result down.
Nothing here reads the environment.

Imports only openrua.errors. Nothing in ``openrua.robot.sim.bridge``
imports this; the container reads absolute paths from the resolved
config file.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import resources
from pathlib import Path

from openrua.errors import NotFound

DEFAULT_HOME = "~/.openrua"
CONFIG_FILENAME = "config.yaml"

# The declarative kinds, one yaml per named entry.
KINDS = ("robots", "simulators", "benchmarks", "agents")
# Kinds whose yaml names code under ``entry_point:``, and the package a
# short name resolves in (``entry_point: foo`` -> openrua.plugins.agents.foo).
ENTRY_POINT_PACKAGES = {
    "agents": "openrua.plugins.agents",
    "benchmarks": "openrua.robot.sim.bridge.environments",
}


def home(override: str | Path | None = None) -> Path:
    """The user directory: the override if given, else ``~/.openrua``."""
    return Path(override or DEFAULT_HOME).expanduser()


def config_path(home_dir: Path | None = None) -> Path:
    """The user's defaults file."""
    return home(home_dir) / CONFIG_FILENAME


def package_config_path() -> Path:
    """The package defaults file (``openrua/configs/config.yaml``), the
    bottom layer under the user's file and the benchmark config."""
    return Path(str(resources.files("openrua"))) / "configs" / CONFIG_FILENAME





def credentials_dir(home_dir: Path | None = None) -> Path:
    """Login profiles, one subdirectory per agent name."""
    return home(home_dir) / "credentials"


def simulators_dir(home_dir: Path | None = None) -> Path:
    """Simulator checkouts (each holding its ``.venv-*``)."""
    return home(home_dir) / "simulators"


def workspaces_dir(home_dir: Path | None = None) -> Path:
    """Working directories of robots brought up with ``openrua up``."""
    return home(home_dir) / "workspaces"


def state_dir(home_dir: Path | None = None) -> Path:
    """What ``openrua up`` remembers about a live robot, one file per name."""
    return home(home_dir) / "state"


def code_root() -> Path:
    """The directory holding the installed ``openrua`` package: the git
    checkout under an editable install, site-packages under a wheel.
    Bind-mounted into the simulated robot so its venv can import the same
    code; also where provenance looks for a git commit."""
    return Path(str(resources.files("openrua"))).resolve().parent


def bundled(kind: str) -> Path:
    """The package directory shipping the declarative entries of one kind."""
    if kind not in KINDS:
        raise KeyError(f"unknown kind {kind!r} (kinds: {', '.join(KINDS)})")
    return Path(str(resources.files("openrua"))) / "configs" / kind


@dataclass(frozen=True)
class Entry:
    name: str
    path: Path


def available(kind: str) -> list[Entry]:
    """Every bundled entry of a kind, by name."""
    base = bundled(kind)
    return [Entry(p.stem, p) for p in sorted(base.glob("*.yaml"))
            if not p.name.startswith(("_", "."))]


def is_path(spec: str) -> bool:
    """A slash or a suffix makes a spec a path; a bare word is a name."""
    return "/" in spec or bool(Path(spec).suffix)


def find(kind: str, name_or_path: str | Path) -> Path:
    """Resolve a bundled name, or a path as given. Raises NotFound naming
    what is bundled."""
    s = str(name_or_path)
    if is_path(s):
        p = Path(s).expanduser()
        if not p.is_file():
            raise NotFound(f"{kind[:-1]} file not found: {p}")
        return p
    p = bundled(kind) / f"{s}.yaml"
    if p.is_file():
        return p
    names = ", ".join(e.name for e in available(kind)) or "(none)"
    raise NotFound(f"no {kind[:-1]} named {s!r} (bundled: {names})",
                   hint=f"openrua {kind} lists them; a file of your own is passed "
                   "as a path (./my-file.yaml)")


def entry_point(kind: str, spec: str, declared_in: Path) -> str:
    """The code a declaration names under ``entry_point:``, resolved to
    what an importer needs: a short name becomes the module under the
    kind's package (``foo`` -> ``openrua.plugins.agents.foo``); a path
    is taken relative to the yaml that names it and returned absolute.
    The module exposes the kind's conventional object (agents:
    ``HOOKS``, benchmarks: ``LOADER``)."""
    if kind not in ENTRY_POINT_PACKAGES:
        raise KeyError(f"{kind} declarations carry no entry_point")
    if is_path(spec):
        p = Path(spec).expanduser()
        if not p.is_absolute():
            p = Path(declared_in).expanduser().resolve().parent / p
        if not p.is_file():
            raise NotFound(f"entry_point {spec!r} of {declared_in}: file not found at {p}")
        return str(p.resolve())
    return f"{ENTRY_POINT_PACKAGES[kind]}.{spec}"


def simulator_venv(spec: str | Path, home_dir: Path | None = None) -> Path:
    """The simulator venv a robot profile names: absolute or ``~`` paths
    as written, anything else relative to ``<home>/simulators/``."""
    p = Path(spec).expanduser()
    return p if p.is_absolute() else simulators_dir(home_dir) / p


def simulator_root(venv: Path) -> Path:
    """The simulator checkout holding a venv (the directory above
    ``.venv-*``)."""
    return venv.parent
