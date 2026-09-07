"""Where things live: the user directory and how named things are found.

Two places hold data, in a fixed lookup order:

1. bundled, inside the installed package: declarative files under
   ``robocli/configs/<kind>/`` (``robots/``, ``benchmarks/``, ``agents/``),
   extension code under ``robocli/plugins/<kind>/`` (``agents/``);
2. the user directory, ``~/.robocli`` unless overridden, with the same
   shape (``robots/``, ``benchmarks/``, ``agents/``, ``plugins/<kind>/``)
   plus what the tool keeps for the user (``credentials/``,
   ``simulators/``, ``workspaces/``, ``state/``, ``config.yaml``).

A name is looked up bundled first, then in the user directory; a path
(anything with a slash or a suffix) is taken as is. A user file carrying
a bundled name does not win: the bundled one is used and ``available()``
reports the pair as shadowed, so ``robocli doctor`` can say so.

The user directory reaches this module as a parameter (``home=``); the
CLI entry point reads ``ROBOCLI_HOME`` and ``--home`` once and passes the
result down. Nothing here reads the environment. A third source, pip
entry points, would slot in after the user directory; not implemented.

Imports only robocli.errors. Nothing in ``robocli.robot.sim.bridge``
imports this; the container reads absolute paths from the resolved
config file.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import resources
from pathlib import Path

from robocli.errors import NotFound

DEFAULT_HOME = "~/.robocli"
CONFIG_FILENAME = "config.yaml"

# Declarative kinds (one yaml per named entry) and the plugin kinds (one
# python module per entry); a name is looked up by kind.
KINDS = {"robots": ".yaml", "benchmarks": ".yaml", "agents": ".yaml"}
PLUGIN_KINDS = {"agents": ".py"}


def home(override: str | Path | None = None) -> Path:
    """The user directory: the override if given, else ``~/.robocli``."""
    return Path(override or DEFAULT_HOME).expanduser()


def config_path(home_dir: Path | None = None) -> Path:
    """The user's defaults file."""
    return home(home_dir) / CONFIG_FILENAME


def package_config_path() -> Path:
    """The package defaults file (``robocli/configs/config.yaml``), the
    bottom layer under the user's file and the benchmark config."""
    return Path(str(resources.files("robocli"))) / "configs" / CONFIG_FILENAME





def plugins_dir(home_dir: Path | None = None) -> Path:
    return home(home_dir) / "plugins"


def credentials_dir(home_dir: Path | None = None) -> Path:
    """Login profiles, one subdirectory per agent name."""
    return home(home_dir) / "credentials"


def simulators_dir(home_dir: Path | None = None) -> Path:
    """Simulator checkouts (each holding its ``.venv-*``)."""
    return home(home_dir) / "simulators"


def workspaces_dir(home_dir: Path | None = None) -> Path:
    """Working directories of robots brought up with ``robocli up``."""
    return home(home_dir) / "workspaces"


def state_dir(home_dir: Path | None = None) -> Path:
    """What ``robocli up`` remembers about a live robot, one file per name."""
    return home(home_dir) / "state"


def code_root() -> Path:
    """The directory holding the installed ``robocli`` package: the git
    checkout under an editable install, site-packages under a wheel.
    Bind-mounted into the simulated robot so its venv can import the same
    code; also where provenance looks for a git commit."""
    return Path(str(resources.files("robocli"))).resolve().parent


def bundled(kind: str) -> Path:
    """The package directory shipping the declarative entries of one kind."""
    if kind not in KINDS:
        raise KeyError(f"unknown kind {kind!r} (kinds: {', '.join(KINDS)})")
    return Path(str(resources.files("robocli"))) / "configs" / kind


def user_dir(kind: str, home_dir: Path | None = None) -> Path:
    """The user directory holding the declarative entries of one kind."""
    if kind not in KINDS:
        raise KeyError(f"unknown kind {kind!r} (kinds: {', '.join(KINDS)})")
    return home(home_dir) / kind


def configs(kind: str, home_dir: Path | None = None) -> tuple[Path, Path]:
    """(bundled, user) directories of one declarative kind, lookup order."""
    return bundled(kind), user_dir(kind, home_dir)


def bundled_plugins(kind: str) -> Path:
    """The package directory shipping the plugin modules of one kind."""
    if kind not in PLUGIN_KINDS:
        raise KeyError(f"unknown plugin kind {kind!r} (kinds: {', '.join(PLUGIN_KINDS)})")
    return Path(str(resources.files("robocli"))) / "plugins" / kind


def user_plugins(kind: str, home_dir: Path | None = None) -> Path:
    """The user directory holding the plugin modules of one kind."""
    if kind not in PLUGIN_KINDS:
        raise KeyError(f"unknown plugin kind {kind!r} (kinds: {', '.join(PLUGIN_KINDS)})")
    return plugins_dir(home_dir) / kind


def plugins(kind: str, home_dir: Path | None = None) -> tuple[Path, Path]:
    """(bundled, user) directories of one plugin kind, lookup order."""
    return bundled_plugins(kind), user_plugins(kind, home_dir)


@dataclass(frozen=True)
class Entry:
    name: str
    path: Path
    source: str                        # "bundled" | "user"
    shadowed_by: Path | None = None    # a user file of the same name, ignored


def available(kind: str, home_dir: Path | None = None) -> list[Entry]:
    """Every named entry of a kind: bundled first, then the user's.
    A user file whose name matches a bundled one is not its own entry;
    it appears as ``shadowed_by`` on the bundled one."""
    suffix = KINDS[kind]
    ours = _named(bundled(kind), suffix)
    theirs = _named(user_dir(kind, home_dir), suffix)
    out = [Entry(n, p, "bundled", theirs.get(n)) for n, p in ours.items()]
    out += [Entry(n, p, "user") for n, p in theirs.items() if n not in ours]
    return out


def _named(base: Path, suffix: str) -> dict[str, Path]:
    if not base.is_dir():
        return {}
    return {p.stem: p for p in sorted(base.glob(f"*{suffix}"))
            if not p.name.startswith(("_", "."))}


def find(kind: str, name_or_path: str | Path, home_dir: Path | None = None) -> Path:
    """Resolve a name (bundled, then the user directory) or a path (as
    given). Raises FileNotFoundError naming what is available."""
    s = str(name_or_path)
    if "/" in s or Path(s).suffix:
        p = Path(s).expanduser()
        if not p.is_file():
            raise NotFound(f"{kind[:-1]} file not found: {p}")
        return p
    suffix = KINDS[kind]
    for base in (bundled(kind), user_dir(kind, home_dir)):
        p = base / f"{s}{suffix}"
        if p.is_file():
            return p
    names = ", ".join(e.name for e in available(kind, home_dir)) or "(none)"
    raise NotFound(f"no {kind[:-1]} named {s!r} (available: {names})",
                   hint=f"robocli {kind} lists them; add your own under "
                   f"{user_dir(kind, home_dir)}/ or pass a path")


def simulator_venv(spec: str | Path, home_dir: Path | None = None) -> Path:
    """The simulator venv a robot profile names: absolute or ``~`` paths
    as written, anything else relative to ``<home>/simulators/``."""
    p = Path(spec).expanduser()
    return p if p.is_absolute() else simulators_dir(home_dir) / p


def simulator_root(venv: Path) -> Path:
    """The simulator checkout holding a venv (the directory above
    ``.venv-*``)."""
    return venv.parent
