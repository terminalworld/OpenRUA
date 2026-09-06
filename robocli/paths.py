"""Where things live: the user directory and how named things are found.

Two places hold data, in a fixed lookup order:

1. bundled, inside the installed package (``robocli/robots/*.yaml``,
   ``robocli/benchmarks/*.yaml``, ``robocli/agents/*.py``);
2. the user directory, ``~/.robocli`` unless overridden, with one
   subdirectory per kind (``robots/``, ``benchmarks/``, ``agents/``) plus
   what the tool keeps for the user (``credentials/``, ``substrates/``,
   ``workspaces/``, ``state/``, ``config.yaml``).

A name is looked up bundled first, then in the user directory; a path
(anything with a slash or a suffix) is taken as is. A user file carrying
a bundled name does not win: the bundled one is used and ``available()``
reports the pair as shadowed, so ``robocli doctor`` can say so.

The user directory reaches this module as a parameter (``home=``); the
CLI entry point reads ``ROBOCLI_HOME`` and ``--home`` once and passes the
result down. Nothing here reads the environment. A third source, pip
entry points, would slot in after the user directory; not implemented.

Stdlib-only leaf: every host-side unit may import this. Nothing in
``robocli.robot.onboard`` does; the container reads resolved absolute
paths from the assembly file.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import resources
from pathlib import Path

DEFAULT_HOME = "~/.robocli"
CONFIG_FILENAME = "config.yaml"

# kind -> file suffix of one named entry; the kinds a name can be looked up in
KINDS = {"robots": ".yaml", "benchmarks": ".yaml", "agents": ".py"}


def home(override: str | Path | None = None) -> Path:
    """The user directory: the override if given, else ``~/.robocli``."""
    return Path(override or DEFAULT_HOME).expanduser()


def config_path(home_dir: Path | None = None) -> Path:
    return home(home_dir) / CONFIG_FILENAME


def robots_dir(home_dir: Path | None = None) -> Path:
    return home(home_dir) / "robots"


def benchmarks_dir(home_dir: Path | None = None) -> Path:
    return home(home_dir) / "benchmarks"


def agents_dir(home_dir: Path | None = None) -> Path:
    return home(home_dir) / "agents"


def credentials_dir(home_dir: Path | None = None) -> Path:
    """Login profiles, one subdirectory per agent name."""
    return home(home_dir) / "credentials"


def substrates_dir(home_dir: Path | None = None) -> Path:
    """Simulator checkouts (each holding its ``.venv-*``)."""
    return home(home_dir) / "substrates"


def workspaces_dir(home_dir: Path | None = None) -> Path:
    """Working directories of robots brought up with ``robocli up``."""
    return home(home_dir) / "workspaces"


def state_dir(home_dir: Path | None = None) -> Path:
    """What ``robocli up`` remembers about a live robot, one file per name."""
    return home(home_dir) / "state"


def code_root() -> Path:
    """The directory holding the installed ``robocli`` package: the git
    checkout under an editable install, site-packages under a wheel.
    Bind-mounted into the simulated body so its venv can import the same
    code; also where provenance looks for a git commit."""
    return Path(str(resources.files("robocli"))).resolve().parent


def bundled(kind: str) -> Path:
    """The package directory shipping the entries of one kind."""
    if kind not in KINDS:
        raise KeyError(f"unknown kind {kind!r} (kinds: {', '.join(KINDS)})")
    return Path(str(resources.files("robocli"))) / kind


def user_dir(kind: str, home_dir: Path | None = None) -> Path:
    if kind not in KINDS:
        raise KeyError(f"unknown kind {kind!r} (kinds: {', '.join(KINDS)})")
    return home(home_dir) / kind


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
            raise FileNotFoundError(f"{kind[:-1]} file not found: {p}")
        return p
    suffix = KINDS[kind]
    for base in (bundled(kind), user_dir(kind, home_dir)):
        p = base / f"{s}{suffix}"
        if p.is_file():
            return p
    names = ", ".join(e.name for e in available(kind, home_dir)) or "(none)"
    raise FileNotFoundError(
        f"no {kind[:-1]} named {s!r} (available: {names}); "
        f"add one under {user_dir(kind, home_dir)}/ or pass a path")


def substrate_venv(spec: str | Path, home_dir: Path | None = None) -> Path:
    """The simulator venv a robot profile names: absolute or ``~`` paths
    as written, anything else relative to ``<home>/substrates/``."""
    p = Path(spec).expanduser()
    return p if p.is_absolute() else substrates_dir(home_dir) / p


def substrate_root(venv: Path) -> Path:
    """The substrate checkout holding a venv (the directory above
    ``.venv-*``)."""
    return venv.parent
