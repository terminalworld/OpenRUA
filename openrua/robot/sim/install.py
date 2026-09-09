"""Build a simulator install from its declaration: render, then run.

A simulator file (and a benchmark's ``install:`` over it) declares what
the bridge's venv is made of: repositories at pinned commits, a Python
version, a requirements lock, editable checkouts, and a shell tail for
what those cannot say. ``render`` turns that dict into one bash script
that ``openrua install`` prints, saves and runs; every step checks
before it acts, so rerunning is cheap. Nothing here reads a yaml: the
declaration arrives resolved (``config.install_for``), file paths
already absolute.

Needs ``uv`` (venvs and installs) and ``git`` on the host.
"""

from __future__ import annotations

import shlex
import subprocess
from pathlib import Path

from openrua import __version__
from openrua.errors import OpenRUAError

PACKAGE_GIT = "https://github.com/terminalworld/OpenRUA"

_HEADER = """#!/usr/bin/env bash
# Rendered by openrua install from the {what} declaration; rerunnable.
set -euo pipefail
ROOT={root}
VENV={venv}
PY={python}
note() {{ printf '    %s\\n' "$*"; }}
run()  {{ printf '    $ %s\\n' "$*"; "$@"; }}
command -v uv >/dev/null || {{ echo "uv is required: https://docs.astral.sh/uv/" >&2; exit 69; }}
command -v git >/dev/null || {{ echo "git is required" >&2; exit 69; }}
mkdir -p "$ROOT"

clone_at() {{  # clone_at DIR REPO COMMIT
  local dir="$1" repo="$2" commit="$3"
  [ -d "$dir/.git" ] || run git clone -q "$repo" "$dir"
  if [ "$(git -C "$dir" rev-parse HEAD)" != "$commit" ]; then
    git -C "$dir" fetch -q origin
    run git -C "$dir" checkout -q "$commit"
  fi
  note "$(basename "$dir") at $(git -C "$dir" rev-parse --short HEAD)"
}}

apply_patch() {{  # apply_patch DIR PATCH
  local dir="$1" patch="$2"
  if git -C "$dir" apply --check "$patch" 2>/dev/null; then
    run git -C "$dir" apply "$patch"
  elif git -C "$dir" apply --reverse --check "$patch" 2>/dev/null; then
    note "$(basename "$patch") already applied"
  else
    echo "$(basename "$patch") fits neither way against $dir; reset the files it touches and rerun" >&2
    exit 1
  fi
}}
"""


def render(install: dict, what: str, simulators_dir: Path, code_root: Path) -> str:
    """The install script for one resolved ``install:`` dict. ``what``
    names the declaration in the script's comment; ``simulators_dir``
    is where checkouts and relative venvs land; ``code_root`` is the
    directory holding the installed package (an editable install of it
    goes into the venv when that is a checkout, else the released
    package at this version)."""
    python = install.get("python")
    if not python:
        raise ValueError(f"{what}: install.python is not declared; openrua install needs "
                         "the venv's Python version (3.12 for jazzy, 3.10 for humble)")
    venv = Path(install["venv"]).expanduser()
    venv = venv if venv.is_absolute() else Path("$ROOT") / venv
    lines = [_HEADER.format(what=what, root=shlex.quote(str(simulators_dir)),
                            venv=_q(venv), python=shlex.quote(str(python)))]
    for c in install.get("checkouts") or []:
        d = Path("$ROOT") / c["path"]
        lines.append(f'clone_at {_q(d)} {shlex.quote(c["repo"])} {shlex.quote(c["commit"])}')
        if c.get("submodules"):
            subs = " ".join(shlex.quote(x) for x in c["submodules"])
            lines.append(f'run git -C {_q(d)} submodule update -q --init {subs}')
        if c.get("patch"):
            lines.append(f'apply_patch {_q(d)} {shlex.quote(c["patch"])}')
    lines.append('[ -x "$VENV/bin/python" ] || run uv venv -q "$VENV" --python "$PY"')
    if install.get("requirements"):
        lines.append(f'run uv pip install -q --python "$VENV/bin/python" -r '
                     f'{shlex.quote(install["requirements"])}')
    editables = [f"-e {_q(Path('$ROOT') / e)}" for e in install.get("editable") or []]
    if editables:
        # compat mode: a path line in a .pth, not setuptools' import hook,
        # which loses packages whose top-level directory holds a
        # same-named subpackage (the LIBERO forks: libero/libero).
        lines.append('CUDA_HOME="${CUDA_HOME:-/usr}" run uv pip install -q --python '
                     '"$VENV/bin/python" --no-deps --config-setting editable_mode=compat '
                     + " ".join(editables))
    if (code_root / "pyproject.toml").is_file():
        ours = f"-e {shlex.quote(str(code_root))}"
    else:
        ours = shlex.quote(f"openrua @ git+{PACKAGE_GIT}@v{__version__}")
    lines.append(f'run uv pip install -q --python "$VENV/bin/python" --no-deps {ours}')
    lines.append('note "$VENV: $("$VENV/bin/python" -V)"')
    if install.get("shell"):
        tail = install["shell"].replace("{root}", '"$ROOT"').replace("{venv}", '"$VENV"')
        lines.append("# the declaration's shell tail")
        lines.append(tail.rstrip("\n"))
    lines.append('note "done"')
    return "\n".join(lines) + "\n"


def _q(p: Path) -> str:
    """Quote a path that may start with ``$ROOT`` (the variable expands
    inside double quotes)."""
    s = str(p)
    if s.startswith("$ROOT"):
        return '"' + s.replace('"', '\\"') + '"'
    return shlex.quote(s)


def run(script: Path) -> None:
    """Run a rendered script with bash; its output is the user's. A
    failing step ends the script (set -e), and the error names it."""
    r = subprocess.run(["bash", str(script)])
    if r.returncode != 0:
        raise OpenRUAError(f"install script failed (exit {r.returncode}): {script}",
                           hint="the failing step is the last command printed above; fix "
                                f"it and rerun, or run bash {script} by hand")
