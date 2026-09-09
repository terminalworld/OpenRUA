"""How the bridge finds the code a resolved config names.

A ``loader`` or ``engine`` line in the resolved config is either a
module path (``openrua.robot.sim.bridge.environments.libero``) or an
absolute ``.py`` file (a benchmark or engine of your own, copied next
to the config). ``module(spec)`` imports either; ``check(obj,
interface, spec)`` verifies the plug shape. environments and engines
both use this leaf; it knows neither.
"""

from __future__ import annotations

import importlib
import importlib.util
import sys
from pathlib import Path


def module(spec: str):
    """The module a spec names: imported by path when it is a ``.py``
    file, by name otherwise."""
    if spec.endswith(".py"):
        return _import_file(Path(spec))
    return importlib.import_module(spec)


def check(obj, interface: tuple[str, ...], spec: str, what: str) -> None:
    """Loud when ``obj`` lacks a name of the interface."""
    missing = [n for n in interface if not hasattr(obj, n)]
    if missing:
        raise ValueError(f"{spec}: {what} lacks {', '.join(missing)}")


def _import_file(path: Path):
    """Import one module by file path under a private name; a failure
    removes the half-initialised module so a retry starts clean."""
    modname = f"_openrua_plug_{path.stem}_{abs(hash(str(path)))}"
    if modname in sys.modules:
        return sys.modules[modname]
    spec = importlib.util.spec_from_file_location(modname, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[modname] = mod
    try:
        spec.loader.exec_module(mod)
    except Exception:
        sys.modules.pop(modname, None)
        raise
    return mod
