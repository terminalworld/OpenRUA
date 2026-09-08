"""Layering contract, machine-enforced (see pyproject.toml block).

Inside the bridge, environments, ros and rpc never see each other
(env, worker and cfg flow as parameters); main.py is the only module
allowed to import all three; nothing host-side imports the bridge
(container process, rclpy); preflight and record are leaves (handles and
paths are handed in).
Parsed from the AST; no import-linter dependency.
"""

from __future__ import annotations

import ast
from pathlib import Path

PKG = Path(__file__).resolve().parents[2] / "openrua"

_BRIDGE = {"openrua.robot.sim.bridge"}
_HOST = {"openrua.runner.preflight", "openrua.runner.record", "openrua.sandbox",
         "openrua.proxy", "openrua.agents", "openrua.runner"}
# The bridge is self-contained: nothing from openrua outside itself (the
# resolved config arrives as data), including the robot package's own
# host side.
_ROBOT_HOST = {"openrua.robot.base", "openrua.robot.real", "openrua.robot.sim.build",
               "openrua.robot.sim.up", "openrua.robot.sim.down", "openrua.robot.sim.client"}
_TOP = {"openrua.cli", "openrua.doctor", "openrua.testing"}
_HOST_LEAVES = {"openrua.config", "openrua.errors"}
_BRIDGE_BAN = _HOST | _ROBOT_HOST | _TOP | _HOST_LEAVES
_LAYERS = _BRIDGE | {"openrua.runner.preflight", "openrua.runner.record",
                     "openrua.sandbox"}
FORBIDDEN = {
    # inside the bridge: the three parts never see each other; main may
    # import all three; nothing imports outward
    "robot/sim/bridge/environments": _BRIDGE_BAN | {
        "openrua.robot.sim.bridge.ros", "openrua.robot.sim.bridge.rpc"},
    "robot/sim/bridge/ros": _BRIDGE_BAN | {
        "openrua.robot.sim.bridge.environments", "openrua.robot.sim.bridge.rpc"},
    "robot/sim/bridge/rpc.py": _BRIDGE_BAN | {
        "openrua.robot.sim.bridge.environments", "openrua.robot.sim.bridge.ros"},
    "robot/sim/bridge/main.py": _BRIDGE_BAN,
    # the robot's host side consumes data handed in by the runner and
    # never sees the bridge, the config schema or the other units
    "robot/__init__.py": _HOST | _BRIDGE | {"openrua.config"} | _TOP,
    "robot/base.py": _HOST | _BRIDGE | {"openrua.config"} | _TOP,
    "robot/sim/build.py": _HOST | _BRIDGE | {"openrua.config"} | _TOP,
    "robot/sim/up.py": _HOST | _BRIDGE | {"openrua.config"} | _TOP,
    "robot/sim/down.py": _HOST | _BRIDGE | {"openrua.config"} | _TOP,
    "robot/sim/client.py": _HOST | _BRIDGE | {"openrua.config"} | _TOP,
    "robot/real": _HOST | _BRIDGE | {"openrua.config"} | _TOP,
    # host side: nobody imports the bridge (container-only; rclpy); only
    # the runner conducts with the robot package
    "runner/main.py": _BRIDGE | _TOP,
    "sandbox": {"openrua.robot", "openrua.runner.preflight", "openrua.runner.record",
                "openrua.config"} | _TOP,
    "runner/preflight.py": {"openrua.robot", "openrua.sandbox", "openrua.proxy",
                    "openrua.agents", "openrua.runner", "openrua.config"} | _TOP,
    "runner/record.py": {"openrua.robot", "openrua.sandbox", "openrua.proxy",
                  "openrua.agents", "openrua.runner", "openrua.config"} | _TOP,
    "proxy": {"openrua.robot", "openrua.config"} | _LAYERS | _TOP,
    "agents": {"openrua.robot", "openrua.plugins"} | _LAYERS | _TOP,
    # hooks modules see the contract and nothing else of openrua
    "plugins": _HOST_LEAVES | _BRIDGE | _TOP | {
        "openrua.robot", "openrua.sandbox", "openrua.proxy", "openrua.runner",
        "openrua.agents.registry", "openrua.agents.launcher",
        "openrua.agents.credentials", "openrua.agents.prompts"},
    # the shared leaves are leaves
    "config": _HOST | {"openrua.robot"} | _TOP,
    "errors.py": _HOST | {"openrua.robot", "openrua.config"} | _TOP,
    # demo renders a trial's files; it sees errors and nothing else
    "demo": _HOST | _BRIDGE | _TOP | {"openrua.robot", "openrua.config"},
    # doctor sits with cli above the units; nothing below imports it
    "doctor": _BRIDGE | {"openrua.testing"},
    "cli": _BRIDGE | {"openrua.testing"},
}


def _imports(path: Path) -> set[str]:
    out: set[str] = set()
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            out.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:  # relative: resolve against the module's package
                pkg_parts = path.relative_to(PKG.parent).parts[:-1]
                base = ".".join(pkg_parts[: len(pkg_parts) - node.level + 1])
                out.add(f"{base}.{node.module}" if node.module else base)
            elif node.module:
                out.add(node.module)
                out.update(f"{node.module}.{a.name}" for a in node.names)
    return out


def test_layering_contract():
    violations = []
    for layer, banned in FORBIDDEN.items():
        target = PKG / layer
        files = [target] if target.is_file() else target.rglob("*.py")
        for py in files:
            if "__pycache__" in py.parts:
                continue
            for imp in _imports(py):
                if any(imp == b or imp.startswith(b + ".") for b in banned):
                    violations.append(
                        f"{py.relative_to(PKG.parent)} imports {imp}")
    assert not violations, "\n".join(violations)
