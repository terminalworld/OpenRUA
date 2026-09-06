"""Layering contract, machine-enforced (see pyproject.toml block).

environment and ros_graph are leaves that never see each other (env/sim/cfg
flow as parameters); boot.py is the only module allowed to import both;
nobody host-side imports robocli.robot (container process, rclpy);
precheck and record are leaves (handles and paths are handed in).
Parsed from the AST; no import-linter dependency.
"""

from __future__ import annotations

import ast
from pathlib import Path

PKG = Path(__file__).resolve().parents[1] / "robocli"

_ONBOARD = {"robocli.robot.onboard"}
_HOST = {"robocli.bench.precheck", "robocli.bench.record", "robocli.sandbox",
         "robocli.proxy", "robocli.agents", "robocli.bench.run"}
# Onboard is bake-ready self-contained: nothing from robocli outside
# itself (the resolved config arrives as data), including the robot
# package's own ground-side verbs.
_GROUND_VERBS = {"robocli.robot.build", "robocli.robot.up",
                 "robocli.robot.down"}
_TOP = {"robocli.cli", "robocli.doctor", "robocli.testing"}
_HOST_LEAVES = {"robocli.config", "robocli.errors"}
_ONBOARD_BAN = _HOST | _GROUND_VERBS | _TOP | _HOST_LEAVES
_LAYERS = _ONBOARD | {"robocli.bench.precheck", "robocli.bench.record",
                      "robocli.sandbox"}
FORBIDDEN = {
    # inside onboard: the three halves never see each other; boot may
    # import all three; nothing imports outward
    "robot/onboard/environment": _ONBOARD_BAN | {
        "robocli.robot.onboard.ros_graph", "robocli.robot.onboard.monitor"},
    "robot/onboard/ros_graph": _ONBOARD_BAN | {
        "robocli.robot.onboard.environment", "robocli.robot.onboard.monitor"},
    "robot/onboard/monitor": _ONBOARD_BAN | {
        "robocli.robot.onboard.environment", "robocli.robot.onboard.ros_graph"},
    "robot/onboard/boot.py": _ONBOARD_BAN,
    # robot's ground verbs: consume data handed in by the conductor
    # and never see onboard
    "robot/build.py": _HOST | {"robocli.robot.onboard"},
    "robot/up.py": _HOST | {"robocli.robot.onboard"},
    "robot/down.py": _HOST | {"robocli.robot.onboard"},
    # host side: nobody imports onboard (container-only; rclpy); only
    # main conducts with the ground verbs
    "bench/run.py": _ONBOARD | _TOP,
    "sandbox": {"robocli.robot", "robocli.bench.precheck", "robocli.bench.record",
                "robocli.config"} | _TOP,
    "bench/precheck.py": {"robocli.robot", "robocli.sandbox", "robocli.proxy",
                    "robocli.agents", "robocli.bench.run", "robocli.config"} | _TOP,
    "bench/record.py": {"robocli.robot", "robocli.sandbox", "robocli.proxy",
                  "robocli.agents", "robocli.bench.run", "robocli.config"} | _TOP,
    "proxy": {"robocli.robot", "robocli.config"} | _LAYERS | _TOP,
    "agents": {"robocli.robot", "robocli.plugins"} | _LAYERS | _TOP,
    # hooks modules see the contract and nothing else of robocli
    "plugins": _HOST_LEAVES | _ONBOARD | _TOP | {
        "robocli.robot", "robocli.sandbox", "robocli.proxy", "robocli.bench",
        "robocli.agents.registry", "robocli.agents.launcher",
        "robocli.agents.credentials", "robocli.agents.prompts"},
    # the shared leaves are leaves
    "config": _HOST | {"robocli.robot"} | _TOP,
    "errors.py": _HOST | {"robocli.robot", "robocli.config"} | _TOP,
    # doctor sits with cli above the units; nothing below imports it
    "doctor.py": _ONBOARD | {"robocli.testing"},
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
