"""Environment loaders: the plug shape, machine-enforced.

Every bundled benchmark's entry_point (and the simulator's native one)
resolves to a module exposing LOADER with every name in
LOADER_INTERFACE; a module without LOADER or with a hole in it fails
loud. No simulator boots here; simulator imports live inside create().
"""

from __future__ import annotations

import pytest
import yaml

from openrua.config import paths
from openrua.robot.sim.bridge import environments as environment


def _bundled_entry_points():
    for e in paths.available("benchmarks"):
        yield paths.entry_point("benchmarks", yaml.safe_load(e.path.read_text())["entry_point"], e.path)
    for e in paths.available("simulators"):
        native = yaml.safe_load(e.path.read_text()).get("native")
        if native:
            yield paths.entry_point("benchmarks", native["entry_point"], e.path)


def test_every_bundled_entry_point_loads_a_full_loader():
    specs = list(_bundled_entry_points())
    assert len(specs) >= 4
    for spec in specs:
        loader = environment.load(spec)
        for name in environment.LOADER_INTERFACE:
            assert hasattr(loader, name), f"{spec} missing {name}"


def test_a_loader_file_of_your_own_and_the_holes_it_can_have(tmp_path):
    good = tmp_path / "good.py"
    good.write_text("class L:\n" + "".join(
        f"    def {n}(self, *a, **k): pass\n" for n in environment.LOADER_INTERFACE) + "LOADER = L()\n")
    assert hasattr(environment.load(str(good)), "success")
    (tmp_path / "none.py").write_text("x = 1\n")
    with pytest.raises(ValueError, match="no LOADER"):
        environment.load(str(tmp_path / "none.py"))
    (tmp_path / "hole.py").write_text("class L:\n    def create(self): pass\nLOADER = L()\n")
    with pytest.raises(ValueError, match="lacks"):
        environment.load(str(tmp_path / "hole.py"))
    with pytest.raises(ModuleNotFoundError):
        environment.load("openrua.robot.sim.bridge.environments.no_such")


def test_robocerebra_cases_are_numbered_and_goals_parsed(tmp_path):
    from openrua.robot.sim.bridge.environments import robocerebra
    root = tmp_path / "RoboCerebra_Bench"
    for n in (1, 10, 2):
        d = root / "Ideal" / f"case{n}"
        d.mkdir(parents=True)
        (d / "scene.bddl").write_text("")
    (root / "Ideal" / "notes").mkdir()                       # no bddl: not a case
    cases = robocerebra._cases(root, "Ideal")
    assert [c.name for c in cases] == ["case1", "case2", "case10"]   # numeric, not lexical
    with pytest.raises(FileNotFoundError, match="have: Ideal"):
        robocerebra._cases(root, "Mix")
    assert robocerebra._bench_root({"task": {"dataset_root": str(root)}}) == root
