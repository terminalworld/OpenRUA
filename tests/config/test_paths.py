"""paths: the user directory, the bundled lookup, entry points."""
from pathlib import Path

import pytest

from openrua.config import paths
from openrua.errors import NotFound


def test_home_defaults_and_overrides(tmp_path):
    assert paths.home() == Path("~/.openrua").expanduser()
    assert paths.home(tmp_path) == tmp_path
    assert paths.credentials_dir(tmp_path) == tmp_path / "credentials"
    assert paths.simulators_dir(tmp_path) == tmp_path / "simulators"
    assert paths.config_path(tmp_path) == tmp_path / "config.yaml"


def test_bundled_entries_ship_in_the_package():
    assert {"panda", "panda-omron"} <= {e.name for e in paths.available("robots")}
    assert {e.name for e in paths.available("simulators")} >= {"robosuite"}
    assert {e.name for e in paths.available("benchmarks")} >= {
        "libero_pro", "capbench", "robocasa365"}
    assert {e.name for e in paths.available("agents")} >= {"claude-code", "codex"}


def test_find_takes_a_bundled_name_or_a_path(tmp_path):
    bundled = paths.find("robots", "panda")
    assert bundled.name == "panda.yaml" and bundled.is_file()
    mine = tmp_path / "my-arm.yaml"
    mine.write_text("machine: {}\n")
    assert paths.find("robots", str(mine)) == mine
    assert paths.find("robots", mine) == mine
    with pytest.raises(NotFound, match="gone.yaml"):
        paths.find("robots", str(tmp_path / "gone.yaml"))


def test_missing_name_says_what_is_bundled_and_how_to_pass_your_own():
    with pytest.raises(NotFound) as e:
        paths.find("robots", "nope")
    assert "panda" in str(e.value) and "path" in e.value.hint


def test_entry_point_short_name_or_path_relative_to_the_yaml(tmp_path):
    yaml_file = tmp_path / "mine.yaml"
    yaml_file.write_text("x: 1\n")
    assert paths.entry_point("agents", "codex_like", yaml_file) == "openrua.plugins.agents.codex_like"
    assert paths.entry_point("benchmarks", "libero", yaml_file) == \
        "openrua.robot.sim.bridge.environments.libero"
    (tmp_path / "mine.py").write_text("LOADER = 1\n")
    assert paths.entry_point("benchmarks", "./mine.py", yaml_file) == str(tmp_path / "mine.py")
    with pytest.raises(NotFound, match="gone.py"):
        paths.entry_point("benchmarks", "./gone.py", yaml_file)
    with pytest.raises(KeyError):
        paths.entry_point("robots", "x", yaml_file)


def test_simulator_venv_relative_names_live_under_home(tmp_path):
    assert paths.simulator_venv("cap-x/.venv-libero", tmp_path) == \
        tmp_path / "simulators" / "cap-x" / ".venv-libero"
    assert paths.simulator_venv("/abs/.venv-x", tmp_path) == Path("/abs/.venv-x")
    assert paths.simulator_root(Path("/s/cap-x/.venv-libero")) == Path("/s/cap-x")


def test_code_root_holds_the_package():
    assert (paths.code_root() / "openrua" / "__init__.py").is_file()


def test_unknown_kind_is_refused():
    with pytest.raises(KeyError):
        paths.bundled("gadgets")
