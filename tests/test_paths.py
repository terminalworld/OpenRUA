"""paths: the user directory and the bundled -> user lookup order."""
from pathlib import Path

import pytest

from robocli import paths


def test_home_defaults_and_overrides(tmp_path):
    assert paths.home() == Path("~/.robocli").expanduser()
    assert paths.home(tmp_path) == tmp_path
    assert paths.robots_dir(tmp_path) == tmp_path / "robots"
    assert paths.credentials_dir(tmp_path) == tmp_path / "credentials"
    assert paths.config_path(tmp_path) == tmp_path / "config.yaml"


def test_bundled_entries_ship_in_the_package():
    names = {e.name for e in paths.available("robots")}
    assert {"panda-sim", "panda-sim-humble", "panda-omron-sim"} <= names
    assert {e.name for e in paths.available("benchmarks")} >= {
        "libero_pro", "capbench", "robocasa365"}
    assert all(e.source == "bundled" for e in paths.available("robots"))


def test_find_by_name_then_user_dir_then_path(tmp_path):
    bundled = paths.find("robots", "panda-sim")
    assert bundled.name == "panda-sim.yaml" and bundled.is_file()
    (tmp_path / "robots").mkdir()
    mine = tmp_path / "robots" / "my-arm.yaml"
    mine.write_text("machine: {}\n")
    assert paths.find("robots", "my-arm", tmp_path) == mine
    assert paths.find("robots", str(mine)) == mine          # a path is taken as is
    entries = {e.name: e for e in paths.available("robots", tmp_path)}
    assert entries["my-arm"].source == "user"


def test_user_file_with_a_bundled_name_is_flagged_not_used(tmp_path):
    (tmp_path / "robots").mkdir()
    shadow = tmp_path / "robots" / "panda-sim.yaml"
    shadow.write_text("machine: {}\n")
    found = paths.find("robots", "panda-sim", tmp_path)
    assert found != shadow and found.is_file()
    entry = next(e for e in paths.available("robots", tmp_path) if e.name == "panda-sim")
    assert entry.source == "bundled" and entry.shadowed_by == shadow
    assert not [e for e in paths.available("robots", tmp_path)
                if e.source == "user" and e.name == "panda-sim"]


def test_missing_name_says_what_exists_and_where_to_add(tmp_path):
    with pytest.raises(FileNotFoundError) as e:
        paths.find("robots", "nope", tmp_path)
    assert "panda-sim" in str(e.value)
    assert str(tmp_path / "robots") in e.value.hint      # where to add your own
    with pytest.raises(FileNotFoundError):
        paths.find("robots", str(tmp_path / "gone.yaml"))


def test_simulator_venv_relative_names_live_under_home(tmp_path):
    assert paths.simulator_venv("cap-x/.venv-libero", tmp_path) == \
        tmp_path / "simulators" / "cap-x" / ".venv-libero"
    assert paths.simulator_venv("/abs/.venv-x", tmp_path) == Path("/abs/.venv-x")
    assert paths.simulator_root(Path("/s/cap-x/.venv-libero")) == Path("/s/cap-x")


def test_code_root_holds_the_package():
    assert (paths.code_root() / "robocli" / "__init__.py").is_file()


def test_unknown_kind_is_refused():
    with pytest.raises(KeyError):
        paths.bundled("gadgets")
