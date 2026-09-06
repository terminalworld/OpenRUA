"""The trial's ACTIVE wall-clock budget: renamed on 2026-08-20 when it
stopped counting time suspended at a quota wall. A config carrying the old
name must fail loudly -- reading it as if nothing changed would run the new
semantics under a name that promised total time."""

import pytest


def test_new_key_is_read():
    from robocli.config import resolve_wall_clock_min
    assert resolve_wall_clock_min(
        {"protocol": {"active_wall_clock_minutes": 240}}) == 240.0


def test_missing_key_falls_back():
    from robocli.config import DEFAULT_WALL_CLOCK_MIN, resolve_wall_clock_min
    assert resolve_wall_clock_min({"protocol": {}}) == DEFAULT_WALL_CLOCK_MIN
    assert resolve_wall_clock_min({}) == DEFAULT_WALL_CLOCK_MIN


def test_legacy_key_fails_loudly_with_the_fix():
    from robocli.config import resolve_wall_clock_min
    with pytest.raises(ValueError) as e:
        resolve_wall_clock_min({"protocol": {"wall_clock_minutes": 240}})
    assert "active_wall_clock_minutes" in str(e.value)
    assert "sed -i" in str(e.value)   # the message carries its own fix


def test_shipped_configs_use_the_new_key():
    import pathlib

    import yaml
    for cfg in (pathlib.Path(__file__).resolve().parents[1] / "robocli" / "configs" / "benchmarks").glob("*.yaml"):
        protocol = (yaml.safe_load(cfg.read_text()) or {}).get("protocol", {})
        assert "wall_clock_minutes" not in protocol, cfg
