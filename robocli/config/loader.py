"""Reading and layering config files: yaml in, validated dicts out.

Every file is validated against its schema model; unknown keys are
errors. Consumers read plain dicts (``dump()``): defaults filled in,
absent optionals left out.

``load_config`` assembles a benchmark config with its robot profile and
the defaults files; ``compose`` does the same from a robot profile's
``world:`` when no benchmark is named; ``apply_suite_overrides`` and
``normalize_arms`` derive the per-suite view a trial actually runs.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, ValidationError

from robocli.config import paths
from robocli.config.schema import Benchmark, ResolvedConfig, RobotProfile, UserConfig
from robocli.errors import ConfigError, UsageError


def validate(model: type[BaseModel], data: Any, source: str | Path) -> BaseModel:
    """Validate ``data`` (parsed yaml) against ``model``; raise ConfigError
    naming ``source`` and each bad key path."""
    try:
        return model.model_validate(data if data is not None else {})
    except ValidationError as e:
        lines = []
        for err in e.errors():
            loc = ".".join(str(x) for x in err["loc"]) or "<root>"
            lines.append(f"  {loc}: {err['msg']}")
        raise ConfigError(f"{source}: does not fit the {model.__name__} schema\n"
                          + "\n".join(lines),
                          hint="robocli config schema prints every key and its meaning") from None


def dump(model: BaseModel) -> dict:
    """The dict consumers read: defaults filled in, absent optionals
    (None) left out so ``cfg.get(key, default)`` keeps its meaning."""
    return model.model_dump(exclude_none=True, by_alias=True)


def load_yaml(path: Path) -> Any:
    try:
        return yaml.safe_load(path.read_text())
    except yaml.YAMLError as e:
        raise ConfigError(f"{path}: not valid YAML: {e}") from None


def load_user_config(path: Path) -> UserConfig:
    """The user's defaults file; absent = no defaults."""
    if not path.is_file():
        return UserConfig()
    return validate(UserConfig, load_yaml(path), path)


def layer_agent(bench_agent: dict, *defaults: UserConfig) -> dict:
    """The benchmark's agent section over the defaults files, lowest
    layer first (package defaults, then the user's file). A defaults
    file contributes only the keys it wrote; ``options`` merge key by
    key, every other key is replaced by the higher layer."""
    out: dict = {}
    layers = [d.agent.model_dump(exclude_unset=True, exclude_none=True) for d in defaults]
    for layer in layers + [bench_agent]:
        for k, v in layer.items():
            if k == "options":
                out["options"] = {**out.get("options", {}), **(v or {})}
            else:
                out[k] = v
    return out


def layer_sandbox(*defaults: UserConfig) -> dict:
    """The sandbox section comes from the defaults files only (a machine
    fact); the highest layer that wrote a key supplies it."""
    out: dict = {}
    for d in defaults:
        out.update(d.sandbox.model_dump(exclude_unset=True))
    return out


def load_robot(robot: str, home: Path | None = None) -> dict:
    """A robot profile by name (bundled, then ``<home>/robots/``) or by
    path, validated (RobotProfile). Returns it as a dict: its
    ``machine:`` section is the robot, ``world:`` the default scene."""
    p = paths.find("robots", robot, home)
    return dump(validate(RobotProfile, load_yaml(p), p))


def load_config(path: Path | str, robot: str | None = None,
                home: Path | None = None) -> dict:
    """A benchmark config (name or path), validated and assembled:
    ``robot: <name>`` in the file (or the argument, which wins; or the
    user's default) pulls in that profile's ``machine:`` section; a file
    carrying its own ``machine:`` is taken as is. The user's
    ``~/.robocli/config.yaml`` supplies agent defaults under the file's
    own, and the package's ``configs/config.yaml`` under that. Returns the
    assembled dict (ResolvedConfig, validated)."""
    p = paths.find("benchmarks", path, home)
    bench = validate(Benchmark, load_yaml(p), p)
    defaults = load_user_config(paths.package_config_path())
    user = load_user_config(paths.config_path(home))
    cfg = dump(bench)
    named = cfg.pop("robot", None)          # always popped: the argument wins
    robot = robot or named or user.robot
    if robot:
        cfg["machine"] = load_robot(robot, home)["machine"]
    if "machine" not in cfg:
        raise ConfigError(
            f"{p}: names no robot (robot: <name>, --robot, or robot: in "
            f"{paths.config_path(home)}) and carries no machine: section")
    cfg["agent"] = layer_agent(cfg.get("agent", {}), defaults, user)
    cfg["sandbox"] = layer_sandbox(defaults, user)
    return dump(validate(ResolvedConfig, cfg, p))


def compose(robot: str | None, bench: str | None,
            home: Path | None = None) -> tuple[dict, str, int]:
    """robot profile + benchmark config -> one assembled config, plus the
    (task_suite, task_id) to load. Without --bench the profile's
    ``world:`` says which scene to load."""
    if not robot and not bench:
        raise UsageError("name a robot or a benchmark (--bench)",
                         hint="robocli robots / robocli benchmarks list them")
    world = {}
    if robot:
        world = load_robot(robot, home).get("world", {})
    bench = bench or world.get("benchmark")
    if not bench:
        raise UsageError(f"robot {robot!r} names no world: and no --bench given",
                         hint="pass --bench <benchmark> or add world: to the profile")
    cfg = load_config(bench, robot, home)
    suite = world.get("task_suite") or cfg["task"]["suites"][0]
    task_id = int(world.get("task_id", 0))
    return cfg, suite, task_id


def apply_suite_overrides(cfg: dict, task_suite: str) -> dict:
    """Deep-merge ``cfg["suite_overrides"][task_suite]`` into cfg, in place.

    ``None`` deletes a key: a suite whose robot mounts no gripper nulls
    ``machine.ports.gripper`` and ``machine.gripper`` so the robot and
    machine.yaml agree. Every consumer reads the result (the trial's
    config.yaml); only the runner runs this.
    """
    def merge(dst: dict, src: dict) -> None:
        for k, v in src.items():
            if v is None:
                dst.pop(k, None)
            elif isinstance(v, dict) and isinstance(dst.get(k), dict):
                merge(dst[k], v)
            else:
                dst[k] = v

    ov = (cfg.get("suite_overrides") or {}).get(task_suite)
    if ov:
        merge(cfg, ov)
        # The override is authored as raw yaml; the merged view must still
        # fit the schema (a misspelled key in an override is the same
        # mistake as one in the profile).
        checked = dump(validate(
            ResolvedConfig, cfg, f"suite_overrides.{task_suite}"))
        cfg.clear()
        cfg.update(checked)
    return cfg


def normalize_arms(cfg: dict) -> dict:
    """Materialize ``machine.arms`` (the per-arm view), in place.

    Single-arm configs author the flat fields (``arm``/``ports``/
    ``gripper``/``frames``); this synthesizes ``arms[0]`` from them so
    every downstream unit (bridge ports, sensors, manifest, preflight)
    loops one uniform list. Multi-arm suites author ``machine.arms``
    explicitly in their suite override and the flat fields are ignored.
    Runs once here, lands in config.yaml; nobody else re-derives it.
    """
    m = cfg.get("machine")
    if not m or "arms" in m:
        return cfg
    ports = m.get("ports", {})
    m["arms"] = [{
        "label": "",
        "joints": m.get("arm", {}).get("joints", []),
        "limits_rad": m.get("arm", {}).get("limits_rad", []),
        "gripper": m.get("gripper"),
        "ports": {k: ports[k] for k in
                  ("trajectory", "gripper", "twist", "wrench") if ports.get(k)},
        "hand_body": "robot0_right_hand",
        "tf_base_body": m.get("tf", {}).get("base_body", "robot0_base"),
        "base_frame": m.get("frames", {}).get("base", "panda_link0"),
        "hand_frame": m.get("frames", {}).get("hand", "panda_hand"),
    }]
    return cfg


# One shared fallback when a config omits protocol.active_wall_clock_minutes.
# The bundled configs set the key explicitly.
DEFAULT_WALL_CLOCK_MIN = 30.0

# The budget used to be protocol.wall_clock_minutes, before it stopped
# counting time a trial spends suspended at a quota wall. A config still
# carrying the old key is refused rather than read as if nothing changed:
# accepting it would run active-time semantics under a name that
# promised total time.
LEGACY_WALL_CLOCK_KEY = "wall_clock_minutes"


def resolve_wall_clock_min(cfg: dict) -> float:
    """The trial's active wall-clock budget in minutes, from the config."""
    protocol = cfg.get("protocol", {})
    if LEGACY_WALL_CLOCK_KEY in protocol:
        raise ValueError(
            f"config uses protocol.{LEGACY_WALL_CLOCK_KEY}, which was renamed "
            "to protocol.active_wall_clock_minutes (the budget counts active "
            "time only, not time suspended at a quota wall). Fix it with:\n"
            f"  sed -i 's/{LEGACY_WALL_CLOCK_KEY}:/active_wall_clock_minutes:/' "
            "<config.yaml>")
    return float(protocol.get("active_wall_clock_minutes",
                              DEFAULT_WALL_CLOCK_MIN))
