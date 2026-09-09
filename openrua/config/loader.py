"""Reading and layering config files: yaml in, validated dicts out.

Every file is validated against its schema model; unknown keys are
errors. Consumers read plain dicts (``dump()``): defaults filled in,
absent optionals left out.

``compose`` assembles one resolved config from a robot (type or
instance), a simulator and, when named, a benchmark, plus the defaults
files; ``load_config`` is the benchmark-first entry the runner uses;
``assemble`` folds robot type, embodiments and install into the
``machine:`` dict every downstream unit reads. ``apply_suite_overrides``
and ``normalize_arms`` derive the per-suite view a trial actually runs.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, ValidationError

from openrua.config import paths
from openrua.config.schema import (Benchmark, Machine, ResolvedConfig, RobotInstance,
                                   RobotType, SimulatorProfile, UserConfig)
from openrua.errors import ConfigError, UsageError


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
                          hint="openrua config schema prints every key and its meaning") from None


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
    data = load_yaml(path)
    if isinstance(data, dict) and isinstance(data.get("agent"), dict):
        old = data["agent"]
        name = old.get("name", "<name>")
        facts = {k: v for k, v in old.items() if k != "name"}
        raise ConfigError(f"{path}: agent: is a name now, and an agent's facts live under "
                          f"agents.<name>",
                          hint="rewrite it as:\n  agent: " + str(name) + "\n  agents:\n    "
                               + str(name) + ": " + (str(facts) if facts else "{}"))
    return validate(UserConfig, data, path)


def default_agent_name(*defaults: UserConfig) -> str | None:
    """The agent the defaults files name, highest layer that wrote it."""
    name = None
    for d in defaults:
        name = d.agent or name
    return name


def layer_agent(bench_agent: dict, *defaults: UserConfig, agent: str | None = None) -> dict:
    """The resolved ``agent:`` section for one agent: the defaults
    files' facts about it (``agents.<name>``, lowest layer first: the
    package's file, then the user's), then the benchmark's agent
    section over them. The agent is ``agent`` (a command's --agent),
    else the benchmark's ``agent.name``, else the defaults' name. A
    benchmark section naming a different agent is that agent's and is
    left out; ``options`` merge key by key, every other key is replaced
    by the higher layer."""
    name = agent or bench_agent.get("name") or default_agent_name(*defaults)
    if not name:
        raise ConfigError("no agent named", hint="openrua config set --agent <name>, or "
                          "agent.name in the benchmark file, or --agent")
    layers = [d.agents[name].model_dump(exclude_unset=True, exclude_none=True)
              for d in defaults if name in d.agents]
    if bench_agent.get("name") in (None, name):
        layers.append({k: v for k, v in bench_agent.items() if k != "name"})
    out: dict = {"name": name}
    for layer in layers:
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


def _merge(dst: dict, src: dict) -> dict:
    """Deep merge ``src`` into ``dst`` in place; None deletes a key."""
    for k, v in src.items():
        if v is None:
            dst.pop(k, None)
        elif isinstance(v, dict) and isinstance(dst.get(k), dict):
            _merge(dst[k], v)
        else:
            dst[k] = copy.deepcopy(v)
    return dst


def load_robot(robot: str, home: Path | None = None) -> tuple[str, dict]:
    """A robots/ file by bundled name or by path. Returns ``("type", facts)`` for a robot type or
    ``("instance", {"type": ..., "machine": ...})`` for a particular
    (usually real) robot; a file with a ``machine:`` section is an
    instance."""
    p = paths.find("robots", robot)
    data = load_yaml(p) or {}
    if "machine" in data:
        # Only the keys the file wrote, so an instance over a type replaces
        # exactly what it says (defaults are filled in after the merge).
        inst = validate(RobotInstance, data, p)
        return "instance", inst.model_dump(exclude_unset=True, by_alias=True)
    return "type", dump(validate(RobotType, data, p))


def load_simulator(sim: str, home: Path | None = None) -> dict:
    """A simulators/ file by bundled name or by path, validated
    (SimulatorProfile); the install's relative files made absolute."""
    del home
    p = paths.find("simulators", sim)
    s = dump(validate(SimulatorProfile, load_yaml(p), p))
    _absolutize_install(s["install"], p)
    return s


def load_benchmark(bench: str) -> tuple[Path, dict]:
    """A benchmarks/ file by bundled name or by path, validated
    (Benchmark); returns the file and the dict, the install's relative
    files made absolute."""
    p = paths.find("benchmarks", bench)
    b = dump(validate(Benchmark, load_yaml(p), p))
    if b.get("install"):
        _absolutize_install(b["install"], p)
    return p, b


def _absolutize_install(install: dict, declared_in: Path) -> None:
    """Files an install section names relative to its own yaml
    (``requirements``, a checkout's ``patch``) become absolute, and
    ``{here}`` in ``shell`` becomes that directory, so the section can
    be merged with another file's and rendered anywhere."""
    here = Path(declared_in).expanduser().resolve().parent

    def absolute(rel: str) -> str:
        q = Path(rel).expanduser()
        return str(q if q.is_absolute() else here / q)

    if install.get("requirements"):
        install["requirements"] = absolute(install["requirements"])
    for c in install.get("checkouts") or []:
        if c.get("patch"):
            c["patch"] = absolute(c["patch"])
    if install.get("shell"):
        install["shell"] = install["shell"].replace("{here}", str(here))


def install_for(sim: str, bench: str | None = None) -> dict:
    """The install a simulator, and a benchmark over it, declare: the
    simulator's section with the benchmark's written keys over it."""
    install = copy.deepcopy(load_simulator(sim)["install"])
    if bench:
        _, b = load_benchmark(bench)
        if b.get("install"):
            install.update({k: v for k, v in b["install"].items() if v is not None})
    return install


def assemble(robot_type: dict, embodiments: list[dict], install: dict,
             cameras: dict | None, source: str, engine: str | None = None) -> dict:
    """robot type + embodiments (simulator's, then the benchmark's) +
    install + the resolved engine -> the ``machine:`` dict (Machine,
    validated). Later embodiments write over earlier ones, key by key."""
    m = copy.deepcopy(robot_type)
    for e in embodiments:
        _merge(m, e)
    if cameras is not None:
        m["cameras"] = copy.deepcopy(cameras)
    backend = {"kind": "sim", "ros_distro": install["ros_distro"],
               "gpus": install.get("gpus", False),
               "simulator": {"venv": install["venv"], "engine": engine}}
    for k in ("container",):
        if install.get(k):
            backend["simulator"][k] = install[k]
    for k in ("image", "sandbox_image", "resources"):
        if install.get(k) is not None:
            backend[k] = install[k]
    m["backend"] = backend
    return dump(validate(Machine, m, source))


def _instance_machine(inst: dict, home: Path | None) -> dict:
    """An instance's machine over its type's facts (when it names one)."""
    if not inst.get("type"):
        return dump(validate(Machine, inst["machine"], "robot instance"))
    kind, facts = load_robot(inst["type"], home)
    if kind != "type":
        raise ConfigError(f"robot instance names type {inst['type']!r}, which is itself "
                          "an instance (has a machine: section)")
    m = copy.deepcopy(facts)
    _merge(m, inst["machine"])
    return dump(validate(Machine, m, f"instance of {inst['type']}"))


def _who_embodies(robot: str, home: Path | None) -> list[str]:
    """Benchmarks whose scenes bring this robot type, for error hints."""
    out = []
    for e in paths.available("benchmarks"):
        try:
            b = validate(Benchmark, load_yaml(e.path), e.path)
        except ConfigError:
            continue
        if robot in b.scenes.robots:
            out.append(e.name)
    return out


@dataclass
class Composed:
    """One resolved config and where it came from."""
    cfg: dict
    suite: str
    task_id: int
    robot: str
    simulator: str | None      # None for a real-robot instance
    benchmark: str | None      # None for an engine's native scene
    install: dict | None = None  # the simulator's install with the benchmark's over it


def compose(robot: str | None, sim: str | None = None, bench: str | None = None,
            home: Path | None = None, agent: str | None = None) -> Composed:
    """robot (type or instance) + simulator + optional benchmark -> one
    resolved config (ResolvedConfig, validated) and the scene to load.

    A benchmark supplies robot and simulator when the arguments do not;
    the user's config.yaml supplies a default robot. A robot type needs
    a simulator that embodies it (its ``robots:``) or a benchmark whose
    ``scenes.robots`` does; a robot instance (a real robot) brings its
    own machine and takes no simulator. Without a benchmark the
    simulator's native scene is loaded.
    """
    defaults = load_user_config(paths.package_config_path())
    user = load_user_config(paths.config_path(home))
    explicit_sim = sim
    bench = bench or user.benchmark or defaults.benchmark
    b = bench_path = None
    if bench:
        bench_path, b = load_benchmark(bench)
        robot = robot or b.get("robot") or user.robot
        sim = sim or b.get("simulator")
    else:
        robot = robot or user.robot or defaults.robot
    sim = sim or user.simulator or defaults.simulator
    composed_install = None
    if b is not None and b.get("machine") and not robot:
        machine, simulator, robot = b["machine"], None, "(inline machine:)"
    else:
        if not robot:
            raise UsageError("name a robot",
                             hint="openrua robots lists them; openrua config set --robot <name> "
                                  "makes one the default")
        kind, r = load_robot(robot, home)
        if kind == "instance":
            if explicit_sim and not (b and b.get("simulator") == explicit_sim):
                raise UsageError(f"{robot} is a robot instance with its own machine: "
                                 "and takes no --sim")
            machine, simulator = _instance_machine(r, home), None
        else:
            if not sim:
                raise UsageError(f"{robot} is a robot type: name the simulator that "
                                 "embodies it (--sim) or a benchmark (--bench)",
                                 hint="openrua simulators / openrua benchmarks list them; "
                                      "openrua config set --sim <name> makes one the default")
            s = load_simulator(sim, home)
            embodiments = [e for e in (s["robots"].get(robot),
                                       (b or {}).get("scenes", {}).get("robots", {}).get(robot))
                           if e]
            if not embodiments:
                brings = _who_embodies(robot, home)
                hint = (f"benchmarks that bring {robot}: {', '.join(brings)}; pass one "
                        "with --bench" if brings else "openrua robots / openrua simulators")
                raise ConfigError(f"simulator {sim} does not embody {robot} (it has: "
                                  f"{', '.join(sorted(s['robots'])) or 'none'})", hint=hint)
            install = copy.deepcopy(s["install"])
            if b and b.get("install"):
                install.update({k: v for k, v in b["install"].items() if v is not None})
            composed_install = install
            cameras = ((b or {}).get("scenes", {}).get("cameras")
                       or (s.get("native") or {}).get("cameras"))
            if not b and not s.get("native"):
                raise UsageError(f"simulator {sim} has no native scene: name a benchmark "
                                 "(--bench)", hint="openrua benchmarks lists them")
            machine = assemble(r, embodiments, install, cameras,
                               f"{robot} on {sim}" + (f" for {bench}" if bench else ""),
                               engine=paths.entry_point("simulators", s["entry_point"],
                                                        paths.find("simulators", sim)))
            simulator = sim
    if b is not None:
        cfg = {k: copy.deepcopy(v) for k, v in b.items()
               if k in ("task", "protocol", "agent", "suite_overrides")}
        cfg["task"]["loader"] = paths.entry_point("benchmarks", b["entry_point"], bench_path)
        source = bench_path
    else:
        sim_path, native = (None, None)
        if simulator:
            sim_path = paths.find("simulators", sim)
            native = load_simulator(sim, home).get("native")
        if native is None:
            raise UsageError(f"{robot} is a real robot: name a benchmark (--bench) to "
                             "run, or use openrua up / openrua agent with --task")
        cfg = {"task": {"benchmark": sim_path.stem, "suites": [native["scene"]],
                        "init_states": "seeded-reset",
                        "loader": paths.entry_point("benchmarks", native["entry_point"],
                                                    sim_path)}}
        source = f"{sim} native scene"
    cfg["machine"] = machine
    cfg["agent"] = layer_agent(cfg.get("agent", {}), defaults, user, agent=agent)
    cfg["sandbox"] = layer_sandbox(defaults, user)
    cfg = dump(validate(ResolvedConfig, cfg, source))
    suite = (b or {}).get("scenes", {}).get("default_suite") or cfg["task"]["suites"][0]
    if suite not in cfg["task"]["suites"]:
        raise ConfigError(f"{source}: scenes.default_suite {suite!r} is not in task.suites")
    return Composed(cfg, suite, 0, robot, simulator,
                    (b or {}).get("task", {}).get("benchmark") if b else None,
                    composed_install)


def load_config(path: Path | str, robot: str | None = None,
                home: Path | None = None, sim: str | None = None,
                agent: str | None = None) -> dict:
    """A benchmark config (name or path), assembled with its robot and
    simulator (the arguments win over the file's ``robot:`` /
    ``simulator:`` lines) and the defaults files. Returns the resolved
    dict (ResolvedConfig, validated)."""
    return compose(robot, sim, str(path), home, agent).cfg


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
        "hand_body": m.get("tf", {}).get("hand_body", "robot0_right_hand"),
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
