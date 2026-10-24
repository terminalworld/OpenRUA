"""config: the schema every file is held to, and how the layers combine."""
from pathlib import Path

import pytest
import yaml

from openrua import config
from openrua.errors import NotFound, UsageError
from openrua.config import paths
from openrua.config import (apply_suite_overrides, compose, load_config, load_robot,
                            load_simulator, normalize_arms)

BUNDLED = [e.name for e in paths.available("benchmarks")]


@pytest.mark.parametrize('name', BUNDLED)
def test_benchmark_defaults_do_not_replace_explicit_api_authentication(name, tmp_path):
    auth = {'mode': 'api', 'key_file': '/private/provider.key'}
    (tmp_path / 'config.yaml').write_text(yaml.safe_dump({'agents': {'claude-code': {'auth': auth}}}))
    assert load_config(name, home=tmp_path)['agent']['auth'] == auth


def test_authentication_authored_in_benchmark_still_overrides_defaults(tmp_path):
    source = config.load_yaml(paths.find('benchmarks', 'capbench'))
    source['agent']['auth'] = {'mode': 'native'}
    # The authored install paths are relative to the bundled benchmark.
    source.pop('install', None)
    benchmark = tmp_path / 'benchmark.yaml'
    benchmark.write_text(yaml.safe_dump(source))
    (tmp_path / 'config.yaml').write_text(yaml.safe_dump({'agents': {'claude-code': {
        'auth': {'mode': 'api', 'key_file': '/private/provider.key'}}}}))
    assert load_config(str(benchmark), home=tmp_path)['agent']['auth'] == {'mode': 'native'}


@pytest.mark.parametrize("name", BUNDLED)
def test_every_bundled_benchmark_assembles(name, tmp_path):
    cfg = load_config(name, home=tmp_path)
    assert set(cfg) >= {"task", "protocol", "agent", "machine"}
    assert "robot" not in cfg                       # resolved into machine:
    assert cfg["agent"]["name"] == "claude-code"   # the bundled configs name it
    assert cfg["agent"]["options"] == {"effort": "high"}
    assert "credentials_dir" not in cfg["agent"]    # default is per agent, under home


@pytest.mark.parametrize("name", BUNDLED)
def test_every_suite_view_still_fits_the_schema(name, tmp_path):
    cfg = load_config(name, home=tmp_path)
    for suite in cfg["task"]["suites"]:
        view = apply_suite_overrides(dict(yaml.safe_load(yaml.safe_dump(cfg))), suite)
        normalize_arms(view)
        config.validate(config.ResolvedConfig, view, f"{name}:{suite}")


def test_explicit_robot_argument_replaces_the_named_one(tmp_path):
    # robocasa365 brings panda-omron into robosuite; asking libero_pro for it
    # is refused with the benchmarks that do bring it
    with pytest.raises(config.ConfigError) as e:
        load_config("libero_pro", robot="panda-omron", home=tmp_path)
    assert "does not embody panda-omron" in str(e.value) and "robocasa365" in e.value.hint
    cfg = load_config("robocasa365", robot="panda-omron", home=tmp_path)
    assert "robot" not in cfg and cfg["machine"]["robot"]["model"].startswith("Franka Emika Panda on Omron")


def test_assembly_folds_type_simulator_and_benchmark(tmp_path):
    """benchmark -> simulator -> robot: each layer's keys land in machine:
    and a later layer writes over an earlier one, key by key."""
    c = compose(None, None, "libero_pro", tmp_path)
    m = c.cfg["machine"]
    assert (c.robot, c.simulator, c.benchmark) == ("panda", "robosuite", "libero_pro")
    assert c.suite == "libero_goal_task"                         # scenes.default_suite
    assert m["arm"]["joints"][0] == "panda_joint1"              # from robots/panda.yaml
    assert m["engine_model"] == "Panda"                          # simulators/robosuite.yaml
    assert m["joint_name_map"] == {"robot0_": "panda_", "gripper0_": "panda_"}
    assert m["controller_kp_scale"] == 10                        # libero_pro's scenes.robots.panda
    assert m["planning"]["planning_frame"] == "world"            # the simulator's, over the type's
    assert m["planning"]["moveit"] is True                       # the type's other planner keys stay
    assert m["backend"]["image"] == "openrua-sim-libero_pro"     # libero_pro's own install
    assert m["backend"]["ros_distro"] == "jazzy" and m["backend"]["simulator"] == {
        "engine": "openrua.robot.sim.bridge.engines.robosuite"}
    assert m["cameras"]["record"] == ["agentview", "robot0_eye_in_hand"]  # scenes.cameras
    cap = load_config("capbench", home=tmp_path)["machine"]
    assert cap["controller_kp_scale"] == 1                       # the engine's own value
    assert cap["planning"]["planning_frame"] == "world"          # the simulator's
    assert cap["backend"]["ros_distro"] == "humble"
    assert cap["backend"]["image"] == "openrua-sim-robosuite"     # nothing of its own: the engine's


def test_native_scene_without_a_benchmark(tmp_path):
    c = compose("panda", "robosuite", None, tmp_path)
    assert (c.benchmark, c.suite, c.task_id) == (None, "Lift", 0)
    assert c.cfg["task"] == {"benchmark": "robosuite", "suites": ["Lift"],
                             "init_states": "seeded-reset", "split": "target", "task_language": {},
                             "loader": "openrua.robot.sim.bridge.environments.robosuite"}
    assert c.cfg["machine"]["backend"]["image"] == "openrua-sim-robosuite"
    assert c.cfg["agent"]["name"] == "claude-code"              # the defaults files


def test_robot_type_alone_asks_for_a_simulator(tmp_path):
    with pytest.raises(UsageError) as e:
        compose("panda", None, None, tmp_path)
    assert "--sim" in str(e.value)


def test_real_instance_brings_its_machine_over_its_type(tmp_path):
    lab = tmp_path / "lab-panda.yaml"                          # a file of your own: by path
    (tmp_path / "cameras").mkdir()
    (tmp_path / "cameras" / "wristcam.yaml").write_text(        # a camera type, by path
        "model: Wrist cam\ntopics:\n  color: /{name}/image\n  info: /{name}/info\n")
    lab.write_text(
        "type: panda\n"
        "machine:\n"
        "  backend: {kind: real, ros_distro: humble, discovery: {network: host}}\n"
        "  cameras: {list: [{name: wrist, type: ./cameras/wristcam.yaml, mount: wrist}]}\n"
        "  robot: {model: Our lab Panda}\n"
        "  gripper: {holds_partway: true, effort_honoured: true}\n")
    # Free use has no invented benchmark, loader, suite or task index.
    free = compose(str(lab), home=tmp_path)
    assert "task" not in free.cfg
    assert (free.simulator, free.benchmark, free.suite, free.task_id) == (None,) * 4
    assert free.cfg["machine"]["robot"]["model"] == "Our lab Panda"
    c = compose(str(lab), None, "libero_pro", tmp_path)
    m = c.cfg["machine"]
    assert c.simulator is None and m["backend"]["kind"] == "real"
    assert m["robot"]["model"] == "Our lab Panda"               # instance over type
    assert m["robot"]["description"].startswith("7-joint")      # type's line kept
    assert m["arm"]["joints"][0] == "panda_joint1"
    assert m["cameras"]["list"][0]["name"] == "wrist"
    # the type is read relative to the file naming it, into the machine
    assert m["cameras"]["types"]["./cameras/wristcam.yaml"]["model"] == "Wrist cam"
    assert m["backend"]["sandbox_image"] == "openrua-sandbox-humble"
    assert m["gripper"]["open"] == 0.04 and m["gripper"]["holds_partway"] is True
    with pytest.raises(UsageError):                            # an instance takes no --sim
        compose(str(lab), "robosuite", None, tmp_path)
    # The type says how the gripper is addressed; only the instance knows
    # what its driver does with a goal.
    lab.write_text(lab.read_text().replace(
        "  gripper: {holds_partway: true, effort_honoured: true}\n", ""))
    with pytest.raises(config.ConfigError) as e:
        compose(str(lab), home=tmp_path)
    assert "holds_partway" in str(e.value)


def test_an_arm_without_frames_is_refused_not_given_panda_names(tmp_path):
    lab = tmp_path / "lab.yaml"
    lab.write_text(
        "machine:\n  backend: {kind: real, discovery: {network: host}}\n  cameras: {list: []}\n"
        "  robot: {model: Lab arm}\n  arm: {joints: [j1], limits_rad: [[-1, 1]]}\n")
    with pytest.raises(config.ConfigError) as e:
        compose(str(lab), home=tmp_path)
    assert "machine.frames is required" in str(e.value)
    lab.write_text(lab.read_text() + "  frames: {world: world, hand: tool0}\n")
    with pytest.raises(config.ConfigError) as e:
        compose(str(lab), home=tmp_path)
    assert "machine.frames.base" in str(e.value)


def test_a_simulated_arm_names_its_bodies(tmp_path):
    sim = yaml.safe_load(paths.find("simulators", "robosuite").read_text())
    del sim["robots"]["panda"]["tf"]["hand_body"]
    path = tmp_path / "robosuite.yaml"
    path.write_text(yaml.safe_dump(sim))
    with pytest.raises(config.ConfigError) as e:
        compose("panda", str(path), None, tmp_path)
    assert "machine.tf.base_body and machine.tf.hand_body are required" in str(e.value)


def test_moveit_names_its_planning_group(tmp_path):
    lab = tmp_path / "lab.yaml"
    lab.write_text(
        "machine:\n  backend: {kind: real, discovery: {network: host}}\n  cameras: {list: []}\n"
        "  robot: {model: Lab arm}\n  arm: {joints: [j1], limits_rad: [[-1, 1]]}\n"
        "  frames: {world: world, base: base_link}\n  planning: {moveit: true}\n")
    with pytest.raises(config.ConfigError) as e:
        compose(str(lab), home=tmp_path)
    assert "moveit: true requires group" in str(e.value)


def test_simulator_file_is_validated(tmp_path):
    s = load_simulator("robosuite", tmp_path)
    assert s["engine"].startswith("robosuite") and "panda" in s["robots"]
    assert s["native"]["entry_point"] == "robosuite"
    bad = tmp_path / "x.yaml"
    bad.write_text("engine: x\ninstall: {venv: v}\nrobots: {panda: {kp: 3}}\n")
    with pytest.raises(config.ConfigError) as e:
        load_simulator(bad, tmp_path)
    assert "robots.panda.kp" in str(e.value)


def test_null_in_an_override_deletes_the_key(tmp_path):
    cfg = load_config("capbench", home=tmp_path)
    view = apply_suite_overrides(dict(yaml.safe_load(yaml.safe_dump(cfg))), "capbench_wipe")
    assert "gripper" not in view["machine"]
    assert "gripper" not in view["machine"]["ports"]
    assert view["machine"]["robot"]["model"].startswith("Franka Emika Panda with fixed")


def test_unknown_key_fails_loud_with_its_path(tmp_path):
    bad = tmp_path / "b.yaml"
    bad.write_text("task: {benchmark: libero_pro, suites: [x]}\n"
                   "protocol: {max_turn: 3}\nrobot: panda\nsimulator: robosuite\n")
    with pytest.raises(config.ConfigError) as e:
        load_config(bad, home=tmp_path)
    assert "protocol.max_turn" in str(e.value) and str(bad) in str(e.value)


def test_bad_override_key_fails_loud(tmp_path):
    cfg = load_config("libero_pro", home=tmp_path)
    cfg["suite_overrides"] = {"libero_goal_task": {"machine": {"grippr": None, "ports": {"twst": "/x"}}}}
    with pytest.raises(config.ConfigError) as e:
        apply_suite_overrides(cfg, "libero_goal_task")
    assert "machine.ports.twst" in str(e.value)


def test_defaults_match_the_reported_runs():
    p = config.Protocol()
    assert (p.max_turns, p.active_wall_clock_minutes, p.trials_per_task) == (500, 240, 10)
    assert p.resume_on_quota_wall is False          # scale-only switch, off by default
    assert p.clock.mode == "paused"
    m = config.Machine(backend={"kind": "sim", "image": "openrua-sim-x", "simulator": {}})
    assert m.controller == "JOINT_POSITION" and m.cameras.rate_hz == 2.0
    assert m.control.gripper.settle_steps == 10
    assert m.backend.image == "openrua-sim-x"
    assert m.backend.sandbox_image == "openrua-sandbox-jazzy"
    h = config.Machine(backend={"kind": "sim", "ros_distro": "humble", "image": "openrua-sim-x",
                                "simulator": {}})
    assert h.backend.sandbox_image == "openrua-sandbox-humble"
    o = config.Machine(backend={"kind": "sim", "ros_distro": "humble", "sandbox_image": "mine",
                                "image": "openrua-sim-x", "simulator": {}})
    assert o.backend.sandbox_image == "mine"
    with pytest.raises(Exception):                    # the image is the composition's, never guessed
        config.Machine(backend={"kind": "sim", "simulator": {}})


def test_backend_union_is_strict():
    real = {"kind": "real", "discovery": {"network": "host"}}
    m = config.Machine(backend=real)
    assert isinstance(m.backend, config.RealBackend) and m.backend.launch is None
    assert m.backend.sandbox_image == "openrua-sandbox-jazzy"     # the robot's distro
    with pytest.raises(config.ConfigError) as e:      # a sim-only key on a real robot
        config.validate(config.Machine, {"backend": {**real, "image": "x"}}, "t")
    assert "image" in str(e.value)
    with pytest.raises(config.ConfigError) as e:      # discovery: exactly one way
        config.validate(config.Machine, {"backend": {"kind": "real", "discovery": {
            "network": "host", "static_peers": ["10.0.0.2"]}}}, "t")
    assert "exactly one" in str(e.value)
    with pytest.raises(config.ConfigError):           # kind is not optional
        config.validate(config.Machine, {"backend": {"image": "x"}}, "t")
    with pytest.raises(config.ConfigError) as e:      # an image needs a launch to run
        config.validate(config.Machine, {"backend": {**real, "image": "drv:foxy"}}, "t")
    assert "launch" in str(e.value)
    m = config.Machine(backend={**real, "image": "drv:foxy", "launch": "ros2 launch a b"})
    assert m.backend.image == "drv:foxy"


def test_user_config_layers_under_the_benchmark(tmp_path):
    (tmp_path / "config.yaml").write_text(
        "agents:\n  claude-code:\n    model: my-model\n"
        "    options: {effort: low, autocompact: 1M}\n    credentials_dir: /creds\n"
        "  codex: {model: gpt-x, credentials_dir: /codex}\n"
        "agent: claude-code\n")
    cfg = load_config("libero_pro", home=tmp_path)
    assert cfg["agent"]["model"] == "claude-opus-5"           # benchmark wins
    assert cfg["agent"]["options"] == {"effort": "high", "autocompact": "1M"}
    assert cfg["agent"]["credentials_dir"] == "/creds"        # benchmark silent: user's
    # --agent picks that agent's facts; the benchmark's section is claude-code's, so it is left out
    cfg = load_config("libero_pro", home=tmp_path, agent="codex")
    assert (cfg["agent"]["name"], cfg["agent"]["model"], cfg["agent"]["credentials_dir"]) == \
        ("codex", "gpt-x", "/codex") and cfg["agent"]["options"] == {}
    # a benchmark naming no robot takes the user's default
    b = tmp_path / "noname.yaml"
    b.write_text("entry_point: libero\ntask: {benchmark: libero_pro, suites: [libero_goal_task]}\n"
                 "simulator: robosuite\n")
    assert load_config(b, robot="panda", home=tmp_path)["machine"]["robot"]["model"] == "Franka Emika Panda"


def test_sandbox_run_args_come_from_the_defaults_file_only(tmp_path):
    assert load_config("libero_pro", home=tmp_path)["sandbox"] == {"run_args": []}
    (tmp_path / "config.yaml").write_text("sandbox:\n  run_args: ['--userns=keep-id']\n")
    cfg = load_config("libero_pro", home=tmp_path)
    assert cfg["sandbox"]["run_args"] == ["--userns=keep-id"]
    # a machine fact has no place in a benchmark config
    b = tmp_path / "b.yaml"
    b.write_text("entry_point: libero\ntask: {benchmark: libero_pro, suites: [libero_goal_task]}\n"
                 "robot: panda\nsimulator: robosuite\nsandbox: {run_args: ['--userns=keep-id']}\n")
    with pytest.raises(config.ConfigError) as e:
        load_config(b, home=tmp_path)
    assert "sandbox" in str(e.value)


def test_user_config_unknown_key_is_an_error(tmp_path):
    (tmp_path / "config.yaml").write_text("agnet: {model: x}\n")
    with pytest.raises(config.ConfigError) as e:
        load_config("libero_pro", home=tmp_path)
    assert "agnet" in str(e.value)


def test_benchmark_without_robot_or_machine_says_where_to_name_one(tmp_path):
    b = tmp_path / "noname.yaml"
    b.write_text("entry_point: libero\ntask: {benchmark: libero_pro, suites: [x]}\n")
    with pytest.raises(UsageError) as e:
        load_config(b, home=tmp_path)
    assert "config set --robot" in e.value.hint and "name a robot" in str(e.value)


def test_camera_list_key_round_trips(tmp_path):
    m = load_config("robocasa365", home=tmp_path)["machine"]
    assert m["cameras"]["list"] == ["robot0_agentview_left", "robot0_agentview_right",
                                    "robot0_eye_in_hand"]
    assert "names" not in m["cameras"]
    kind, t = load_robot("panda-omron")
    assert kind == "type" and "cameras" not in t and "backend" not in t


def test_schema_is_json_and_documents_fields():
    s = config.ResolvedConfig.model_json_schema()
    assert s["properties"]["protocol"]
    protocol = s["$defs"]["Protocol"]["properties"]
    assert "description" in protocol["resume_on_quota_wall"]


def test_a_benchmark_of_your_own_names_its_loader_by_path(tmp_path):
    (tmp_path / "my_loader.py").write_text("LOADER = object()\n")
    b = tmp_path / "mine.yaml"
    b.write_text("entry_point: ./my_loader.py\ntask: {benchmark: mine, suites: [s]}\n"
                 "robot: panda\nsimulator: robosuite\n")
    cfg = load_config(b, home=tmp_path)
    assert cfg["task"]["loader"] == str(tmp_path / "my_loader.py")
    b.write_text("entry_point: ./gone.py\ntask: {benchmark: mine, suites: [s]}\n"
                 "robot: panda\nsimulator: robosuite\n")
    with pytest.raises(NotFound, match="gone.py"):
        load_config(b, home=tmp_path)


def test_agent_and_model_arguments_win_over_the_file(tmp_path):
    """--agent picks another agent (the file's claude-code section is that
    agent's and is left out); --model overrides whichever model resulted."""
    cfg = load_config("libero_pro", home=tmp_path, agent="codex")
    assert cfg["agent"]["name"] == "codex"
    assert "model" not in cfg["agent"]                     # the runner takes codex's default
    assert cfg["agent"].get("options", {}).get("effort") != "high"   # the file's section is dropped
    cfg = load_config("libero_pro", home=tmp_path, agent="codex", model="gpt-6-astra")
    assert cfg["agent"]["model"] == "gpt-6-astra"
    cfg = load_config("libero_pro", home=tmp_path, model="claude-sonnet-5")
    assert cfg["agent"] ["name"] == "claude-code" and cfg["agent"]["model"] == "claude-sonnet-5"


def test_simulation_still_requires_a_task(tmp_path):
    cfg = compose("panda", "robosuite", home=tmp_path).cfg
    del cfg["task"]
    with pytest.raises(config.ConfigError, match="requires a benchmark task or native scene"):
        config.validate(config.ResolvedConfig, cfg, "missing scene")


def test_empty_selection_does_not_reenable_saved_benchmark(tmp_path):
    (tmp_path / 'config.yaml').write_text('agent: codex\n')
    native = compose('panda', 'robosuite', '', tmp_path)
    assert native.benchmark is None and native.suite == 'Lift'
    with pytest.raises(UsageError, match='name the simulator'):
        compose('panda-omron', '', '', tmp_path)
    with pytest.raises(UsageError, match='name a robot'):
        compose(None, None, None, tmp_path)


def test_benchmark_cannot_be_loaded_by_an_unrelated_simulator(tmp_path):
    with pytest.raises(config.ConfigError, match='requires simulator robosuite'):
        compose('panda', 'maniskill', 'libero_pro', tmp_path)


# ------------------------------------------------- gripper interface vs behaviour

_SIM = {"kind": "sim", "image": "openrua-sim-x", "simulator": {}}


def test_gripper_needs_its_behaviour_where_it_is_driven():
    with pytest.raises(config.ConfigError) as e:
        config.validate(config.Machine, {"backend": _SIM, "gripper": {
            "unit": "rad", "open": 0.0, "closed": 0.79, "max_effort": 50.0}}, "x")
    assert "holds_partway" in str(e.value) and "machine.gripper" in str(e.value)
    m = config.Machine(backend=_SIM, gripper={
        "unit": "rad", "open": 0.0, "closed": 0.79, "max_effort": 50.0,
        "holds_partway": True, "effort_honoured": False})
    assert m.gripper.closed > m.gripper.open          # closing may run either way


def test_old_gripper_keys_are_refused_with_the_rewrite():
    with pytest.raises(config.ConfigError) as e:
        config.validate(config.RobotType, {"robot": {"model": "x"}, "gripper": {
            "open_m": 0.04, "closed_m": 0.0, "max_effort": 30.0,
            "stops_at": ["open", "closed"]}}, "x")
    assert "open: <open_m>" in str(e.value) and "holds_partway: false" in str(e.value)
    with pytest.raises(config.ConfigError) as e:
        config.validate(config.Machine, {"backend": _SIM, "control": {
            "gripper": {"open_threshold_m": 0.02}}}, "x")
    assert "open_threshold_m was removed" in str(e.value)


def test_every_bundled_gripper_resolves_with_its_behaviour(tmp_path):
    # Each embodiment that drives a gripper says what it does with a goal.
    for name, sim in (("panda", "robosuite"), ("panda", "maniskill"), ("widowx", "maniskill"),
                      ("panda", "calvin"), ("panda", "vlabench")):
        try:
            cfg = config.compose(name, sim, home=tmp_path).cfg
        except UsageError:          # simulators without a native scene need a benchmark
            continue
        g = cfg["machine"]["gripper"]
        assert g["holds_partway"] is False and g["effort_honoured"] is False, (name, sim)


def test_a_real_robot_names_each_cameras_type(tmp_path):
    lab = tmp_path / "lab.yaml"
    base = ("machine:\n  backend: {kind: real, discovery: {network: host}}\n"
            "  robot: {model: Lab arm}\n")
    lab.write_text(base)
    with pytest.raises(config.ConfigError, match="lists its cameras"):
        compose(str(lab), home=tmp_path)
    lab.write_text(base + "  cameras: {list: [wrist]}\n")
    with pytest.raises(config.ConfigError, match="needs its type"):
        compose(str(lab), home=tmp_path)


def test_a_robot_that_installs_its_own_interfaces_gets_its_own_sandbox(tmp_path):
    lab = tmp_path / "lab.yaml"
    base = ("machine:\n  backend: {kind: real, ros_distro: humble, discovery: {network: host}%s}\n"
            "  robot: {model: Lab arm}\n  cameras: {list: []}\n")
    lab.write_text(base % "")
    assert compose(str(lab), home=tmp_path).cfg["machine"]["backend"]["sandbox_image"] \
        == "openrua-sandbox-humble"
    lab.write_text(base % ", sandbox_install: apt-get install -y ros-humble-franka-msgs")
    image = compose(str(lab), home=tmp_path).cfg["machine"]["backend"]["sandbox_image"]
    assert image.startswith("openrua-sandbox-humble-") and len(image.split("-")[-1]) == 8


def test_build_sandbox_for_a_robot_adds_its_install(tmp_path, monkeypatch):
    from openrua.cli.commands import build as build_cmd
    lab = tmp_path / "lab.yaml"
    lab.write_text("machine:\n  backend: {kind: real, ros_distro: humble, discovery: {network: host},"
                   " sandbox_install: apt-get install -y ros-humble-franka-msgs}\n"
                   "  robot: {model: Lab arm}\n  cameras: {list: []}\n")
    built, docker = {}, []
    monkeypatch.setattr(build_cmd, "build_sandbox", lambda **kw: built.update(kw) or (kw["tag"], "d"))
    monkeypatch.setattr(build_cmd, "_report_versions", lambda *a: None)
    monkeypatch.setattr(build_cmd.subprocess, "run", lambda argv, **kw: docker.append(argv))
    tag, _ = build_cmd._sandbox(["codex@0.161.0"], None, "jazzy", None, None, tmp_path, str(lab))
    assert built["distro"] == "humble" and tag.startswith("openrua-sandbox-humble-")
    # built as a candidate, tagged for real only once its CLIs checked out
    assert built["tag"].startswith(f"{tag}-candidate-") and ["docker", "tag", built["tag"], tag] in docker
    assert built["preinstall"].endswith("&& apt-get install -y ros-humble-franka-msgs")
    assert "openrua.machine_install_sha256" in built["labels"]
    assert built["stamp"] == ""                       # pinned: the cache may serve
    build_cmd._sandbox(["codex"], None, "jazzy", None, None, tmp_path, str(lab))
    assert built["stamp"]                             # unpinned: a fresh install


def test_a_real_robot_ignores_the_saved_simulation_defaults(tmp_path):
    # The docs told real-robot users to pass --sim '' --bench '' so the
    # saved default benchmark would not turn a free session into a trial.
    lab = tmp_path / "lab.yaml"
    lab.write_text("machine:\n  backend: {kind: real, discovery: {network: host}}\n"
                   "  robot: {model: Lab arm}\n  cameras: {list: []}\n")
    (tmp_path / "config.yaml").write_text("agent: codex\n")
    c = compose(str(lab), home=tmp_path)
    assert (c.simulator, c.benchmark) == (None, None) and "task" not in c.cfg
    named = compose(str(lab), None, "libero_pro", tmp_path)    # named: still honoured
    assert named.cfg["task"]["benchmark"] == "libero_pro"


def test_a_launch_runs_where_its_file_is_unless_told_otherwise(tmp_path):
    lab = tmp_path / "rig" / "lab.yaml"
    lab.parent.mkdir()
    lab.write_text("machine:\n  backend: {kind: real, launch: python3 driver/up.py,"
                   " discovery: {network: host}}\n  robot: {model: Lab arm}\n  cameras: {list: []}\n")
    backend = compose(str(lab), home=tmp_path).cfg["machine"]["backend"]
    assert backend["cwd"] == str(lab.parent) and backend["stop_timeout_s"] == 30.0
