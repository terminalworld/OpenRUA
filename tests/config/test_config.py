"""config: the schema every file is held to, and how the layers combine."""
from pathlib import Path

import pytest
import yaml

from openrua import config
from openrua.errors import UsageError
from openrua.config import paths
from openrua.config import (apply_suite_overrides, compose, load_config, load_robot,
                            load_simulator, normalize_arms)

BUNDLED = [e.name for e in paths.available("benchmarks")]


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
    assert m["backend"]["simulator"]["venv"] == "cap-x/.venv-libero"   # libero_pro's install
    assert m["backend"]["ros_distro"] == "jazzy" and m["backend"]["image"] == "openrua-sim-jazzy"
    assert m["cameras"]["record"] == ["agentview", "robot0_eye_in_hand"]  # scenes.cameras
    cap = load_config("capbench", home=tmp_path)["machine"]
    assert cap["controller_kp_scale"] == 1                       # the engine's own value
    assert cap["backend"]["ros_distro"] == "humble"


def test_native_scene_without_a_benchmark(tmp_path):
    c = compose("panda", "robosuite", None, tmp_path)
    assert (c.benchmark, c.suite, c.task_id) == (None, "Lift", 0)
    assert c.cfg["task"] == {"benchmark": "robosuite", "suites": ["Lift"],
                             "init_states": "seeded-reset", "split": "target", "task_language": {}}
    assert c.cfg["machine"]["backend"]["simulator"]["venv"] == "cap-x/.venv-capbench"
    assert c.cfg["agent"]["name"] == "claude-code"              # the defaults files


def test_robot_type_alone_asks_for_a_simulator(tmp_path):
    with pytest.raises(UsageError) as e:
        compose("panda", None, None, tmp_path)
    assert "--sim" in str(e.value)


def test_real_instance_brings_its_machine_over_its_type(tmp_path):
    (tmp_path / "robots").mkdir()
    (tmp_path / "robots" / "lab-panda.yaml").write_text(
        "type: panda\n"
        "machine:\n"
        "  backend: {kind: real, ros_distro: humble, discovery: {network: host}}\n"
        "  cameras: {list: [wrist]}\n"
        "  robot: {model: Our lab Panda}\n")
    # a real robot alone has no scene to load; with a benchmark it runs its tasks
    with pytest.raises(UsageError):
        compose("lab-panda", None, None, tmp_path)
    c = compose("lab-panda", None, "libero_pro", tmp_path)
    m = c.cfg["machine"]
    assert c.simulator is None and m["backend"]["kind"] == "real"
    assert m["robot"]["model"] == "Our lab Panda"               # instance over type
    assert m["robot"]["description"].startswith("7-joint")      # type's line kept
    assert m["arm"]["joints"][0] == "panda_joint1" and m["cameras"]["list"] == ["wrist"]
    assert m["backend"]["sandbox_image"] == "openrua-sandbox-humble"
    with pytest.raises(UsageError):                            # an instance takes no --sim
        compose("lab-panda", "robosuite", None, tmp_path)


def test_simulator_file_is_validated(tmp_path):
    s = load_simulator("robosuite", tmp_path)
    assert s["engine"].startswith("robosuite") and "panda" in s["robots"]
    assert s["native"]["loader"] == "robosuite"
    (tmp_path / "simulators").mkdir()
    bad = tmp_path / "simulators" / "x.yaml"
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
    m = config.Machine(backend={"kind": "sim", "simulator": {"venv": "x/.venv"}})
    assert m.controller == "JOINT_POSITION" and m.cameras.rate_hz == 2.0
    assert m.control.gripper.open_threshold_m == 0.02
    assert m.backend.image == "openrua-sim-jazzy"
    assert m.backend.sandbox_image == "openrua-sandbox-jazzy"
    h = config.Machine(backend={"kind": "sim", "ros_distro": "humble",
                                "simulator": {"venv": "x/.venv"}})
    assert (h.backend.image, h.backend.sandbox_image) == ("openrua-sim-humble", "openrua-sandbox-humble")
    o = config.Machine(backend={"kind": "sim", "ros_distro": "humble", "sandbox_image": "mine",
                                "simulator": {"venv": "x/.venv"}})
    assert (o.backend.image, o.backend.sandbox_image) == ("openrua-sim-humble", "mine")


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
        "agent:\n  model: my-model\n  options: {effort: low, autocompact: 1M}\n"
        "  credentials_dir: /creds\nrobot: panda\n")
    cfg = load_config("libero_pro", home=tmp_path)
    assert cfg["agent"]["model"] == "claude-opus-5"           # benchmark wins
    assert cfg["agent"]["options"] == {"effort": "high", "autocompact": "1M"}
    assert cfg["agent"]["credentials_dir"] == "/creds"        # benchmark silent: user's
    # a benchmark naming no robot takes the user's default
    b = tmp_path / "noname.yaml"
    b.write_text("task: {benchmark: libero_pro, suites: [libero_goal_task]}\n"
                 "simulator: robosuite\n")
    assert load_config(b, home=tmp_path)["machine"]["robot"]["model"] == "Franka Emika Panda"


def test_sandbox_run_args_come_from_the_defaults_file_only(tmp_path):
    assert load_config("libero_pro", home=tmp_path)["sandbox"] == {"run_args": []}
    (tmp_path / "config.yaml").write_text("sandbox:\n  run_args: ['--userns=keep-id']\n")
    cfg = load_config("libero_pro", home=tmp_path)
    assert cfg["sandbox"]["run_args"] == ["--userns=keep-id"]
    # a machine fact has no place in a benchmark config
    b = tmp_path / "b.yaml"
    b.write_text("task: {benchmark: libero_pro, suites: [libero_goal_task]}\n"
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
    b.write_text("task: {benchmark: libero_pro, suites: [x]}\n")
    with pytest.raises(UsageError) as e:
        load_config(b, home=tmp_path)
    assert "config set robot" in e.value.hint and "name a robot" in str(e.value)


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
