"""config: the schema every file is held to, and how the layers combine."""
from pathlib import Path

import pytest
import yaml

from robocli import config
from robocli.config import paths
from robocli.bench.run import apply_suite_overrides, load_config, load_robot, normalize_arms

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
    cfg = load_config("libero_pro", robot="panda-omron-sim", home=tmp_path)
    assert "robot" not in cfg and cfg["machine"]["robot"]["model"].startswith("Franka Emika Panda on Omron")


def test_null_in_an_override_deletes_the_key(tmp_path):
    cfg = load_config("capbench", home=tmp_path)
    view = apply_suite_overrides(dict(yaml.safe_load(yaml.safe_dump(cfg))), "capbench_wipe")
    assert "gripper" not in view["machine"]
    assert "gripper" not in view["machine"]["ports"]
    assert view["machine"]["robot"]["model"].startswith("Franka Emika Panda with fixed")


def test_unknown_key_fails_loud_with_its_path(tmp_path):
    bad = tmp_path / "b.yaml"
    bad.write_text("task: {benchmark: libero_pro, suites: [x]}\n"
                   "protocol: {max_turn: 3}\nrobot: panda-sim\n")
    with pytest.raises(config.ConfigError) as e:
        load_config(bad, home=tmp_path)
    assert "protocol.max_turn" in str(e.value) and str(bad) in str(e.value)


def test_bad_override_key_fails_loud(tmp_path):
    cfg = load_config("libero_pro", home=tmp_path)
    cfg["suite_overrides"] = {"libero_goal_task": {"machine": {"grippr": None, "ports": {"twst": "/x"}}}}
    with pytest.raises(config.ConfigError) as e:
        apply_suite_overrides(cfg, "libero_goal_task")
    assert "machine.ports.twst" in str(e.value)


def test_defaults_match_the_paper_runs():
    p = config.Protocol()
    assert (p.max_turns, p.active_wall_clock_minutes, p.trials_per_task) == (500, 240, 10)
    assert p.resume_on_quota_wall is False          # scale-only switch, off by default
    assert p.clock.mode == "paused"
    m = config.Machine(backend={"kind": "sim", "simulator": {"venv": "x/.venv"}})
    assert m.controller == "JOINT_POSITION" and m.cameras.rate_hz == 2.0
    assert m.control.gripper.open_threshold_m == 0.02
    assert m.backend.image == "robocli-sim-jazzy"


def test_backend_union_is_strict():
    real = {"kind": "real", "discovery": {"network": "host"}}
    m = config.Machine(backend=real)
    assert isinstance(m.backend, config.RealBackend) and m.backend.launch is None
    with pytest.raises(config.ConfigError) as e:      # a sim-only key on a real robot
        config.validate(config.Machine, {"backend": {**real, "image": "x"}}, "t")
    assert "image" in str(e.value)
    with pytest.raises(config.ConfigError) as e:      # discovery: exactly one way
        config.validate(config.Machine, {"backend": {"kind": "real", "discovery": {
            "network": "host", "static_peers": ["10.0.0.2"]}}}, "t")
    assert "exactly one" in str(e.value)
    with pytest.raises(config.ConfigError):           # kind is not optional
        config.validate(config.Machine, {"backend": {"image": "x"}}, "t")


def test_user_config_layers_under_the_benchmark(tmp_path):
    (tmp_path / "config.yaml").write_text(
        "agent:\n  model: my-model\n  options: {effort: low, autocompact: 1M}\n"
        "  credentials_dir: /creds\nrobot: panda-sim\n")
    cfg = load_config("libero_pro", home=tmp_path)
    assert cfg["agent"]["model"] == "claude-opus-5"           # benchmark wins
    assert cfg["agent"]["options"] == {"effort": "high", "autocompact": "1M"}
    assert cfg["agent"]["credentials_dir"] == "/creds"        # benchmark silent: user's
    # a benchmark naming no robot takes the user's default
    b = tmp_path / "noname.yaml"
    b.write_text("task: {benchmark: libero_pro, suites: [libero_goal_task]}\n")
    assert load_config(b, home=tmp_path)["machine"]["robot"]["model"] == "Franka Emika Panda"


def test_user_config_unknown_key_is_an_error(tmp_path):
    (tmp_path / "config.yaml").write_text("agnet: {model: x}\n")
    with pytest.raises(config.ConfigError) as e:
        load_config("libero_pro", home=tmp_path)
    assert "agnet" in str(e.value)


def test_benchmark_without_robot_or_machine_says_where_to_name_one(tmp_path):
    b = tmp_path / "noname.yaml"
    b.write_text("task: {benchmark: libero_pro, suites: [x]}\n")
    with pytest.raises(config.ConfigError) as e:
        load_config(b, home=tmp_path)
    assert "config.yaml" in str(e.value) and "--robot" in str(e.value)


def test_robot_profile_camera_list_key_round_trips():
    m = load_robot("panda-omron-sim")["machine"]
    assert m["cameras"]["list"] == ["robot0_agentview_left", "robot0_agentview_right",
                                    "robot0_eye_in_hand"]
    assert "names" not in m["cameras"]


def test_schema_is_json_and_documents_fields():
    s = config.ResolvedConfig.model_json_schema()
    assert s["properties"]["protocol"]
    protocol = s["$defs"]["Protocol"]["properties"]
    assert "description" in protocol["resume_on_quota_wall"]
