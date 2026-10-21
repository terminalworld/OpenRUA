"""``openrua exec``: one recorded run of the agent on a task, simulated or real."""

import pytest

from openrua.cli import build_parser
from openrua.cli.commands import execute
from openrua.errors import UsageError

REAL = """machine:
  backend:
    kind: real
    ros_distro: jazzy
    discovery:
      network: host
  robot:
    model: Fake arm
    description: two joints
  cameras:
    list: []
"""


def parse(tmp_path, *argv):
    return build_parser().parse_args(["--home", str(tmp_path / "home"), "exec", *argv])


@pytest.fixture
def ran(monkeypatch):
    calls = []
    monkeypatch.setattr(execute, "run_set", lambda cfg, source, trials, home, label:
                        calls.append((cfg, source, trials, label)) or 0)
    return calls


def test_a_real_robots_task_is_the_sentence_and_its_rules_are_a_runs(tmp_path, ran):
    file = tmp_path / "arm.yaml"
    file.write_text(REAL)
    assert execute.run(parse(tmp_path, str(file), "put the cup on the plate", "--run-id", "hw-1")) == 0
    cfg, source, trials, label = ran[0]
    assert source == file.resolve() and label == "arm"
    assert (trials.task, trials.task_suite, trials.task_ids, trials.seeds, trials.run_id) \
        == ("put the cup on the plate", "task", [0], [0], "hw-1")
    assert trials.operator == "agent"
    # A recorded run limits the agent to its model service, as a benchmark run does.
    assert cfg["protocol"]["limits"]["internet"] == "model-only"


def test_a_simulated_robots_task_comes_from_its_scene(tmp_path, ran):
    execute.run(parse(tmp_path, "panda", "--sim", "robosuite", "--internet", "open"))
    cfg, source, trials, label = ran[0]
    assert label == "panda-robosuite" and source.name == "robosuite.yaml"
    assert trials.task is None and trials.internet == "open"
    assert trials.run_id.startswith("exec-")
    # The simulator's own scene takes a sentence; a benchmark scene states its own.
    execute.run(parse(tmp_path, "panda", "lift the cube", "--sim", "robosuite"))
    assert ran[-1][2].task == "lift the cube"
    with pytest.raises(UsageError, match="states its own task"):
        execute.run(parse(tmp_path, "panda", "a sentence", "--bench", "libero_pro"))


def test_a_real_robot_needs_its_task(tmp_path, ran):
    file = tmp_path / "arm.yaml"
    file.write_text(REAL)
    with pytest.raises(UsageError, match="a sentence"):
        execute.run(parse(tmp_path, str(file)))
    assert ran == []


def test_a_sentence_names_the_simulators_own_scene_over_a_saved_benchmark(tmp_path, ran):
    (tmp_path / "home").mkdir()
    (tmp_path / "home" / "config.yaml").write_text("benchmark: libero_pro\n")
    execute.run(parse(tmp_path, "panda", "lift the cube", "--sim", "robosuite"))
    assert ran[-1][3] == "panda-robosuite" and ran[-1][2].task == "lift the cube"
