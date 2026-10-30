"""``openrua exec``: one recorded run of the agent on a task, simulated or real."""

import pytest

from openrua.cli import build_parser
from openrua.cli.commands import execute
from openrua.config import projects
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


@pytest.fixture(autouse=True)
def project_root(tmp_path, monkeypatch):
    root = tmp_path / 'project'
    root.mkdir()
    monkeypatch.chdir(root)
    projects.trust(root, tmp_path / 'home')
    return root


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
    assert trials.project == (tmp_path / 'project').resolve()
    # A recorded run limits the agent to its model service, as a benchmark run does.
    assert cfg["protocol"]["limits"]["internet"] == "model-only"


def test_a_simulated_robots_task_comes_from_its_scene(tmp_path, ran):
    execute.run(parse(tmp_path, "panda", "--sim", "robosuite", "--internet", "open"))
    cfg, source, trials, label = ran[0]
    assert label == "panda-robosuite" and source.name == "robosuite.yaml"
    assert trials.task is None and trials.internet == "open"
    assert trials.run_id.startswith("exec-")
    # Project tasks can override the scene's sentence, while keeping its scoring.
    execute.run(parse(tmp_path, "panda", "lift the cube", "--sim", "robosuite"))
    assert ran[-1][2].task == "lift the cube"
    execute.run(parse(tmp_path, "panda", "a sentence", "--bench", "libero_pro"))
    assert ran[-1][3] == 'libero_pro' and ran[-1][2].task == 'a sentence'


def test_a_real_robot_needs_its_task(tmp_path, ran):
    file = tmp_path / "arm.yaml"
    file.write_text(REAL)
    with pytest.raises(UsageError, match="a sentence"):
        execute.run(parse(tmp_path, str(file)))
    assert ran == []


def test_recording_check_and_video_flags_reach_the_runner(tmp_path, ran):
    file = tmp_path / 'arm.yaml'
    file.write_text(REAL)
    execute.run(parse(tmp_path, str(file), 'inspect', '--record-wait', '7',
                      '--record-fps', '10', '--no-video'))
    trials = ran[0][2]
    assert trials.record == () and trials.record_wait == 7
    assert trials.record_fps == 10
    assert trials.no_video is True
    execute.run(parse(tmp_path, str(file), 'inspect'))
    assert ran[-1][2].record_wait is None and ran[-1][2].no_video is False
    assert ran[-1][2].record_fps is None


def test_explicit_agent_uses_its_own_model_in_an_existing_project(tmp_path, project_root, ran, monkeypatch):
    monkeypatch.setattr(execute.updates, 'notice', lambda *a: None)
    projects.save_selection(project_root, dict(robot='panda', sim='robosuite', bench='', agent='codex', model='codex-only-model'))
    (tmp_path / 'home/config.yaml').write_text('agents:\n  claude-code:\n    model: claude-project-default\n')
    execute.run(parse(tmp_path, 'inspect', '--agent', 'claude-code'))
    assert ran[0][0]['agent']['model'] == 'claude-project-default'


def test_a_sentence_names_the_simulators_own_scene(tmp_path, ran):
    (tmp_path / "home" / "config.yaml").write_text("agent: claude-code\n")
    execute.run(parse(tmp_path, "panda", "lift the cube", "--sim", "robosuite"))
    assert ran[-1][3] == "panda-robosuite" and ran[-1][2].task == "lift the cube"


def test_task_alone_reuses_project_settings_over_global_defaults(tmp_path, project_root, ran):
    projects.save_selection(project_root, dict(robot='panda', sim='robosuite', bench='',
                                               agent='codex@0.134.0', model='project-model'))
    (tmp_path / 'home/config.yaml').write_text('agent: claude-code\n')
    execute.run(parse(tmp_path, 'run scripts/grasp.py'))
    cfg, _, trials, _ = ran[-1]
    assert trials.task == 'run scripts/grasp.py'
    assert cfg['machine']['backend']['kind'] == 'sim'
    assert cfg['agent']['name'] == 'codex@0.134.0'
    assert cfg['agent']['model'] == 'project-model'
    assert projects.selection(project_root)['agent'] == 'codex@0.134.0'
    assert projects.selection(project_root)['sim'] == str(project_root / '.openrua/configs/simulators/robosuite.yaml')


def test_project_real_robot_can_run_a_task_without_repeating_its_profile(tmp_path, project_root, ran):
    profile = project_root / 'arm.yaml'
    profile.write_text(REAL)
    projects.save_selection(project_root, dict(robot=str(profile), sim='', bench='',
                                               agent='codex', model=''))
    execute.run(parse(tmp_path, 'inspect'))
    cfg, _, trials, _ = ran[-1]
    assert cfg['machine']['backend']['kind'] == 'real'
    assert trials.task == 'inspect' and trials.project == project_root


@pytest.mark.parametrize('position', ['before', 'after'])
@pytest.mark.parametrize('flag', ['--workspace', '-C', '--cd'])
def test_explicit_workspace_is_consumed_on_either_side_of_exec(tmp_path, ran, position, flag):
    other = tmp_path / 'other'
    options = [flag, str(other), '--trust-workspace']
    words = ['exec', '--robot', 'panda', 'inspect', '--sim', 'robosuite']
    argv = options + words if position == 'before' else words + options
    args = build_parser().parse_args(['--home', str(tmp_path / 'home'), *argv])
    execute.run(args)
    assert ran[-1][2].project == other.resolve()
    assert projects.trusted(other, tmp_path / 'home')


def test_untrusted_unattended_workspace_fails_without_writing(tmp_path, monkeypatch, ran):
    monkeypatch.setattr('sys.stdin.isatty', lambda: False)
    other = tmp_path / 'not-created'
    with pytest.raises(UsageError, match='not been trusted'):
        execute.run(parse(tmp_path, '--workspace', str(other), 'panda', '--sim', 'robosuite'))
    assert not other.exists() and not ran


def test_explicit_real_robot_drops_the_projects_saved_simulator(tmp_path, project_root, ran):
    projects.save_selection(project_root, dict(robot='panda', sim='robosuite', bench='libero_pro',
                                               agent='codex', model=''))
    profile = project_root / 'arm.yaml'
    profile.write_text(REAL)
    execute.run(parse(tmp_path, str(profile), 'inspect'))
    assert ran[-1][0]['machine']['backend']['kind'] == 'real'
    assert projects.selection(project_root)['sim'] == str(project_root / '.openrua/configs/simulators/robosuite.yaml')


def test_a_new_project_task_keeps_the_saved_scene_and_is_not_a_robot_path(tmp_path, project_root, ran):
    projects.save_selection(project_root, dict(robot='panda', sim='robosuite', bench='libero_pro',
                                               agent='codex', model=''))
    execute.run(parse(tmp_path, 'read notes/machine.yaml'))
    assert ran[-1][3] == 'libero_pro'
    assert ran[-1][2].task == 'read notes/machine.yaml'
    assert projects.selection(project_root)['bench'] == str(project_root / '.openrua/configs/benchmarks/libero_pro.yaml')
