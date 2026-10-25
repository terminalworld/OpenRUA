"""Owned configurations remain usable after copying and editing a project."""

import json
import shutil
from pathlib import Path

import pytest
import yaml

from openrua import config
from openrua.config import paths, projects
from openrua.config.inputs import driver_images


@pytest.mark.parametrize('benchmark', [entry.name for entry in paths.available('benchmarks')])
def test_all_bundled_benchmarks_become_movable_project_inputs(tmp_path, benchmark):
    original, moved = tmp_path / 'original', tmp_path / 'moved'
    projects.save_selection(original, dict(robot='', sim='', bench=benchmark, agent='codex', model=''))
    shutil.copytree(original, moved)
    shutil.rmtree(original)
    choices = projects.selection(moved)
    result = config.compose(choices['robot'], choices['sim'], choices['bench'], tmp_path / 'fresh-home')
    assert result.benchmark == benchmark
    assert result.source == choices['bench']
    assert not str(original) in json.dumps(result.cfg)
    for field in ('robot', 'sim', 'bench'):
        if choices[field]:
            assert Path(choices[field]).is_relative_to(moved)


def test_project_changes_do_not_change_another_projects_sources_or_images(tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    choices = dict(robot='panda', sim='robosuite', bench='', agent='codex', model='')
    projects.save_selection(a, choices); projects.save_selection(b, choices)
    before = config.image_owner(projects.selection(b)['sim'])
    file = a / '.openrua/configs/simulators/robosuite/requirements.txt'
    file.write_text(file.read_text() + '\n# Project A changes its build inputs\n')
    assert config.image_owner(projects.selection(a)['sim']) != before
    assert config.image_owner(projects.selection(b)['sim']) == before
    own = a / '.openrua/configs/robots/panda.yaml'
    own.write_text(own.read_text() + '\n# Maintained by this project\n')
    projects.save_selection(a, choices)
    assert own.read_text().endswith('# Maintained by this project\n')


def test_cli_overrides_do_not_change_saved_selection_and_use_the_callers_base(tmp_path):
    root, caller = tmp_path / 'project', tmp_path / 'caller'
    caller.mkdir()
    projects.save_selection(root, dict(robot='panda', sim='robosuite', bench='', agent='codex', model='saved'))
    before = projects.settings_path(root).read_bytes()
    file = caller / 'arm.yaml'
    file.write_text('machine: {backend: {kind: real, discovery: {network: host}}, robot: {model: Test}, cameras: {list: []}}\n')
    choices = projects.options(root, dict(robot='./arm.yaml', agent='claude-code'), relative_to=caller)
    assert choices['robot'] == str(file) and choices['sim'] == choices['bench'] == ''
    assert choices['model'] is None
    assert projects.settings_path(root).read_bytes() == before


def test_driver_image_slots_follow_project_build_inputs(tmp_path):
    context = tmp_path / 'driver'; context.mkdir()
    file = context / 'Dockerfile'; file.write_text('FROM scratch\n')
    source = dict(build={'driver': str(context)}, image='driver', launch='python3 driver/up.py --image {images[driver]}')
    first = dict(source); driver_images(first)
    assert first['image'] in first['launch'] and first['image'] in first['build']
    file.write_text('FROM scratch\nLABEL project=changed\n')
    second = dict(source); driver_images(second)
    assert first['image'] != second['image']


def test_project_path_symlinks_cannot_redirect_template_writes(tmp_path):
    outside = tmp_path / 'outside'; outside.mkdir()
    root = tmp_path / 'project'; root.mkdir()
    (root / '.openrua').mkdir()
    (root / '.openrua/configs').symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match='leaves'):
        projects.save_selection(root, dict(robot='panda', sim='robosuite', bench='', agent='codex', model=''))
    assert list(outside.iterdir()) == []


def test_image_identity_does_not_change_the_cached_recipe_commands(tmp_path):
    from openrua.cli.commands.build import simulator_targets
    from openrua.robot.sim import install
    root = tmp_path / 'project'
    projects.save_selection(root, dict(robot='panda', sim='robosuite', bench='', agent='codex', model=''))
    simulator = projects.selection(root)['sim']
    name, recipe, tag = simulator_targets([], [simulator], False, tmp_path / 'home')[0]
    assert name == 'robosuite' and tag.startswith('openrua-sim-robosuite-')
    local, _ = install.render(recipe, name, tmp_path / 'no-checkout')
    bundled, _ = install.render(config.install_for('robosuite'), 'robosuite', tmp_path / 'no-checkout')
    assert local == bundled


def test_generated_inputs_leave_the_users_config_directory_alone(tmp_path):
    root = tmp_path / 'project'
    own = root / 'configs' / 'task.yaml'
    own.parent.mkdir(parents=True)
    own.write_text('task: user-owned\n')
    projects.save_selection(root, dict(robot='panda', sim='robosuite', bench='', agent='codex', model=''))
    assert list(own.parent.iterdir()) == [own]
    assert own.read_text() == 'task: user-owned\n'
    assert (root / '.openrua/configs/robots/panda.yaml').is_file()
    assert '.openrua/configs' in projects.selection(root)['robot']


def test_explicit_saved_inputs_keep_their_location_when_project_settings_change(tmp_path):
    root = tmp_path / 'project'
    projects.save_selection(root, dict(robot='panda', sim='robosuite', bench='', agent='codex', model=''))
    original = projects.selection(root)
    custom = root / 'configs'
    shutil.move(str(root / '.openrua/configs'), custom)
    choices = {**original, 'robot': str(custom / 'robots/panda.yaml'),
               'sim': str(custom / 'simulators/robosuite.yaml'), 'model': 'another-model'}
    projects.save_selection(root, choices)
    assert projects.selection(root)['robot'] == choices['robot']
    assert projects.selection(root)['sim'] == choices['sim']
    assert not (root / '.openrua/configs').exists()
    result = config.compose(choices['robot'], choices['sim'], '', tmp_path / 'home')
    assert result.robot == choices['robot']
    assert result.cfg['machine']['robot']['model'] == 'Franka Emika Panda'
