"""Preserve user skill files while bridging native project directories."""

import json
from pathlib import Path

import pytest

from openrua import skills
from openrua.cli import main
from openrua.config import projects


def write_skill(root, name='camera-check', description='Check current camera images.'):
    path = root / name
    path.mkdir(parents=True)
    (path / 'SKILL.md').write_text(f'---\nname: {name}\ndescription: {description}\n---\nRead current observations.\n')
    (path / 'run.py').write_text('print("checked")\n')
    return path


def test_new_native_directories_share_one_editable_source(tmp_path):
    assert skills.bridge(tmp_path, ('.claude/skills', '.agents/skills')) == []
    common = tmp_path / skills.PROJECT_DIRECTORY
    alias = tmp_path / '.claude/skills'
    method = write_skill(alias)
    assert method.resolve() == common / method.name
    (common / method.name / 'run.py').write_text('print("updated")\n')
    assert (method / 'run.py').read_text() == 'print("updated")\n'
    skills.bridge(tmp_path, ('.claude/skills',))
    assert len(skills.catalog([('project', common), ('project', alias)])) == 1


def test_existing_native_files_survive_and_new_skills_join_next_session(tmp_path):
    common = tmp_path / skills.PROJECT_DIRECTORY
    native = tmp_path / '.claude/skills'
    first = write_skill(common, 'common-method')
    second = write_skill(native, 'native-method')
    assert skills.bridge(tmp_path, ('.claude/skills',)) == []
    assert not native.is_symlink()
    assert (common / second.name).resolve() == second
    assert (native / first.name).resolve() == first
    third = write_skill(native, 'later-method')
    skills.bridge(tmp_path, ('.claude/skills',))
    assert (common / third.name).resolve() == third


def test_conflicts_are_reported_without_replacing_either_definition(tmp_path):
    first = write_skill(tmp_path / skills.PROJECT_DIRECTORY)
    second = write_skill(tmp_path / '.claude/skills', description='Different method.')
    before = [(path / 'SKILL.md').read_bytes() for path in (first, second)]
    assert skills.bridge(tmp_path, ('.claude/skills',))
    assert [(path / 'SKILL.md').read_bytes() for path in (first, second)] == before


def test_bridge_refuses_external_symlink_instead_of_writing_other_projects(tmp_path):
    root = tmp_path / 'project'; root.mkdir()
    outside = tmp_path / 'outside'; outside.mkdir()
    (root / '.agents').symlink_to(outside)
    with pytest.raises(ValueError, match='inside'):
        skills.bridge(root, ('.claude/skills',))
    assert list(outside.iterdir()) == []


def test_invalid_metadata_is_visible_and_not_imported(tmp_path):
    path = tmp_path / 'source' / 'invalid'
    path.mkdir(parents=True)
    (path / 'SKILL.md').write_text('No front matter')
    entries = skills.catalog([('project', path.parent)])
    assert entries[0].error and entries[0].name == 'invalid'
    with pytest.raises(ValueError, match='not a valid'):
        skills.install(path, tmp_path / 'destination')
    assert not (tmp_path / 'destination').exists()


def test_personal_import_copies_resources_and_never_overwrites(tmp_path):
    source = write_skill(tmp_path / 'source')
    destination = skills.install(source / 'SKILL.md', tmp_path / 'personal')
    assert (destination / 'run.py').read_bytes() == (source / 'run.py').read_bytes()
    (destination / 'run.py').write_text('user edit')
    with pytest.raises(ValueError, match='already exists'):
        skills.install(source, destination.parent)
    assert (destination / 'run.py').read_text() == 'user edit'


@pytest.mark.parametrize('mode', skills.MODES)
def test_project_policy_changes_guidance_without_removing_existing_skills(tmp_path, mode):
    method = write_skill(tmp_path / skills.PROJECT_DIRECTORY)
    original = (method / 'SKILL.md').read_bytes()
    skills.set_mode(tmp_path, mode)
    assert skills.mode(tmp_path) == mode
    assert f'Accumulation policy: {mode}' in skills.instructions(tmp_path, [])
    assert (method / 'SKILL.md').read_bytes() == original


def test_invalid_policy_fails_loudly(tmp_path):
    skills.set_mode(tmp_path, 'manual')
    (tmp_path / '.openrua/skills.json').write_text('{"accumulation": "typo"}')
    with pytest.raises(ValueError, match='invalid skills policy'):
        skills.instructions(tmp_path, [])


def test_cli_read_only_listing_needs_no_trust_or_docker(tmp_path, capsys):
    home, root = tmp_path / 'home', tmp_path / 'project'
    write_skill(root / skills.PROJECT_DIRECTORY)
    assert main(['--home', str(home), 'skills', '-C', str(root), 'list', '--json']) == 0
    entries = json.loads(capsys.readouterr().out)
    assert entries[0]['scope'] == 'project' and entries[0]['error'] is None
    assert not home.exists() and not (root / '.openrua').exists()


def test_cli_mutation_requires_workspace_trust_but_personal_import_is_explicit(tmp_path, capsys):
    home, root = tmp_path / 'home', tmp_path / 'project'
    assert main(['--home', str(home), 'skills', '-C', str(root), 'policy', 'manual']) == 2
    assert not root.exists()
    assert main(['--home', str(home), 'skills', '-C', str(root), '--trust-workspace', 'policy', 'manual']) == 0
    assert projects.trusted(root, home) and skills.mode(root) == 'manual'
    source = write_skill(tmp_path / 'source')
    assert main(['--home', str(home), 'skills', 'add', str(source), '--personal']) == 0
    assert (home / 'skills' / source.name / 'run.py').is_file()
