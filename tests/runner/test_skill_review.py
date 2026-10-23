"""Project ownership, proposal isolation and recoverable review outcomes."""

import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from openrua import agents, reviews
from openrua.cli import main
from openrua.runner import live_state, skill_review
from openrua.runner.skill_review_sources import select
from openrua.sessions.core import initial_state
from openrua.sessions.store import SQLiteStore
from tests.test_skills import write_skill


def conversation(home, name, project, *, closed=True, text='Check the camera.'):
    state = initial_state()
    state['closed'] = closed
    state['messages'] = [{'text': text, 'status': 'completed'}]
    path = home / 'sandboxes' / name / 'conversation.sqlite'
    store = SQLiteStore(path, state)
    for index in range(5):
        store.update(lambda value, index=index: (None, [{'kind': 'example', 'text': f'result {index}'}]))
    store.close()
    live_state.save(name, home, project=str(project), status='stopped' if closed else 'running')
    return path


@pytest.fixture
def apparatus(tmp_path, monkeypatch):
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir()
    (root / 'README.md').write_text('Maintained robot project.\n')
    conversation(home, 'closed', root)
    conversation(home, 'active', root, closed=False, text='Do not include active work.')
    conversation(home, 'other', tmp_path / 'other', text='PRIVATE OTHER PROJECT')
    folder = root / '.openrua/runs/one/trials/task/seed0'; folder.mkdir(parents=True)
    (folder / 'result.json').write_text(json.dumps({'project': str(root), 'termination': 'self_finished', 'success': False}))
    (folder / 'transcript.jsonl').write_text('Previous task found a repeatable camera check.\n')
    key = tmp_path / 'test-key'; key.write_text('synthetic-review-key-not-used-for-a-model')
    adapter = agents.get('codex')
    settings = {'auth': {'mode': 'api', 'key_file': str(key)}}
    calls = []
    def worker(adapter, profile, workspace, inputs, runtime, image, model, options, seconds, max_turns, output, details):
        calls.append((workspace, inputs, runtime))
        method = write_skill(workspace / 'proposed/.agents/skills')
        (workspace / 'summary.md').write_text('Proposed camera check based on the completed records.')
        return 'completed'
    monkeypatch.setattr(skill_review, '_worker', worker)
    def run(**kwargs):
        options = dict(image='fixture-image', model='fixture-model', since=None, max_records=20,
                       max_bytes=1024*1024, max_file_bytes=128*1024, max_files=200, max_events=100, seconds=10)
        return skill_review.run(root, home, adapter, settings, **{**options, **kwargs})
    return SimpleNamespace(root=root, home=home, run=run, calls=calls, worker=worker, adapter=adapter, settings=settings, key=key)


def test_review_uses_only_completed_work_and_returns_proposals_without_applying(apparatus):
    a = apparatus
    result = a.run()
    assert result['status'] == 'completed' and result['proposal_only']
    assert len(result['sources']) == 2
    output = Path(result['output'])
    assert (output / 'proposed/.agents/skills/camera-check/SKILL.md').is_file()
    assert '+name: camera-check' in (output / 'changes.diff').read_text()
    assert not (a.root / '.agents').exists() and not (a.root / '.claude').exists()
    inputs = Path(result['input_directory'])
    assert inputs.is_dir() and inputs.is_relative_to(a.home)
    text = '\n'.join(path.read_text() for path in inputs.rglob('*') if path.is_file())
    assert 'PRIVATE OTHER PROJECT' not in text and 'Do not include active work' not in text
    assert not (output / 'input').exists()
    assert not (inputs.parent / 'profile').exists() and not (inputs.parent / 'workspace').exists()


def test_active_conversations_do_not_consume_completed_record_limit(apparatus):
    a = apparatus
    sources, skipped = select(a.root, a.home, (), since=None, limit=1, max_file_bytes=100000, max_events=100)
    assert len(sources) == 1 and sources[0].name == 'sessions/closed'
    assert any('active' in row['reason'] for row in skipped)


def test_foreground_changes_are_retained_and_reported(apparatus, monkeypatch):
    a = apparatus
    original = write_skill(a.root / '.agents/skills') / 'SKILL.md'
    def worker(*args):
        workspace = args[2]
        path = workspace / 'proposed/.agents/skills/camera-check/SKILL.md'
        path.write_text(path.read_text() + '\nProposed improvement.\n')
        original.write_text('Foreground edit retained.\n')
        return 'completed'
    monkeypatch.setattr(skill_review, '_worker', worker)
    result = a.run()
    assert result['status'] == 'completed'
    assert original.read_text() == 'Foreground edit retained.\n'
    assert result['changes'][0]['source_changed'] is True
    assert 'Proposed improvement' in (Path(result['output']) / 'changes.diff').read_text()


def test_review_preserves_executable_resources_and_reports_mode_only_edits(apparatus, monkeypatch):
    a = apparatus
    source = write_skill(a.root / '.agents/skills') / 'check.sh'
    source.write_text('#!/bin/sh\nprintf "camera fixture\\n"\n')
    source.chmod(0o750)

    def worker(*args):
        workspace, inputs = args[2:4]
        relative = '.agents/skills/camera-check/check.sh'
        assert (inputs / 'project' / relative).stat().st_mode & 0o777 == 0o750
        assert (workspace / 'proposed' / relative).stat().st_mode & 0o777 == 0o750
        (workspace / 'proposed' / relative).chmod(0o755)
        return 'completed'

    monkeypatch.setattr(skill_review, '_worker', worker)
    result = a.run()
    assert result['status'] == 'completed', result
    assert source.stat().st_mode & 0o777 == 0o750
    change = next(row for row in result['changes'] if row['path'].endswith('check.sh'))
    assert change['before_sha256'] == change['after_sha256']
    assert (change['before_mode'], change['after_mode']) == (0o750, 0o755)
    assert not change['source_changed']
    assert (Path(result['output']) / 'proposed' / change['path']).stat().st_mode & 0o777 == 0o755


def test_permission_changes_make_a_new_input_set(apparatus, monkeypatch):
    a = apparatus
    source = write_skill(a.root / '.agents/skills') / 'SKILL.md'
    calls = []
    monkeypatch.setattr(skill_review, '_worker', lambda *a: calls.append(True) or 'completed')
    assert a.run()['status'] == 'completed'
    source.chmod(0o640)
    assert a.run()['status'] == 'completed'
    assert len(calls) == 2


def test_readonly_copies_are_redacted_without_changing_source_permissions(apparatus, monkeypatch):
    a = apparatus
    source = write_skill(a.root / '.agents/skills') / 'SKILL.md'
    secret = a.key.read_text()
    source.write_text(source.read_text() + '\n' + secret)
    source.chmod(0o444)

    def worker(*args):
        for root in (args[2] / 'proposed', args[3] / 'project'):
            path = root / '.agents/skills/camera-check/SKILL.md'
            assert path.stat().st_mode & 0o777 == 0o444
            assert secret not in path.read_text()
        return 'completed'

    monkeypatch.setattr(skill_review, '_worker', worker)
    result = a.run()
    assert result['status'] == 'completed', result
    assert source.stat().st_mode & 0o777 == 0o444 and secret in source.read_text()
    for root in ('baseline', 'proposed'):
        path = Path(result['output']) / root / '.agents/skills/camera-check/SKILL.md'
        assert path.stat().st_mode & 0o777 == 0o444
        assert secret not in path.read_text()


def test_linked_skill_keeps_its_actual_owner_and_supporting_files(apparatus, monkeypatch):
    a = apparatus
    owner = write_skill(a.root / 'methods')
    (owner / 'check.py').write_text('print("camera")\n')
    native = a.root / '.agents/skills'
    native.mkdir(parents=True)
    (native / owner.name).symlink_to(owner)

    def worker(*args):
        workspace = args[2]
        candidate = workspace / 'proposed/methods/camera-check/SKILL.md'
        assert candidate.is_file()
        assert (candidate.parent / 'check.py').is_file()
        candidate.write_text(candidate.read_text() + '\nA useful correction.\n')
        return 'completed'

    monkeypatch.setattr(skill_review, '_worker', worker)
    result = a.run()
    assert result['status'] == 'completed', result
    assert [row['path'] for row in result['changes']] == ['methods/camera-check/SKILL.md']
    assert not result['changes'][0]['source_changed']
    assert (native / owner.name).is_symlink()
    assert 'A useful correction' not in (owner / 'SKILL.md').read_text()


def test_project_inside_ignored_parent_repo_still_captures_its_context(apparatus, monkeypatch):
    a = apparatus
    subprocess.run(['git', 'init', str(a.root.parent)], check=True, capture_output=True)
    (a.root.parent / '.gitignore').write_text('project/\n')

    def worker(*args):
        inputs = args[3]
        assert (inputs / 'project/README.md').read_text() == 'Maintained robot project.\n'
        return 'completed'

    monkeypatch.setattr(skill_review, '_worker', worker)
    assert a.run()['status'] == 'completed'


@pytest.mark.parametrize('status', ['failed', 'timed_out'])
def test_failed_work_is_retained_without_auth_or_replay(apparatus, monkeypatch, status):
    a = apparatus
    monkeypatch.setattr(skill_review, '_worker', lambda *args: status)
    result = a.run()
    assert result['status'] == status
    assert Path(result['output'], 'report.json').is_file()
    assert Path(result['input_directory']).is_dir()
    assert not Path(result['input_directory']).parent.joinpath('profile').exists()
    assert not (a.root / '.agents').exists()


def test_cancelled_worker_has_a_final_interrupted_report(apparatus, monkeypatch):
    def interrupt(*args):
        raise KeyboardInterrupt()
    monkeypatch.setattr(skill_review, '_worker', interrupt)
    result = apparatus.run()
    assert result['status'] == 'interrupted'
    assert json.loads(Path(result['output'], 'report.json').read_text())['status'] == 'interrupted'


def test_identical_inputs_skip_model_work_unless_forced(apparatus):
    first = apparatus.run()
    second = apparatus.run()
    assert second['status'] == 'skipped' and second['reused_review'] == first['id']
    assert len(apparatus.calls) == 1
    assert apparatus.run(force=True)['status'] == 'completed'
    assert len(apparatus.calls) == 2


def test_empty_project_skips_without_docker_or_credentials(tmp_path, monkeypatch):
    root, home = tmp_path / 'project', tmp_path / 'home'
    monkeypatch.setattr(skill_review.agents, 'resolve_profile', lambda *a, **k: pytest.fail('do not request credentials'))
    assert main(['--home', str(home), 'skills', '-C', str(root), '--trust-workspace', 'review', '--json']) == 0
    reports = skill_review.reports(root)
    assert len(reports) == 1 and reports[0]['status'] == 'skipped'


def test_pinned_review_uses_the_agents_configured_api_profile(apparatus, monkeypatch):
    a = apparatus
    (a.home / 'config.yaml').write_text('agent: codex\nagents:\n  codex:\n    auth:\n      mode: api\n      key_file: '
                                       + str(a.key) + '\n')
    monkeypatch.setattr(skill_review.agents, 'resolve_profile', lambda *a, **k: pytest.fail('API selection must not use native credentials'))
    assert main(['--home', str(a.home), 'skills', '-C', str(a.root), '--trust-workspace',
                 'review', '--agent', 'codex@0.163.0', '--json']) == 0
    assert len(a.calls) == 1


@pytest.mark.parametrize('location', ['.', 'runtime', 'skill-reviews', 'linked-state'])
def test_review_refuses_private_state_inside_project_before_capture(tmp_path, monkeypatch, location):
    home = tmp_path if location == 'skill-reviews' else tmp_path / 'project' / location
    project = home / 'skill-reviews' if location == 'skill-reviews' else tmp_path / 'project'
    project.mkdir(parents=True)
    if location == 'linked-state':
        home = tmp_path / 'external-home'
        home.mkdir()
        (home / 'skill-reviews').symlink_to(project)
    monkeypatch.setattr(skill_review, 'select', lambda *a, **k: pytest.fail('no record capture'))
    from openrua.errors import UsageError
    with pytest.raises(UsageError, match='outside the robot project'):
        skill_review.run(project, home, agents.get('codex'), {}, image='unused', model='unused',
                         since=None, max_records=1, max_bytes=1000, max_file_bytes=1000,
                         max_files=10, max_events=10, seconds=10)
    assert not (project / '.openrua').exists()


def test_unknown_report_does_not_restart_and_reading_does_not_modify_it(apparatus):
    result = apparatus.run()
    path = Path(result['output']) / 'report.json'
    value = json.loads(path.read_text()); value['status'] = 'running'; reviews.write_json(path, value)
    before = path.read_bytes()
    rows = skill_review.reports(apparatus.root)
    assert rows[0]['status'] == 'unknown' and path.read_bytes() == before
    assert len(apparatus.calls) == 1


def test_symlink_and_wrong_directory_outputs_are_reported_not_read(apparatus, monkeypatch):
    a = apparatus
    outside = a.root.parent / 'do-not-read'; outside.mkdir()
    (outside / 'SKILL.md').write_text('SECRET OUTSIDE THE REVIEW')
    def worker(*args):
        workspace = args[2]
        target = workspace / 'proposed/.agents/skills/leak'
        target.parent.mkdir(parents=True, exist_ok=True)
        target.symlink_to(outside)
        write_skill(workspace / '.agents/skills', 'wrong-directory')
        return 'completed'
    monkeypatch.setattr(skill_review, '_worker', worker)
    result = a.run()
    assert result['status'] == 'invalid_output'
    assert any('outside proposed' in error for error in result['validation_errors'])
    assert not list(Path(result['output'], 'proposed').rglob('SKILL.md'))


def test_known_auth_values_are_scrubbed_and_input_hashes_describe_stored_bytes(apparatus, monkeypatch):
    a = apparatus
    secret = a.key.read_text()
    original = write_skill(a.root / '.agents/skills') / 'SKILL.md'
    original.write_text(original.read_text() + '\n' + secret + '\n')
    def worker(*args):
        workspace, inputs = args[2:4]
        path = workspace / 'proposed/.agents/skills/camera-check/SKILL.md'
        assert secret not in path.read_text()
        path.write_text(path.read_text() + '\nImprove the method.\n')
        (workspace / 'summary.md').write_text(secret)
        return 'completed'
    monkeypatch.setattr(skill_review, '_worker', worker)
    result = a.run()
    assert result['status'] == 'completed' and result['changes'][0]['source_changed'] is False
    assert secret in original.read_text()
    for path in Path(result['output']).rglob('*'):
        if path.is_file(): assert secret.encode() not in path.read_bytes()
    manifest = json.loads(Path(result['output'], 'manifest.json').read_text())
    for item in manifest['files']:
        assert reviews.digest(Path(result['input_directory'], item['path']).read_bytes()) == item['sha256']


def test_bounded_conversation_export_keeps_recent_events_and_reports_truncation(tmp_path):
    path = conversation(tmp_path, 'record', tmp_path / 'project')
    store = SQLiteStore.open_readonly(path)
    try:
        result = store.export(max_bytes=100000, max_events=2)
        assert result['truncated'] and [event['text'] for event in result['events']] == ['result 3', 'result 4']
        assert len(json.dumps(result, ensure_ascii=False, separators=(',', ':')).encode()) <= 100000
        with pytest.raises(ValueError, match='input limit'):
            store.export(max_bytes=1, max_events=2)
    finally:
        store.close()
