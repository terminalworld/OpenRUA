from copy import deepcopy
from datetime import datetime, timedelta, timezone
from pathlib import Path
import time

import pytest

from openrua.cli import skill_schedules as plans, skill_scheduler as runtime
from openrua.config import projects


@pytest.fixture
def setup(tmp_path):
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir()
    projects.trust(root, home)
    spec = dict(name='Review camera methods',
                calendar=dict(time='21:30', days='mon,tue,wed,thu,fri', timezone='Asia/Shanghai'),
                options=dict(agent='codex', model='gpt-5.6-sol', effort='high',
                             image='prepared-image', wall_clock_min=1))
    return root, home, spec


def create(setup, enabled=True):
    root, home, spec = setup
    return plans.create(home, root, 'a' * 32, spec, enabled=enabled)


def due(home, row):
    row['next_run'] = '2000-01-03T13:30:00+00:00'
    plans.write(home, row)
    return row


def test_calendar_uses_local_zone_and_weekdays(setup):
    _, _, spec = setup
    at = datetime(2026, 10, 23, 15, 0, tzinfo=timezone.utc)
    # Friday 23:00 in Shanghai: the next selected day is Monday.
    assert plans.following(spec['calendar'], at) == '2026-10-26T13:30:00+00:00'
    for calendar in ({**spec['calendar'], 'timezone': 'missing-zone'},
                     {**spec['calendar'], 'time': '24:01'}, {**spec['calendar'], 'days': '0'}):
        with pytest.raises(ValueError): plans.trigger(calendar)


def test_background_plan_resolves_relative_manifest_in_project_and_preserves_pin(setup):
    root, home, spec = setup
    spec['options']['agent'] = 'adapters/custom.yaml@1.2.3'
    row = create(setup, False)
    assert row['options']['agent'] == str(root / 'adapters/custom.yaml') + '@1.2.3'


def test_cli_edit_agent_resets_model_defaults_and_retains_explicit_effort(setup):
    from openrua import agents
    from openrua.cli import build_parser
    root, home, _ = setup
    row = create(setup, False)
    args = build_parser().parse_args(['--home', str(home), 'skills', '-C', str(root),
        'schedule', 'edit', row['id'], '--agent', 'claude-code', '--effort', 'medium', '--json'])
    assert args.fn(args) == 0
    current = plans.read(home, root, row['id'])
    assert current['options']['model'] == agents.get('claude-code').default_model
    assert current['options']['effort'] == 'medium' and not current['enabled']


def test_plan_creation_is_idempotent_scoped_and_requires_trust(setup, tmp_path):
    root, home, spec = setup
    row = create(setup, False)
    assert not row['enabled']
    assert plans.create(home, root, row['id'], spec) == row
    assert plans.listing(home, root) == [row]
    other = tmp_path / 'other'; other.mkdir()
    with pytest.raises(ValueError, match='another project'): plans.read(home, other, row['id'])
    with pytest.raises(ValueError, match='trust'): plans.create(home, other, 'b' * 32, spec)
    with pytest.raises(ValueError, match='outside'): plans.create(root / 'private', root, 'b' * 32, spec)
    with pytest.raises(ValueError): plans.load(home, '../outside')


def test_many_missed_times_are_one_claim_and_next_time_is_future(setup):
    _, home, _ = setup
    row = due(home, create(setup))
    claimed = plans.claim(home, row['id'], row['revision'], row['next_run'])
    assert claimed['active_task']
    assert datetime.fromisoformat(claimed['next_run']) > plans.now()
    assert plans.claim(home, row['id'], row['revision'], row['next_run']) is None
    plans.finish(home, row['id'], dict(id=claimed['active_task'], status='completed'))
    assert plans.claim(home, row['id'], row['revision'], row['next_run']) is None


@pytest.mark.parametrize('status,enabled', [('failed', True), ('timed_out', True), ('completed', True),
    ('quota_limited', False), ('authentication_required', False), ('unknown', False)])
def test_failure_policy_uses_canonical_worker_result(setup, monkeypatch, status, enabled):
    root, home, _ = setup
    row = due(home, create(setup))
    started = []
    monkeypatch.setattr(runtime.skill_tasks, 'start', lambda *args: started.append(args))
    monkeypatch.setattr(runtime.skill_tasks, 'read', lambda h, p, key: dict(id=key, status=status, error='native evidence'))
    runtime.execute(home, row['id'], row['revision'], row['next_run'], None)
    result = plans.read(home, root, row['id'])
    assert result['enabled'] == enabled and result['active_task'] is None
    assert result['last_result']['status'] == status
    # A stale delivery of this occurrence cannot cause another model task.
    runtime.execute(home, row['id'], row['revision'], row['next_run'], None)
    assert len(started) == 1


def test_resume_starts_from_future_and_newer_edit_survives_old_auth_failure(setup):
    root, home, spec = setup
    row = due(home, create(setup))
    claimed = plans.claim(home, row['id'], row['revision'], row['next_run'])
    changed = deepcopy(spec); changed['name'] = 'Edited review'
    plans.edit(home, root, row['id'], changed, row['revision'])
    plans.finish(home, row['id'], dict(id=claimed['active_task'], status='authentication_required'))
    assert plans.read(home, root, row['id'])['enabled']
    with pytest.raises(ValueError, match='changed'): plans.edit(home, root, row['id'], spec, row['revision'])
    plans.action(home, root, row['id'], 'pause')
    resumed = plans.action(home, root, row['id'], 'resume')
    assert resumed['enabled'] and datetime.fromisoformat(resumed['next_run']) > plans.now()


def test_removed_plan_preserves_current_task_and_last_report(setup):
    root, home, _ = setup
    row = due(home, create(setup))
    claimed = plans.claim(home, row['id'], row['revision'], row['next_run'])
    plans.action(home, root, row['id'], 'remove')
    assert not plans.listing(home, root)
    assert plans.load(home, row['id'])['active_task'] == claimed['active_task']
    plans.finish(home, row['id'], dict(id=claimed['active_task'], status='completed', review_id='report'))
    assert plans.load(home, row['id'])['last_result']['review_id'] == 'report'
    with pytest.raises(ValueError, match='removed'): plans.action(home, root, row['id'], 'resume')


def test_restart_adopts_existing_task_without_starting_another(setup, monkeypatch):
    root, home, _ = setup
    row = due(home, create(setup))
    claimed = plans.claim(home, row['id'], row['revision'], row['next_run'])
    def forbidden(*args): raise AssertionError('must not launch an adopted task')
    monkeypatch.setattr(runtime.skill_tasks, 'start', forbidden)
    monkeypatch.setattr(runtime.skill_tasks, 'read', lambda *args: dict(id=claimed['active_task'], status='completed'))
    runtime.execute(home, row['id'], row['revision'], row['next_run'], claimed['active_task'])
    assert plans.read(home, root, row['id'])['last_result']['status'] == 'completed'


def test_scheduler_shutdown_does_not_start_a_queued_occurrence(setup, monkeypatch):
    root, home, _ = setup
    row = due(home, create(setup))
    def forbidden(*args): raise AssertionError('shutdown must not spend on a new review')
    monkeypatch.setattr(runtime.skill_tasks, 'start', forbidden)
    runtime.execute(home, row['id'], row['revision'], row['next_run'], None, stopped=lambda: True)
    assert plans.read(home, root, row['id'])['next_run'] == row['next_run']


def test_api_schedule_actions_are_project_scoped_and_explicit(setup, monkeypatch, tmp_path):
    from openrua.cli.project_skills import ProjectSkills
    from openrua.cli import build_parser
    root, home, spec = setup
    starts = []
    monkeypatch.setattr(runtime, 'ensure', lambda *a, **k: starts.append(a))
    api = ProjectSkills(home, lambda: root)
    assert api.dispatch({'operation': 'schedules'})['schedules'] == []
    assert starts == []
    body = dict(operation='schedule-create', request_id='b' * 32, definition=spec)
    row = api.dispatch(body)
    assert row['enabled'] and api.dispatch(body)['id'] == row['id']
    assert len(plans.listing(home, root)) == 1
    other = tmp_path / 'other'; other.mkdir(); projects.trust(other, home)
    other_api = ProjectSkills(home, lambda: other)
    with pytest.raises(ValueError, match='another project'):
        other_api.dispatch(dict(operation='schedule-action', id=row['id'], action='pause'))
    with pytest.raises(ValueError): api.dispatch(dict(operation='scheduler', action='stop'))
    result = api.dispatch(dict(operation='schedule-action', id=row['id'], action='pause'))
    assert not result['enabled']
    args = build_parser().parse_args(['--home', str(home), 'skills', '-C', str(root), 'schedule', 'show', row['id'], '--json'])
    assert args.fn(args) == 0


def test_actual_detached_scheduler_survives_client_and_coalesces_empty_project(setup):
    root, home, _ = setup
    row = due(home, create(setup))
    def wait(predicate):
        end = time.monotonic() + 20
        while time.monotonic() < end:
            value = predicate()
            if value: return value
            time.sleep(.05)
        raise AssertionError((home / 'skill-scheduler/service.log').read_text())
    try:
        first = runtime.ensure(home, explicit=True)
        assert first['running']
        # A new caller joins the same owner, independent of its creating object.
        wait(lambda: runtime.status(home).get('instance'))
        again = runtime.ensure(home, explicit=True)
        assert again['pid'] == first['pid']
        def finished():
            value = plans.read(home, root, row['id'])
            return value if value.get('last_result') else None
        result = wait(finished)
        assert result['last_result']['status'] == 'skipped'
        assert datetime.fromisoformat(result['next_run']) > plans.now()
        assert len(runtime.skill_tasks.listing(home, root)) == 1
        runtime.stop(home)
        wait(lambda: not runtime.status(home)['running'])
        runtime.autostart(home)
        assert not runtime.status(home)['running'], 'normal startup respects a deliberate stop'
        restarted = runtime.ensure(home, explicit=True)
        assert restarted['running']
        assert len(runtime.skill_tasks.listing(home, root)) == 1, 'completed work must not be replayed'
        plans.action(home, root, row['id'], 'pause')
        wait(lambda: not runtime.status(home)['running'])
    finally:
        runtime.stop(home)
        wait(lambda: not runtime.status(home)['running'])


def test_normal_start_restores_after_owner_exit_without_system_service(setup):
    import subprocess
    import sys
    root, home, _ = setup
    create(setup)
    log = home / 'controlled-owner.log'
    process = None
    def wait(predicate):
        end = time.monotonic() + 10
        while time.monotonic() < end:
            result = predicate()
            if result: return result
            time.sleep(.05)
        raise AssertionError(log.read_text())
    try:
        with log.open('w') as stream:
            process = subprocess.Popen([sys.executable, '-m', 'openrua.cli.skill_scheduler', '--home', str(home)],
                                       stdout=stream, stderr=stream)
        original = wait(lambda: runtime.status(home).get('instance'))
        # Terminate only the process handle this test created, as on shutdown.
        process.terminate(); process.wait(timeout=5)
        assert not runtime.status(home)['running']
        assert not (home / 'skill-scheduler/stop').exists()
        runtime.autostart(home)
        assert wait(lambda: runtime.status(home).get('instance') != original)
        assert runtime.status(home)['running']
    finally:
        if process is not None and process.poll() is None:
            process.terminate(); process.wait(timeout=5)
        runtime.stop(home)
        wait(lambda: not runtime.status(home)['running'])
