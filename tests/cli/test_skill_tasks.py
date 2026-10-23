"""Manual reviews have durable request identity and independent process ownership."""

import json
from pathlib import Path
import threading
import time
from uuid import uuid4

import pytest

from openrua import reviews
from openrua.cli import skill_tasks
from openrua.config import projects


OPTIONS = dict(agent='codex', model='fixture-model', image='no-image-needed', wall_clock_min=1)


def test_real_child_review_survives_client_recreation_and_skips_empty_project(tmp_path, monkeypatch):
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir()
    projects.trust(root, home)
    identity, calls = uuid4().hex, []
    original = skill_tasks.subprocess.Popen
    def spawn(*args, **kwargs):
        calls.append(kwargs)
        return original(*args, **kwargs)
    monkeypatch.setattr(skill_tasks.subprocess, 'Popen', spawn)
    first = skill_tasks.start(home, root, identity, OPTIONS)
    assert first['id'] == identity
    assert skill_tasks.start(home, root, identity, OPTIONS)['id'] == identity
    deadline = time.monotonic() + 10
    while True:
        result = skill_tasks.listing(home, root)[0]
        if result['status'] not in {'running', 'starting'}: break
        assert time.monotonic() < deadline, result
        time.sleep(.02)
    assert result['status'] == 'skipped', (result, (skill_tasks.location(home, identity) / 'process.log').read_text())
    assert len(calls) == 1 and calls[0]['start_new_session'] is True and calls[0]['cwd'] == root
    assert result['review_id']
    assert skill_tasks.run(home, root, identity) == 0
    assert len(list((root / '.openrua/skill-reviews').glob('*/report.json'))) == 1
    with pytest.raises(ValueError): skill_tasks.read(home, tmp_path, identity)
    with pytest.raises(ValueError): skill_tasks.start(home, root, identity, {**OPTIONS, 'model': 'other'})


def test_two_clients_cannot_run_models_simultaneously_for_one_project(tmp_path, monkeypatch):
    from openrua.cli.commands import skill_review
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir()
    projects.trust(root, home)
    monkeypatch.setattr(skill_tasks.subprocess, 'Popen', lambda *args, **kwargs: None)
    first, second = uuid4().hex, uuid4().hex
    skill_tasks.start(home, root, first, OPTIONS)
    skill_tasks.start(home, root, second, OPTIONS)
    entered, release, calls = threading.Event(), threading.Event(), []
    def review(args, on_result=None, cancelled=None):
        calls.append(args.workspace)
        entered.set()
        assert release.wait(5)
        on_result(dict(status='completed', id='a' * 32))
        return 0
    monkeypatch.setattr(skill_review, 'run', review)
    thread = threading.Thread(target=skill_tasks.run, args=(home, root, first))
    thread.start()
    try:
        assert entered.wait(5)
        assert skill_tasks.run(home, root, second) == 1
        assert skill_tasks.read(home, root, second)['status'] == 'busy'
    finally:
        release.set()
        thread.join(5)
    assert len(calls) == 1 and not thread.is_alive()


def test_interrupted_task_is_inspectable_and_never_replayed(tmp_path, monkeypatch):
    from openrua.cli.commands import skill_review
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir()
    monkeypatch.setattr(skill_tasks.subprocess, 'Popen', lambda *a, **k: None)
    identity = uuid4().hex
    skill_tasks.start(home, root, identity, OPTIONS)
    directory = skill_tasks.location(home, identity)
    (directory / 'owner.lock').touch()
    reviews.write_json(directory / 'status.json', dict(status='running'))
    monkeypatch.setattr(skill_review, 'run', lambda *a, **k: pytest.fail('do not replay a persisted task'))
    assert skill_tasks.read(home, root, identity)['status'] == 'unknown'
    assert skill_tasks.run(home, root, identity) == 1
    with pytest.raises(ValueError): skill_tasks.location(home, '../escape')


def test_cancel_before_worker_start_never_calls_the_model(tmp_path, monkeypatch):
    from openrua.cli.commands import skill_review
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir()
    monkeypatch.setattr(skill_tasks.subprocess, 'Popen', lambda *a, **k: None)
    identity = uuid4().hex
    skill_tasks.start(home, root, identity, OPTIONS)
    assert skill_tasks.cancel(home, root, identity)['status'] == 'cancel_requested'
    monkeypatch.setattr(skill_review, 'run', lambda *a, **k: pytest.fail('a canceled task must not start a model'))
    assert skill_tasks.run(home, root, identity) == 130
    assert skill_tasks.read(home, root, identity)['status'] == 'interrupted'


def test_owned_child_observes_cancel_without_signaling_other_processes(tmp_path, monkeypatch):
    import subprocess
    import sys
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir()
    original = subprocess.Popen
    monkeypatch.setattr(skill_tasks.subprocess, 'Popen', lambda *a, **k: None)
    identity = uuid4().hex
    skill_tasks.start(home, root, identity, OPTIONS)
    directory = skill_tasks.location(home, identity)
    program = '''
import fcntl, sys, time
from pathlib import Path
from openrua import reviews
root = Path(sys.argv[1])
with (root / 'owner.lock').open('a') as held:
    fcntl.flock(held, fcntl.LOCK_EX)
    reviews.write_json(root / 'status.json', dict(status='running'))
    deadline = time.monotonic() + 10
    while not (root / 'cancel').exists():
        assert time.monotonic() < deadline
        time.sleep(.01)
    reviews.write_json(root / 'status.json', dict(status='interrupted'))
'''
    child = original([sys.executable, '-c', program, str(directory)])
    try:
        deadline = time.monotonic() + 5
        while skill_tasks.read(home, root, identity)['status'] != 'running':
            assert time.monotonic() < deadline
            time.sleep(.01)
        assert skill_tasks.cancel(home, root, identity)['status'] == 'cancel_requested'
        assert child.wait(timeout=5) == 0
        assert skill_tasks.read(home, root, identity)['status'] == 'interrupted'
    finally:
        if child.poll() is None: child.kill()
        child.wait()
