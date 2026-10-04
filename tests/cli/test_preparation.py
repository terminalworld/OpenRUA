"""Preparation builds the selected dependencies and never starts a failed setup."""
import io
from types import SimpleNamespace

import pytest

from openrua.config import compose
from openrua.cli import preparation
from openrua.errors import UnavailableError


def values():
    return dict(robot='panda-omron', sim='robosuite', bench='robocasa365',
                agent='codex', model='', name='test')


def test_build_plan_uses_benchmark_image_and_its_ros_distro(tmp_path):
    cfg = compose(None, None, 'robocasa365', tmp_path, agent='codex')
    plan = preparation.image_commands(cfg)
    assert plan[0] == ('openrua-sim-robocasa365', ['build', '--bench', 'robocasa365'])
    assert plan[1][0] == 'openrua-sandbox-humble'
    assert plan[1][1][-2:] == ['--agent', 'codex']
    assert 'humble' in plan[1][1]


def test_prepare_builds_missing_only_and_retry_reuses_completed_images(tmp_path, monkeypatch):
    monkeypatch.setattr(preparation.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=0, stderr=''))
    have, calls, fail = set(), [], [True]
    monkeypatch.setattr(preparation, 'image_exists', lambda image: image in have)
    def build(argv, **kwargs):
        calls.append(argv)
        if '--bench' in argv:
            image = 'openrua-sim-robocasa365'
        elif 'sandbox' in argv:
            image = 'openrua-sandbox-humble'
        else:
            image = 'openrua-proxy'
        code = 1 if image.endswith('sandbox-humble') and fail[0] else 0
        if not code: have.add(image)
        return SimpleNamespace(stdout=io.StringIO('Docker build output\n'), wait=lambda: code)
    monkeypatch.setattr(preparation.subprocess, 'Popen', build)
    progress = []
    with pytest.raises(UnavailableError, match='Log:'):
        preparation.prepare(tmp_path, values(), progress.append)
    assert len(calls) == 2 and have == {'openrua-sim-robocasa365'}
    assert any('Docker build output' in p.read_text() for p in (tmp_path / 'preparation').glob('*.log'))
    fail[0] = False
    preparation.prepare(tmp_path, values(), progress.append)
    assert len(calls) == 4 and len(have) == 3
    preparation.prepare(tmp_path, values(), progress.append)
    assert len(calls) == 4


def test_no_build_when_docker_cannot_be_reached(tmp_path, monkeypatch):
    monkeypatch.setattr(preparation.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=1, stderr='permission denied'))
    monkeypatch.setattr(preparation.subprocess, 'Popen', lambda *a, **k: pytest.fail('must not build'))
    with pytest.raises(UnavailableError, match='permission denied'):
        preparation.prepare(tmp_path, values())
