"""Invalid recording requests fail before the runner acquires resources."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from openrua.cli import build_parser
from openrua.errors import UsageError
from openrua.runner import main


@pytest.mark.parametrize('backend,record,wait,match', [
    ('real', (), 0, 'positive'), ('real', (), -1, 'positive'),
    ('real', (), float('inf'), 'positive'), ('real', (), float('nan'), 'positive'),
    ('sim', (), 2, 'real robot'), ('real', None, 2, 'requires camera recording'),
    ('real', (), 2, 'at least one camera'),
])
def test_invalid_checks_do_not_create_run_directories(tmp_path, monkeypatch, backend, record, wait, match):
    monkeypatch.setattr(main, 'not_ready', lambda _: [])
    cfg = {'machine': {'backend': {'kind': backend}, 'cameras': {'list': []}}}
    args = SimpleNamespace(record=record, record_wait=wait, task='inspect')
    with pytest.raises(UsageError, match=match):
        main.run_set(deepcopy(cfg), tmp_path / 'config.yaml', args, tmp_path / 'home', 'fixture')
    assert list(tmp_path.iterdir()) == []


def test_bench_exposes_the_same_recording_controls():
    args = build_parser().parse_args(['bench', '--config', 'capbench', '--run-id', 'test',
                                     '--task-suite', 'lift', '--record-wait', '5',
                                     '--record-fps', '10', '--no-video'])
    assert args.record_wait == 5 and args.no_video is True
    assert args.record_fps == 10


@pytest.mark.parametrize('backend,record,fps,match', [
    ('real', (), 0, 'positive'), ('real', (), -1, 'positive'),
    ('real', (), float('inf'), 'positive'), ('real', (), float('nan'), 'positive'),
    ('sim', (), 10, 'real robot'), ('real', None, 10, 'requires camera recording'),
    ('real', (), 10, 'at least one camera'),
])
def test_invalid_rates_do_not_create_run_directories(tmp_path, monkeypatch, backend, record, fps, match):
    monkeypatch.setattr(main, 'not_ready', lambda _: [])
    cfg = {'machine': {'backend': {'kind': backend}, 'cameras': {'list': []}}}
    args = SimpleNamespace(record=record, record_fps=fps, task='inspect')
    with pytest.raises(UsageError, match=match):
        main.run_set(deepcopy(cfg), tmp_path / 'config.yaml', args, tmp_path / 'home', 'fixture')
    assert list(tmp_path.iterdir()) == []
