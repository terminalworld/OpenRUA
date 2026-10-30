"""The requested camera rate reaches capture and is saved before the task."""

import json

import pytest

from openrua.cli import build_parser
from openrua.runner import main, trial
from tests.runner.test_recording_readiness import _camera
from tests.runner.test_trial_real import _harness
from tests.runner.test_trial_recording import Tap


@pytest.mark.parametrize('requested,effective', [(None, 2.0), (10.0, 10.0), (12.5, 12.5)])
def test_camera_rate_reaches_capture_and_is_saved_before_execution(
        monkeypatch, tmp_path, proxy_address, requested, effective):
    _, cfg, token = _harness(monkeypatch, tmp_path)
    _camera(cfg)
    run = tmp_path / 'run'
    result = run / 'trials/suite-0/seed0/result.json'
    calls = []

    class Recorder(Tap):
        def __init__(self, sandbox, cameras, fps, record):
            super().__init__(calls, result)
            assert cameras == {'front': '/front/image'}
            assert record == result.parent / 'frames'
            self.fps = fps

        def start(self):
            calls.append(('capture started', self.fps))

    def operator(ctx):
        assert calls == [('capture started', effective)]
        assert json.loads((result.parent / 'provenance.json').read_text())['record_fps'] == effective
        assert json.loads((run / 'config.json').read_text())['record_fps'] == effective
        calls.append('agent')
        return {'termination': 'self_finished'}

    monkeypatch.setattr(trial, 'CameraTap', Recorder)
    rec = trial.run_trial(cfg, tmp_path / 'config.yaml', run, 'suite', 0, 0,
                          operator, 60, token_file=str(token), task='inspect',
                          home=tmp_path / 'home', record_cameras=(), record_fps=requested,
                          provenance={'record_fps': 99})
    assert rec['termination'] == 'self_finished'
    assert rec['record_fps'] == effective
    assert json.loads(result.read_text())['record_fps'] == effective
    assert calls == [('capture started', effective), 'agent', 'recording stopped']


def test_run_set_forwards_requested_rate_to_trial(monkeypatch, tmp_path):
    _, cfg, _ = _harness(monkeypatch, tmp_path)
    _camera(cfg)
    args = build_parser().parse_args([
        'bench', '--config', 'fixture', '--run-id', 'rate', '--task-suite', 'suite',
        '--task', 'inspect', '--record', '--record-fps', '10',
        '--wall-clock-min', '60', '--runs-root', str(tmp_path / 'runs'),
    ])
    args.record, args.task_ids, args.seeds = (), [0], [0]
    seen = []

    def run_trial(*a, **kw):
        seen.append(kw['record_fps'])
        return {'success': None, 'termination': 'self_finished', 'wall_seconds': 0}

    monkeypatch.setattr(main, 'not_ready', lambda _: [])
    monkeypatch.setattr(main, 'run_trial', run_trial)
    monkeypatch.setattr(main.record, 'provenance', lambda *a, **kw: {})
    monkeypatch.setattr(main.record, 'write_run_summary', lambda *a: None)
    assert main.run_set(cfg, tmp_path / 'config.yaml', args, tmp_path / 'home', 'fixture') == 0
    assert seen == [10]


@pytest.mark.parametrize('fps,backend,record_cameras,has_camera,match', [
    (0, 'real', (), True, 'positive'),
    (-1, 'real', (), True, 'positive'),
    (float('inf'), 'real', (), True, 'positive'),
    (float('nan'), 'real', (), True, 'positive'),
    (True, 'real', (), True, 'positive'),
    (10, 'sim', (), True, 'real robot'),
    (10, 'real', None, True, 'recording enabled'),
    (10, 'real', (), False, 'no camera'),
    (10, 'real', ('missing',), True, 'missing'),
])
def test_invalid_rate_is_rejected_before_resource_preparation(
        monkeypatch, tmp_path, fps, backend, record_cameras, has_camera, match):
    _, cfg, token = _harness(monkeypatch, tmp_path)
    cfg['machine']['backend']['kind'] = backend
    if has_camera:
        _camera(cfg)
    monkeypatch.setattr(trial, 'ensure_internal_network', lambda: pytest.fail('no startup'))
    with pytest.raises(ValueError, match=match):
        trial.run_trial(cfg, tmp_path / 'config.yaml', tmp_path / 'run', 'suite', 0, 0,
                        lambda _: pytest.fail('no task'), 60, token_file=str(token),
                        record_cameras=record_cameras, record_fps=fps)
    assert not (tmp_path / 'run').exists()
