"""An optional startup check gates the task, with cleanup and saved diagnostics."""

import json

import pytest

from openrua.runner import trial
from tests.runner.test_trial_real import _harness
from tests.runner.test_trial_recording import Tap


def _camera(cfg):
    cfg['machine']['cameras'] = {
        'types': {'camera': {'model': 'fixture', 'topics': {'color': '/{name}/image'}}},
        'list': [{'name': 'front', 'type': 'camera'}],
    }


def test_recording_check_is_saved_before_the_operator_starts(monkeypatch, tmp_path, proxy_address):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    _camera(cfg)
    run = tmp_path / 'run'
    result = run / 'trials/suite-0/seed0/result.json'
    calls = []
    tap = Tap(calls, result)

    def wait(seconds):
        saved = json.loads(result.read_text())
        assert saved['recording_check'] == {'status': 'pending', 'timeout_s': 4.5, 'elapsed_seconds': 0}
        assert 'operator_meta' not in saved
        assert seconds == 4.5
        calls.append('first frame saved')

    def operator(ctx):
        saved = json.loads(result.read_text())
        assert saved['recording_check']['status'] == 'passed'
        assert saved['recording_check']['elapsed_seconds'] >= 0
        calls.append('operator')
        return {'termination': 'self_finished'}

    tap.wait_for_frames = wait
    monkeypatch.setattr(trial, 'record_cameras_of', lambda *a: tap)
    rec = trial.run_trial(cfg, tmp_path / 'config.yaml', run, 'suite', 0, 0,
                          operator, 60, token_file=str(token), task='inspect',
                          home=tmp_path / 'home', record_cameras=(), record_wait=4.5)
    assert calls == ['first frame saved', 'operator', 'recording stopped']
    assert rec['recording_check']['status'] == 'passed'
    assert rec['termination'] == 'self_finished'
    assert handle.calls == ['shutdown']
    provenance = json.loads((result.parent / 'provenance.json').read_text())
    assert provenance['record_wait'] == 4.5


@pytest.mark.parametrize('error,status,termination', [
    (TimeoutError('No saved frame from front'), 'failed', 'anomaly'),
    (RuntimeError('Camera process exited'), 'failed', 'anomaly'),
    (KeyboardInterrupt(), 'canceled', 'stopped'),
])
def test_recording_check_failure_keeps_diagnostics_and_cleans_resources(
        monkeypatch, tmp_path, proxy_address, error, status, termination):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    _camera(cfg)
    run = tmp_path / 'run'
    result = run / 'trials/suite-0/seed0/result.json'
    calls = []
    tap = Tap(calls, result, missing=True)

    def wait(seconds):
        raise error

    tap.wait_for_frames = wait
    monkeypatch.setattr(trial, 'record_cameras_of', lambda *a: tap)
    monkeypatch.setattr(trial, 'halt', lambda *a, **kw: calls.append('halt'))
    monkeypatch.setattr(trial, 'sandbox_down', lambda *a: calls.append('sandbox stopped'))
    rec = trial.run_trial(cfg, tmp_path / 'config.yaml', run, 'suite', 0, 0,
                          lambda _: pytest.fail('task must not start'), 60,
                          token_file=str(token), task='inspect', home=tmp_path / 'home',
                          record_cameras=(), record_wait=1)
    saved = json.loads(result.read_text())
    assert saved == json.loads(json.dumps(rec))
    assert saved['recording_check']['status'] == status
    assert saved['recording']['missing_cameras'] == ['front']
    assert saved['termination'] == termination
    assert saved['success'] is None
    assert 'operator_meta' not in saved
    assert saved['cleanup']['status'] == 'completed'
    assert calls[-2:] == ['recording stopped', 'sandbox stopped']
    assert handle.calls == ['shutdown']
    if status == 'failed':
        assert str(error) in saved['anomaly']


@pytest.mark.parametrize('timeout,backend,record_cameras,has_camera,match', [
    (0, 'real', (), True, 'positive'),
    (float('inf'), 'real', (), True, 'positive'),
    (float('nan'), 'real', (), True, 'positive'),
    (True, 'real', (), True, 'positive'),
    (1, 'sim', (), True, 'real robot'),
    (1, 'real', None, True, 'recording enabled'),
    (1, 'real', (), False, 'no camera'),
    (1, 'real', ('missing',), True, 'missing'),
])
def test_invalid_recording_check_is_rejected_before_resource_preparation(
        monkeypatch, tmp_path, timeout, backend, record_cameras, has_camera, match):
    _, cfg, token = _harness(monkeypatch, tmp_path)
    cfg['machine']['backend']['kind'] = backend
    if has_camera:
        _camera(cfg)
    monkeypatch.setattr(trial, 'ensure_internal_network', lambda: pytest.fail('no startup'))
    with pytest.raises(ValueError, match=match):
        trial.run_trial(cfg, tmp_path / 'config.yaml', tmp_path / 'run', 'suite', 0, 0,
                        lambda _: pytest.fail('no task'), 60, token_file=str(token),
                        record_cameras=record_cameras, record_wait=timeout)
    assert not (tmp_path / 'run').exists()
