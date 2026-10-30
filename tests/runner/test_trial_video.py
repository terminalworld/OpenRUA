"""Video export retains the trial claim after robot resources have stopped."""

import json

import pytest

from openrua.runner import lock, trial, video
from tests.runner.test_trial_real import _harness
from tests.runner.test_trial_recording import Tap


@pytest.mark.parametrize('outcome', ['completed', 'failed', 'canceled'])
def test_video_export_runs_after_cleanup_under_the_original_claim(
        monkeypatch, tmp_path, proxy_address, capsys, outcome):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    run = tmp_path / 'run'
    directory = run / 'trials/suite-0/seed0'
    result = directory / 'result.json'
    calls = []
    elapsed = []
    secret = token.read_text().strip().split('=', 1)[1]
    write_result = trial.record.write_result

    def checked_write(path, value):
        assert secret not in json.dumps(value)
        write_result(path, value)

    monkeypatch.setattr(trial.record, 'write_result', checked_write)
    tap = Tap(calls, result)
    monkeypatch.setattr(trial, 'record_cameras_of', lambda *a: tap)
    monkeypatch.setattr(trial, 'sandbox_down', lambda *a: calls.append('sandbox stopped'))

    def encode(frames, output):
        assert calls == ['recording stopped', 'sandbox stopped']
        assert handle.calls == ['shutdown']
        assert lock.holder(directory) is not None
        assert lock.acquire(directory, 'concurrent-fixture') is None
        saved = json.loads(result.read_text())
        assert saved['success'] is True
        assert saved['termination'] == 'self_finished'
        assert saved['cleanup']['status'] == 'completed'
        assert saved['video_export']['status'] == 'exporting'
        elapsed.append(saved['wall_seconds'])
        if outcome == 'failed':
            raise RuntimeError(f'encoder failed reading {secret}')
        if outcome == 'canceled':
            raise KeyboardInterrupt
        return {'status': 'completed', 'cameras': {}, 'errors': []}

    monkeypatch.setattr(video, 'export_camera_videos', encode)
    rec = trial.run_trial(cfg, tmp_path / 'config.yaml', run, 'suite', 0, 0,
                          lambda _: {'termination': 'self_finished'}, 60,
                          token_file=str(token), task='inspect', home=tmp_path / 'home',
                          record_cameras=(), export_video=True, judge=lambda _: True)
    assert rec['success'] is True and rec['termination'] == 'self_finished'
    assert rec['anomaly'] is None
    assert rec['video_export']['status'] == outcome
    assert rec.get('interrupted', False) is (outcome == 'canceled')
    assert json.loads(result.read_text()) == json.loads(json.dumps(rec))
    assert rec['wall_seconds'] == elapsed[0]
    assert lock.holder(directory) is None
    assert secret not in capsys.readouterr().out
    if outcome == 'failed':
        assert rec['video_export']['errors'] == ['encoder failed reading [REDACTED]']


def test_video_export_waits_for_confirmed_resource_cleanup(monkeypatch, tmp_path, proxy_address):
    _, cfg, token = _harness(monkeypatch, tmp_path)
    run = tmp_path / 'run'
    directory = run / 'trials/suite-0/seed0'
    result = directory / 'result.json'
    tap = Tap([], result)
    monkeypatch.setattr(trial, 'record_cameras_of', lambda *a: tap)

    def stop(*a):
        raise RuntimeError('resource still running')

    monkeypatch.setattr(trial, 'sandbox_down', stop)
    monkeypatch.setattr(video, 'export_camera_videos', lambda *a: pytest.fail('cleanup incomplete'))
    rec = trial.run_trial(cfg, tmp_path / 'config.yaml', run, 'suite', 0, 0,
                          lambda _: {'termination': 'self_finished'}, 60,
                          token_file=str(token), task='inspect', home=tmp_path / 'home',
                          record_cameras=(), export_video=True)
    assert rec['video_export']['status'] == 'skipped'
    assert rec['cleanup']['status'] == 'failed'
    assert rec['termination'] == 'self_finished'
    assert lock.holder(directory) is None
