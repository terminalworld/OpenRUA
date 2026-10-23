"""A successful CLI exit does not hide native quota and error results."""
from contextlib import nullcontext
import json
from types import SimpleNamespace

import pytest

from openrua import agents
from openrua.runner import skill_review


@pytest.mark.parametrize('frame,status', [
    ({'type': 'error', 'message': 'usage limit reached'}, 'quota_limited'),
    ({'type': 'turn.failed', 'error': {'message': 'request failed'}}, 'failed'),
    ({'type': 'turn.completed', 'usage': {'input_tokens': 2, 'output_tokens': 1}}, 'completed'),
])
def test_native_result_classifies_zero_exit_and_always_cleans_up(tmp_path, monkeypatch, frame, status):
    adapter = agents.get('codex')
    monkeypatch.setattr(adapter, 'launch_argv', lambda *a, **k: ['fixture'])
    monkeypatch.setattr(skill_review, '_docker', lambda *a, **k: 'image-digest')
    monkeypatch.setattr(skill_review, 'ensure', lambda *a: 'http://fixed-peer:8888')
    monkeypatch.setattr(skill_review, 'url_from_network', lambda *a: 'http://127.0.0.1:8888')
    monkeypatch.setattr(skill_review.relay, 'tcp_peer', lambda *a: nullcontext())
    monkeypatch.setattr(skill_review.subprocess, 'run', lambda *a, **k:
                        SimpleNamespace(returncode=0, stdout='{"host":"127.0.0.1","port":1234}'))

    def process(argv, **kwargs):
        kwargs['stdout'].write(json.dumps(frame) + '\n')
        kwargs['stdout'].flush()
        return SimpleNamespace(wait=lambda **kw: 0, returncode=0, poll=lambda: 0)

    monkeypatch.setattr(skill_review.subprocess, 'Popen', process)
    removed = []
    monkeypatch.setattr(skill_review, 'down', removed.append)
    details = {}
    profile = agents.PreparedProfile(tmp_path, ())
    result = skill_review._worker(adapter, profile, tmp_path, tmp_path, tmp_path, 'fixture-image',
                                  'fixture-model', {}, 10, 5, tmp_path, details)
    assert result == status
    assert details['returncode'] == 0
    assert removed == [details['container']]
    if status == 'quota_limited':
        assert details['quota']
    elif status == 'completed':
        assert details['native_totals']['usage']['input_tokens'] == 2
