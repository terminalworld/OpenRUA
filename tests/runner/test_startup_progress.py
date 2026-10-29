"""Startup progress reports actual work without exposing unrelated log output."""

import io

import pytest

from openrua.runner import startup


@pytest.mark.parametrize('fails', [False, True])
def test_phase_records_duration_and_preserves_failure(monkeypatch, capsys, fails):
    clock = iter((4.0, 4.25))
    monkeypatch.setattr(startup.time, 'monotonic', lambda: next(clock))
    error = RuntimeError('original failure')

    def operation():
        with startup.phase('Starting the agent container'):
            assert capsys.readouterr().out == '[startup] Starting the agent container\n'
            if fails:
                raise error

    if fails:
        with pytest.raises(RuntimeError) as caught:
            operation()
        assert caught.value is error
    else:
        operation()
    outcome = 'failed' if fails else 'completed'
    assert capsys.readouterr().out == f'[startup timing] Starting the agent container: {outcome} in 0.250s\n'


def test_partial_phase_waits_for_newline_and_other_output_is_not_promoted():
    stream = io.StringIO('build details\n[prepare] not an owner phase\n[startup] Starting the agent')
    seen = []
    startup.read_progress(stream, seen.append)
    assert not seen
    position = stream.tell()
    stream.seek(0, 2)
    stream.write(' container\n[startup timing] Starting the agent container: completed in 0.250s\n')
    stream.seek(position)
    startup.read_progress(stream, seen.append)
    assert seen == ['[prepare] Starting the agent container',
                    '[startup timing] Starting the agent container: completed in 0.250s']
    startup.read_progress(stream, seen.append)
    assert len(seen) == 2
