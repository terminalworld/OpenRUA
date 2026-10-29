"""An unavailable Docker runtime must not be treated as a missing CLI."""

import subprocess
from types import SimpleNamespace

import pytest

from openrua.sandbox import SandboxError, check


@pytest.mark.parametrize('returncode', [0, 1])
def test_cli_result_preserves_output_and_status(monkeypatch, returncode):
    result = subprocess.CompletedProcess([], returncode, stdout='agent 1.2.3\n', stderr='native diagnostic\n')
    monkeypatch.setattr(check.subprocess, 'run', lambda *a, **k: result)
    assert check.cli_result('custom-image', 'agent --version') is result
    assert check.cli_ready('custom-image', 'agent --version') == (returncode == 0)


def test_docker_failure_keeps_its_original_diagnostic(monkeypatch):
    monkeypatch.setattr(check.subprocess, 'run', lambda *a, **k: SimpleNamespace(
        returncode=125, stderr='Docker data-root mount is unavailable', stdout=''))
    with pytest.raises(SandboxError, match='data-root mount') as caught:
        check.cli_ready('custom-image', 'codex --version')
    assert 'not rebuilt' in caught.value.hint


def test_timed_out_check_cleans_up_only_its_own_container(monkeypatch):
    calls = []
    def run(argv, **kwargs):
        calls.append(argv)
        if argv[1] == 'run':
            raise subprocess.TimeoutExpired(argv, 60)
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(check.subprocess, 'run', run)
    with pytest.raises(SandboxError, match='timed out'):
        check.cli_ready('custom-image', 'codex --version')
    name = calls[0][calls[0].index('--name') + 1]
    assert calls[1] == ['docker', 'rm', '-f', name]
    assert name.startswith('openrua-image-check-')


@pytest.mark.parametrize('already_removed', [False, True])
def test_interrupted_check_cleans_up_its_container_and_preserves_cancellation(monkeypatch, already_removed):
    calls = []
    interruption = KeyboardInterrupt()

    def run(argv, **kwargs):
        calls.append(argv)
        if argv[1] == 'run':
            raise interruption
        return SimpleNamespace(returncode=int(already_removed),
                               stderr='No such container' if already_removed else '')

    monkeypatch.setattr(check.subprocess, 'run', run)
    with pytest.raises(KeyboardInterrupt) as caught:
        check.cli_ready('custom-image', 'codex --version')
    assert caught.value is interruption
    name = calls[0][calls[0].index('--name') + 1]
    assert calls[1:] == [['docker', 'rm', '-f', name]]


@pytest.mark.parametrize('failure', ['docker', 'os', 'timeout', 'interrupt'])
def test_interrupted_check_reports_unconfirmed_cleanup(monkeypatch, failure):
    calls = []

    def run(argv, **kwargs):
        calls.append(argv)
        if argv[1] == 'run':
            raise KeyboardInterrupt()
        if failure == 'os':
            raise OSError('Docker socket disappeared')
        if failure == 'timeout':
            raise subprocess.TimeoutExpired(argv, 30)
        if failure == 'interrupt':
            raise KeyboardInterrupt()
        return SimpleNamespace(returncode=1, stderr='Docker daemon is unavailable')

    monkeypatch.setattr(check.subprocess, 'run', run)
    with pytest.raises(SandboxError, match='was interrupted.*cleanup failed') as caught:
        check.cli_ready('custom-image', 'codex --version')
    name = calls[0][calls[0].index('--name') + 1]
    assert calls[1:] == [['docker', 'rm', '-f', name]]
    assert f'docker rm -f {name}' in caught.value.hint
