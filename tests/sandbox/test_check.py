"""An unavailable Docker runtime must not be treated as a missing CLI."""

import subprocess
from types import SimpleNamespace

import pytest

from openrua.sandbox import SandboxError, check


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
