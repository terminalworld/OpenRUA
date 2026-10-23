"""Image repairs promote only their own verified candidate."""

from types import SimpleNamespace

import pytest

from openrua.cli.commands import build
from openrua.errors import UnavailableError


def test_each_build_checks_and_promotes_its_own_candidate(tmp_path, monkeypatch):
    candidates, docker, checks = [], [], []
    def compile(**kwargs):
        candidates.append(kwargs['tag'])
        assert kwargs['base_image'] == 'custom-tools'
        return kwargs['tag'], 'digest'
    monkeypatch.setattr(build, 'build_sandbox', compile)
    monkeypatch.setattr(build, '_report_versions', lambda tag, agents: checks.append(tag))
    monkeypatch.setattr(build.subprocess, 'run', lambda argv, **kw: docker.append(argv) or SimpleNamespace(returncode=0))
    for _ in range(2):
        build._sandbox(['codex'], 'touch /opt/fixture', 'humble', None, 'tools', tmp_path, base_image='custom-tools')
    assert candidates[0] != candidates[1]
    assert checks == candidates
    assert [argv for argv in docker if argv[1] == 'tag'] == [['docker', 'tag', tag, 'tools'] for tag in candidates]


def test_failed_cli_validation_does_not_replace_the_working_tag(tmp_path, monkeypatch):
    docker = []
    monkeypatch.setattr(build, 'build_sandbox', lambda **kw: (kw['tag'], 'digest'))
    def reject(*args):
        raise UnavailableError('fixture CLI is not usable')
    monkeypatch.setattr(build, '_report_versions', reject)
    monkeypatch.setattr(build.subprocess, 'run', lambda argv, **kw: docker.append(argv) or SimpleNamespace(returncode=0))
    with pytest.raises(UnavailableError, match='not usable'):
        build._sandbox(['codex'], None, 'humble', None, 'custom-tools', tmp_path, base_image='custom-tools')
    assert len(docker) == 1 and docker[0][:2] == ['docker', 'rmi']
