"""Extending a terminal image keeps its identity and removes temporary tags."""

import json
from types import SimpleNamespace

import pytest

from openrua.sandbox import SandboxError, build


@pytest.mark.parametrize('fails', [False, True])
def test_extension_pins_the_base_and_retains_its_user(monkeypatch, fails):
    calls, built = [], []
    def run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=0, stdout=json.dumps({'Id': 'sha256:base', 'Config': {'User': '1001:1002'}}))
    def compile(dockerfile, context, tag, args, labels):
        built.append((args, labels))
        assert args['BASE_IMAGE'] == calls[1][-1]
        assert calls[1][2] == 'sha256:base'
        assert args['BASE_USER'] == '1001:1002'
        assert labels['openrua.base_image_sha256'] == 'sha256:base'
        if fails:
            raise SandboxError('fixture install failure')
        return 'sha256:extended'
    monkeypatch.setattr(build.subprocess, 'run', run)
    monkeypatch.setattr(build, '_docker_build', compile)
    if fails:
        with pytest.raises(SandboxError, match='fixture install failure'):
            build.build(base_image='custom', tag='result', preinstall='touch /opt/fixture')
    else:
        assert build.build(base_image='custom', tag='result', preinstall='touch /opt/fixture') == ('result', 'sha256:extended')
    assert calls[-1] == ['docker', 'rmi', calls[1][-1]]
    assert len(built) == 1


def test_extension_does_not_claim_to_recreate_users():
    with pytest.raises(SandboxError, match='preserves existing users'):
        build.build(base_image='custom', robot_uid=1234)
