"""The small CLI image has the shared runtime contract without ROS packages."""

import pytest

from openrua.sandbox import SandboxError, build


def test_agent_build_passes_generic_slots_and_describes_its_recipe(monkeypatch):
    captured = []
    monkeypatch.setattr(build, '_docker_build',
                        lambda *args: captured.append(args) or 'sha256:fixture')
    assert build.build_agent(tag='fixture', preinstall='install-cli', prepare='install-runtime',
                             robot_uid=1003, labels={'fixture.fact': 'value'}, stamp='fresh') == (
                                 'fixture', 'sha256:fixture')
    dockerfile, context, tag, args, labels = captured[0]
    assert dockerfile.name == 'agent.Dockerfile' and context == dockerfile.parent
    assert args == {'ROBOT_UID': '1003', 'PREPARE': 'install-runtime',
                    'PREINSTALL': 'install-cli', 'PREINSTALL_STAMP': 'fresh'}
    assert labels['openrua.agent_recipe_sha256'] == build.agent_recipe_sha256()
    assert labels['fixture.fact'] == 'value'
    recipe = dockerfile.read_text()
    assert 'FROM ubuntu:24.04' in recipe
    assert 'USER robot' in recipe and 'WORKDIR /workspace' in recipe
    assert 'python3' in recipe
    assert 'FROM ros:' not in recipe and 'ros-${' not in recipe
    assert recipe.index('ARG PREPARE') < recipe.index('ARG PREINSTALL')


@pytest.mark.parametrize('command', ['echo "unsupported"', 'line1\nline2'])
def test_agent_build_rejects_invalid_install_before_docker(monkeypatch, command):
    monkeypatch.setattr(build, '_docker_build', lambda *a: pytest.fail('must validate before building'))
    with pytest.raises(SandboxError, match='single-line'):
        build.build_agent(tag='invalid', preinstall=command)
