"""Metadata queries reuse matching installed images without robot configuration."""

from copy import copy
import json
import subprocess
from types import SimpleNamespace

import pytest

from openrua import agents
from openrua.cli import agent_images
from openrua.errors import UnavailableError
from openrua.sandbox import SandboxError


def image(identity, created, *tags):
    return {'Id': identity, 'Created': created, 'RepoTags': list(tags) or None}


def inventory(monkeypatch, images):
    calls = []

    def run(argv, **kwargs):
        calls.append(argv)
        output = '\n'.join(facts['Id'] for facts in images) if argv[2] == 'ls' else json.dumps(images)
        return SimpleNamespace(returncode=0, stdout=output, stderr='')

    monkeypatch.setattr(agent_images.subprocess, 'run', run)
    return calls


def test_newer_updated_sandbox_wins_over_old_metadata_image(monkeypatch):
    adapter = agents.get('codex')
    calls = inventory(monkeypatch, [
        image('sha256:old', '2026-10-29T12:00:00Z', 'openrua-agent-old:latest'),
        image('sha256:new', '2026-10-29T12:00:00.123456789Z', 'openrua-sandbox-jazzy:latest'),
    ])
    checked = []
    monkeypatch.setattr(agent_images, 'cli_ready', lambda identity, command: checked.append(identity) or True)
    assert agent_images.find(adapter, 'codex --version') == 'sha256:new'
    assert checked == ['sha256:new']
    assert f'label=openrua.agent.codex.install_sha256={agents.fact_sha256(adapter, "install")}' in calls[0]
    assert calls[1] == ['docker', 'image', 'inspect', 'sha256:old', 'sha256:new']


def test_candidate_base_and_dangling_images_are_not_selected(monkeypatch):
    inventory(monkeypatch, [
        image('sha256:candidate', '2026-10-29T15:00:00Z', 'openrua-agent-candidate-123:latest'),
        image('sha256:base', '2026-10-29T14:00:00Z', 'openrua-build-base-123:latest'),
        image('sha256:dangling', '2026-10-29T13:00:00Z'),
        image('sha256:valid', '2026-10-29T12:00:00Z', 'custom-tools:latest'),
    ])
    checked = []
    monkeypatch.setattr(agent_images, 'cli_ready', lambda identity, command: checked.append(identity) or True)
    assert agent_images.find(agents.get('codex'), None) == 'sha256:valid'
    assert checked == ['sha256:valid']


def test_wrong_pin_and_wrong_uid_are_checked_before_reuse(monkeypatch):
    adapter = agents.get('codex@0.164.0')
    inventory(monkeypatch, [
        image('sha256:wrong-pin', '2026-10-29T15:00:00Z', 'newer-tools:latest'),
        image('sha256:wrong-uid', '2026-10-29T14:00:00Z', 'other-user:latest'),
        image('sha256:compatible', '2026-10-29T13:00:00Z', 'matching-tools:latest'),
    ])
    monkeypatch.setattr(agent_images.os, 'getuid', lambda: 1003)
    probe = adapter.installation_check()[1]
    checked = []

    def ready(identity, command):
        checked.append(identity)
        assert command.startswith('test "$(id -u robot)" = "1003" && (')
        assert probe in command and '0.164.0' in command
        return identity == 'sha256:compatible'

    monkeypatch.setattr(agent_images, 'cli_ready', ready)
    assert agent_images.find(adapter, probe) == 'sha256:compatible'
    assert checked == ['sha256:wrong-pin', 'sha256:wrong-uid', 'sha256:compatible']


def test_custom_manifest_install_recipe_controls_image_filter(monkeypatch):
    adapter = copy(agents.get('codex'))
    adapter.prepare = 'install-custom-runtime'
    adapter.install_template = 'install-custom-cli@{version}'
    calls = inventory(monkeypatch, [])
    assert agent_images.find(adapter, 'custom-cli --version') is None
    assert len(calls) == 1
    custom = agents.fact_sha256(adapter, 'install')
    assert custom != agents.fact_sha256(agents.get('codex'), 'install')
    assert f'label=openrua.agent.codex.install_sha256={custom}' in calls[0]


@pytest.mark.parametrize('stage', ['list', 'inspect', 'probe'])
def test_docker_failure_is_not_a_cache_miss(monkeypatch, stage):
    facts = image('sha256:tools', '2026-10-29T12:00:00Z', 'tools:latest')

    def run(argv, **kwargs):
        listing = argv[2] == 'ls'
        failed = (stage == 'list' and listing) or (stage == 'inspect' and not listing)
        return SimpleNamespace(returncode=1 if failed else 0,
                               stdout='sha256:tools' if listing else json.dumps([facts]),
                               stderr='Docker daemon unavailable' if failed else '')

    def ready(*args):
        raise SandboxError('Docker daemon unavailable')

    monkeypatch.setattr(agent_images.subprocess, 'run', run)
    monkeypatch.setattr(agent_images, 'cli_ready', ready)
    with pytest.raises(UnavailableError, match='Docker daemon unavailable'):
        agent_images.find(agents.get('codex'), 'codex --version')


@pytest.mark.parametrize('failure', [OSError('Docker socket unavailable'), subprocess.TimeoutExpired('docker', 30)])
def test_unavailable_image_inspection_is_visible(monkeypatch, failure):
    def run(*args, **kwargs):
        raise failure

    monkeypatch.setattr(agent_images.subprocess, 'run', run)
    with pytest.raises(UnavailableError, match='Cannot inspect installed agent images'):
        agent_images.find(agents.get('codex'), None)


def test_no_compatible_image_returns_no_match(monkeypatch):
    inventory(monkeypatch, [image('sha256:wrong', '2026-10-29T12:00:00Z', 'tools:latest')])
    monkeypatch.setattr(agent_images, 'cli_ready', lambda *a: False)
    assert agent_images.find(agents.get('codex'), 'codex --version') is None
