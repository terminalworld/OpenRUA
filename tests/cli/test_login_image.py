"""Native sign-in can use the exact environment prepared for model discovery."""

from types import SimpleNamespace

import pytest

from openrua import agents
from openrua.cli.commands import build, login
from openrua.errors import AuthError


@pytest.mark.parametrize('name', ['codex', 'claude-code'])
def test_explicit_image_sign_in_never_finds_or_builds_a_sandbox(tmp_path, monkeypatch, name):
    adapter = agents.get(name)
    where = tmp_path / 'profile'
    monkeypatch.setattr(login, 'image_with', lambda *_: pytest.fail('must use the supplied image'))
    monkeypatch.setattr(build, '_sandbox', lambda *a, **kw: pytest.fail('must not build a ROS sandbox'))
    calls = []

    def run(argv, **kwargs):
        calls.append(argv)
        (where / adapter.credentials.filename).write_text('{}')
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(login.subprocess, 'run', run)
    login.sign_in(tmp_path, adapter, where, image='prepared-agent-image')
    assert len(calls) == 1
    assert calls[0][-len(adapter.credentials.login) - 1:] == [
        'prepared-agent-image', *adapter.credentials.login]
    assert f'{where.resolve()}:{adapter.credentials.mount_point}' in calls[0]


@pytest.mark.parametrize('interactive', [False, True])
def test_ensure_preserves_image_and_agent_pin_through_terminal_handoff(tmp_path, monkeypatch, interactive):
    where = tmp_path / 'profile'
    monkeypatch.setattr(login, 'directory', lambda *a: where)
    calls, handoffs = [], []

    def sign_in(home, adapter, target, *, image):
        calls.append((home, adapter.name, adapter.version, target, image))

    def interact(operation):
        handoffs.append(True)
        operation()

    monkeypatch.setattr(login, 'sign_in', sign_in)
    login.ensure(tmp_path, 'codex@0.164.0', ask=lambda _: 'y',
                 interact=interact if interactive else None, image='prepared-codex-image')
    assert calls == [(tmp_path, 'codex', '0.164.0', where, 'prepared-codex-image')]
    assert handoffs == ([True] if interactive else [])


def test_declined_sign_in_does_not_start_the_explicit_image(tmp_path, monkeypatch):
    monkeypatch.setattr(login, 'directory', lambda *a: tmp_path / 'profile')
    monkeypatch.setattr(login, 'sign_in', lambda *a, **kw: pytest.fail('sign-in was declined'))
    with pytest.raises(AuthError, match='not signed in'):
        login.ensure(tmp_path, 'codex', ask=lambda _: 'n',
                     interact=lambda operation: operation(), image='prepared-codex-image')


def test_api_auth_does_not_request_native_sign_in_with_an_image(tmp_path, monkeypatch):
    monkeypatch.setattr(login, '_settings', lambda *a: pytest.fail('API auth needs no native login'))
    login.ensure(tmp_path, 'codex', 'api', image='prepared-codex-image')
