"""Native profile discovery shares one login and preserves explicit accounts."""
from pathlib import Path

import pytest

from openrua import agents


@pytest.mark.parametrize('name,native', [('claude-code', '.claude'), ('codex', '.codex')])
def test_native_login_is_linked_without_copy_and_refresh_remains_visible(tmp_path, name, native):
    agent = agents.get(name)
    source = tmp_path / native
    source.mkdir()
    secret = source / agent.credentials.filename
    secret.write_text('initial synthetic credential')
    alias = tmp_path / 'openrua/credentials' / name
    select = lambda link: agents.resolve_profile(agent, None, alias, user_home=tmp_path, environment={}, link=link)
    assert select(False) == source
    assert not alias.exists()  # doctor is read-only
    assert select(True) == alias and alias.is_symlink()
    replacement = source / 'new-login'
    replacement.write_text('refreshed synthetic credential')
    replacement.replace(secret)
    assert (alias / secret.name).read_text() == 'refreshed synthetic credential'
    assert select(True) == alias


def test_explicit_and_existing_accounts_are_never_silently_replaced(tmp_path):
    agent = agents.get('claude-code')
    alias = tmp_path / 'openrua/credentials/claude-code'
    alias.mkdir(parents=True)
    (alias / agent.credentials.filename).write_text('dedicated account')
    assert agents.resolve_profile(agent, '~/chosen', alias, user_home=tmp_path, environment={}, link=True) == tmp_path / 'chosen'
    assert agents.resolve_profile(agent, None, alias, user_home=tmp_path, environment={}, link=True) == alias
    assert not alias.is_symlink()
    override = {'CLAUDE_CONFIG_DIR': str(tmp_path / 'explicit-native')}
    assert agents.resolve_profile(agent, None, alias, user_home=tmp_path, environment=override, link=True) == tmp_path / 'explicit-native'
    assert (alias / agent.credentials.filename).read_text() == 'dedicated account'


def test_missing_login_and_broken_alias_are_reported_without_mutation(tmp_path):
    agent = agents.get('codex')
    alias = tmp_path / 'codex-login'
    assert agents.resolve_profile(agent, None, alias, user_home=tmp_path, environment={}, link=True) == tmp_path / '.codex'
    assert not alias.exists()
    alias.symlink_to(tmp_path / 'removed-account')
    assert agents.resolve_profile(agent, None, alias, user_home=tmp_path, environment={}, link=True) == alias
    assert alias.is_symlink()


def test_native_environment_profile_is_aliased_when_destination_is_absent(tmp_path):
    agent = agents.get('codex')
    source = tmp_path / 'custom-codex'
    source.mkdir()
    (source / 'auth.json').write_text('synthetic')
    alias = tmp_path / 'openrua/credentials/codex'
    selected = agents.resolve_profile(agent, None, alias, user_home=tmp_path,
                                      environment={'CODEX_HOME': str(source)}, link=True)
    assert selected == alias and alias.is_symlink() and alias.resolve() == source
