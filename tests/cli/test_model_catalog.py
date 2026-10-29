"""Drive native metadata streams without submitting a turn or retaining a child."""
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from openrua.agents.conversation import Conversation
from openrua.cli.model_catalog import read
from openrua.cli import model_catalog
from openrua.errors import UnavailableError
from openrua.plugins.agents.claude_conversation import ClaudeConversation


def test_native_metadata_only_returns_choices_without_submitting_a_prompt():
    program = '''
import json,sys
request=json.loads(sys.stdin.readline())
assert request['request']['subtype']=='initialize'
print(json.dumps({'type':'control_response','response':{'request_id':request['request_id'],
 'subtype':'success','response':{'models':[{'value':'native-model','displayName':'Native model'}]}}}),flush=True)
assert not sys.stdin.read()
'''
    rows = read(Conversation([sys.executable, '-u', '-c', program], ClaudeConversation('native')))
    assert rows[0]['value'] == 'native-model'


def test_native_early_exit_and_timeout_are_visible():
    with pytest.raises(UnavailableError, match='ended before'):
        read(Conversation([sys.executable, '-c', 'pass'], ClaudeConversation('native')))
    with pytest.raises(UnavailableError, match='timed out'):
        read(Conversation([sys.executable, '-c', 'import time; time.sleep(60)'],
                          ClaudeConversation('native')), timeout=.05)


@pytest.mark.parametrize('auth', ['native', 'api'])
@pytest.mark.parametrize('outcome', ['success', 'failure', 'cancel'])
def test_model_lookup_keeps_profile_selection_and_cleans_its_container(tmp_path, monkeypatch, auth, outcome):
    calls, profiles, queries = [], [], []
    source = tmp_path / 'selected-account'
    key = tmp_path / 'selected-key'
    def profile(selected, destination):
        profiles.append((selected, destination))
        destination.mkdir()
        return SimpleNamespace(mounts=[f'{destination}:/agent-profile'])
    def catalog(*args, **kwargs):
        queries.append((args, kwargs))
        return 'connection'
    adapter = SimpleNamespace(name='selected', default_model='default', prepare_profile=profile,
                              prepare_api_profile=profile, model_catalog=catalog)
    monkeypatch.setattr(model_catalog.agent_settings, 'resolve', lambda *a:
        (adapter, {'credentials_dir': str(source), 'options': {'effort': 'high'}}))
    monkeypatch.setattr(model_catalog.agents, 'api_key_file', lambda a, selected:
                        key if selected['mode'] == 'api' else None)
    monkeypatch.setattr(model_catalog.agents, 'resolve_profile', lambda a, selected, *args, **kwargs:
                        Path(selected))
    def run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=0, stderr='')
    monkeypatch.setattr(model_catalog.subprocess, 'run', run)
    def metadata(connection):
        assert connection == 'connection'
        if outcome == 'failure': raise UnavailableError('native lookup failed')
        if outcome == 'cancel': raise KeyboardInterrupt()
        return [{'value': 'native-model'}]
    monkeypatch.setattr(model_catalog, 'read', metadata)
    values = {'agent': 'selected', 'model': 'chosen', 'auth_mode': auth,
              'api_key_file': str(key)}
    if outcome == 'success':
        assert model_catalog.choices(tmp_path, values, 'agent-only-image') == [{'value': 'native-model'}]
    else:
        with pytest.raises(KeyboardInterrupt if outcome == 'cancel' else UnavailableError):
            model_catalog.choices(tmp_path, values, 'agent-only-image')
    assert profiles[0][0] == (key if auth == 'api' else source)
    assert not profiles[0][1].exists()
    name = calls[0][calls[0].index('--name') + 1]
    assert calls[-1] == ['docker', 'rm', '-f', name]
    assert calls[0][-3:] == ['agent-only-image', 'sleep', 'infinity']
    assert calls[0][calls[0].index('--network') + 1] == 'none'
    assert queries == [((name, 'chosen', ''), {'options': {'effort': 'high'}})]


@pytest.mark.parametrize('lookup_failed', [False, True])
def test_lookup_reports_unconfirmed_container_cleanup(tmp_path, monkeypatch, lookup_failed):
    calls = []
    adapter = SimpleNamespace(name='fixture', default_model='native',
        prepare_api_profile=lambda *a: SimpleNamespace(mounts=[]),
        model_catalog=lambda *a, **k: None)
    monkeypatch.setattr(model_catalog.agent_settings, 'resolve', lambda *a: (adapter, {}))
    monkeypatch.setattr(model_catalog.agents, 'api_key_file', lambda *a: tmp_path / 'key')
    def run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=int(argv[1] == 'rm'), stderr='Docker is not responding')
    monkeypatch.setattr(model_catalog.subprocess, 'run', run)
    if not lookup_failed:
        monkeypatch.setattr(model_catalog, 'read', lambda *a: [{'value': 'native-model'}])
    expected = 'does not expose.*\nCould not remove' if lookup_failed else '^Could not remove'
    with pytest.raises(UnavailableError, match=expected) as caught:
        model_catalog.choices(tmp_path, {'agent': 'fixture'}, 'agent-image')
    assert 'Docker is not responding' in str(caught.value)
    name = calls[0][calls[0].index('--name') + 1]
    assert caught.value.hint.endswith(f'docker rm -f {name}')
