"""Native profile staging must not import prepaid fallback or shared history."""
import json
from pathlib import Path

import pytest

from openrua import agents
from openrua.agents.credentials import resolve_profile

ROOT = Path(__file__).resolve().parents[2] / 'examples/plugins'


def kimi_profile(home):
    source = home / '.kimi-code'
    (source / 'credentials').mkdir(parents=True)
    (source / 'config.toml').write_text('''default_model = "managed-model"
[models.managed-model]
provider = "managed:kimi-code"
model = "kimi-k2.6"
max_context_size = 262144
capabilities = ["image_in", "tool_use"]
[providers."managed:kimi-code"]
type = "kimi"
base_url = "https://api.kimi.com/coding/v1"
api_key = "must-not-be-copied"
[providers."managed:kimi-code".oauth]
storage = "file"
key = "oauth/kimi-code"
[providers.prepaid]
type = "kimi"
api_key = "must-not-be-copied"
base_url = "https://api.moonshot.ai/v1"
''')
    (source / 'credentials/kimi-code.json').write_text(json.dumps({
        'access_token': 'synthetic-access', 'refresh_token': 'synthetic-refresh',
        'expires_at': 1, 'token_type': 'Bearer', 'scope': '', 'expires_in': 3600}))
    (source / 'sessions').mkdir()
    (source / 'sessions/private').write_text('history')
    return source


def zcode_profile(home):
    source = home / '.zcode'
    (source / 'v2').mkdir(parents=True)
    document = {'schemaVersion': 1, 'config': {
        'defaultModelSelection': {'providerId': 'plan', 'modelId': 'glm-4.5-air'},
        'providerConfigRules': {'providerRules': [
            {'providerId': 'plan', 'enabled': True, 'config': {
                'group': 'standard-personal',
                'access': {'type': 'zhipu-coding-plan-api-key', 'apiKey': 'synthetic-plan'},
                'api': {'type': 'anthropic-messages', 'baseUrl': 'https://open.bigmodel.cn/api/anthropic'},
                'personalModelIds': ['glm-4.5-air']}},
            {'providerId': 'prepaid', 'config': {'access': {'type': 'api-key', 'apiKey': 'must-not-be-copied'}}}]}}}
    (source / 'v2/provider_config.json').write_text(json.dumps(document))
    (source / 'v2/credentials.json').write_text('{"encrypted": "must-not-be-copied"}')
    return source


@pytest.mark.parametrize('name,fixture', [('kimi', kimi_profile), ('zcode', zcode_profile)])
def test_native_discovery_private_sessions_and_no_api_fallback(tmp_path, name, fixture):
    source = fixture(tmp_path)
    agent = agents.get(str(ROOT / name / 'agent.yaml'))
    alias = tmp_path / 'openrua/credentials' / name
    selected = resolve_profile(agent, None, alias, user_home=tmp_path, environment={}, link=True)
    assert selected == alias and alias.resolve() == source
    assert agent.inspect_login(selected).available
    one = agent.prepare_profile(selected, tmp_path / 'one')
    two = agent.prepare_profile(selected, tmp_path / 'two')
    assert one.directory != two.directory
    assert not (one.directory / 'sessions').exists()
    assert not (one.directory / '.zcode').exists()
    file = one.directory / ('config.toml' if name == 'kimi' else 'provider_config.json')
    assert 'must-not-be-copied' not in file.read_text()
    assert file.stat().st_mode & 0o777 == 0o600
    with pytest.raises(ValueError, match='separate'):
        agent.prepare_profile(source, source)
    assert agent.inspect_login(source).available
    # Missing native profiles never invoke API preparation or create an alias.
    missing = tmp_path / 'missing'
    assert not agent.inspect_login(missing).available
    with pytest.raises(ValueError):
        agent.prepare_profile(missing, tmp_path / 'unused')
    assert not (tmp_path / 'unused').exists()


def test_kimi_refresh_directory_is_shared_and_redaction_reads_rotated_tokens(tmp_path):
    source = kimi_profile(tmp_path)
    agent = agents.get(str(ROOT / 'kimi/agent.yaml'))
    prepared = agent.prepare_profile(source, tmp_path / 'profile')
    assert prepared.mounts[-2:] == (f'{source}/credentials:/kimi-home/credentials',
                                    f'{source}/oauth:/kimi-home/oauth')
    assert not (prepared.directory / 'credentials/kimi-code.json').exists()
    replacement = source / 'credentials/next.json'
    replacement.write_text('{"access_token":"rotated-access","refresh_token":"rotated-refresh"}')
    replacement.replace(source / 'credentials/kimi-code.json')
    assert prepared.read_secrets() == ['rotated-access', 'rotated-refresh']


@pytest.mark.parametrize('mutation', ['api', 'endpoint', 'model', 'malformed'])
def test_kimi_unsupported_native_selection_fails_before_staging(tmp_path, mutation):
    source = kimi_profile(tmp_path)
    path = source / 'config.toml'
    content = path.read_text()
    content = {'api': content.replace('default_model = "managed-model"', 'default_model = "prepaid"'),
               'endpoint': content.replace('https://api.kimi.com/coding/v1', 'https://untrusted.invalid'),
               'model': content.replace('model = "kimi-k2.6"', 'model = "other"'),
               'malformed': '[invalid' }[mutation]
    path.write_text(content)
    agent = agents.get(str(ROOT / 'kimi/agent.yaml'))
    assert not agent.inspect_login(source).available
    with pytest.raises(ValueError):
        agent.prepare_profile(source, tmp_path / 'unused')
    assert not (tmp_path / 'unused').exists()


@pytest.mark.parametrize('access_type', ['api-key', 'zhipu-account'])
def test_zcode_rejects_prepaid_and_encrypted_oauth_paths(tmp_path, access_type):
    source = zcode_profile(tmp_path)
    path = source / 'v2/provider_config.json'
    data = json.loads(path.read_text())
    data['config']['providerConfigRules']['providerRules'][0]['config']['access']['type'] = access_type
    path.write_text(json.dumps(data))
    agent = agents.get(str(ROOT / 'zcode/agent.yaml'))
    assert not agent.inspect_login(source).available
    with pytest.raises(ValueError, match='Coding Plan'):
        agent.prepare_profile(source, tmp_path / 'unused')
    assert not (tmp_path / 'unused').exists()


def test_zcode_native_base_override_uses_native_layout(tmp_path):
    source = zcode_profile(tmp_path)
    agent = agents.get(str(ROOT / 'zcode/agent.yaml'))
    selected = resolve_profile(agent, None, tmp_path / 'alias', user_home=tmp_path / 'other',
                               environment={'ZCODE_DATA_BASE_DIR': str(tmp_path)})
    assert selected == tmp_path
    assert agent.inspect_login(selected).available
    prepared = agent.prepare_profile(selected, tmp_path / 'profile')
    assert prepared.read_secrets() == ['synthetic-plan']
