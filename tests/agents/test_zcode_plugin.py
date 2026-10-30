"""The bundled agent uses the ordinary plugin and explicit-auth contracts."""
import json
from pathlib import Path

import pytest
import yaml

from openrua import agents, config
from openrua.cli import build_parser
from openrua.cli.commands import config as configure
from openrua.testing import check_manifest

MANIFEST = Path(__file__).resolve().parents[2] / 'openrua/configs/agents/zcode.yaml'


def test_external_plugin_loads_and_appears_as_a_bundled_option():
    agent = check_manifest(MANIFEST)
    assert agent.name == 'zcode'
    assert any(m.name == 'zcode' for m in agents.manifests())
    assert not agent.inspect_login(Path('/missing')).available
    with pytest.raises(ValueError, match='native'):
        agent.prepare_profile(Path('/missing'), Path('/unused'))


def test_explicit_api_profile_roundtrip_and_no_login_fallback(tmp_path):
    key = tmp_path / 'key'
    key.write_text('synthetic-zcode-key')
    args = build_parser().parse_args(['--home', str(tmp_path), 'config', 'set', '--user',
        '--agent', str(MANIFEST), '--auth', 'api', '--api-key-file', str(key)])
    configure.run(args)
    cfg = config.compose('panda', 'robosuite', None, tmp_path).cfg
    assert cfg['agent']['auth']['mode'] == 'api'
    agent = agents.get(str(MANIFEST))
    profile = agent.prepare_api_profile(key, tmp_path / 'profile')
    data = json.loads((profile.directory / 'provider_config.json').read_text())['config']
    provider = data['providerConfigRules']['providerRules'][0]['config']
    assert provider['api']['baseUrl'] == 'https://open.bigmodel.cn/api/paas/v4'
    assert provider['access']['apiKey'] == key.read_text()
    assert data['defaultModelSelection']['modelId'] == agent.default_model
    assert not (profile.directory / '.zcode/v2/credentials.json').exists()
    assert (profile.directory / 'provider_config.json').stat().st_mode & 0o777 == 0o600
    assert profile.read_secrets() == [key.read_text()]
    assert profile.mounts == (f'{profile.directory}:/zcode-home',)
    with pytest.raises(ValueError, match='separate'):
        agent.prepare_api_profile(key, tmp_path)
    key.write_text('')
    with pytest.raises(ValueError, match='nonempty'):
        agent.prepare_api_profile(key, profile.directory)
    assert (profile.directory / 'provider_config.json').exists()


def test_launches_share_native_configuration_and_require_exact_resume():
    agent = agents.get(str(MANIFEST))
    conversation = agent.conversation('box', agent.default_model, 'http://proxy', session_id='saved')
    assert conversation.argv[-2:] == ['zcode', 'app-server']
    first = conversation.protocol.begin().outbound[0]
    assert first['method'] == 'session/resume'
    assert first['params']['sessionId'] == 'saved'
    interactive = agent.interactive_argv('box', agent.default_model, 'http://proxy')
    headless = agent.launch_argv('box', 'inspect', agent.default_model, 1, 'http://proxy')
    for argv in (conversation.argv, interactive, headless):
        assert 'ZCODE_PERSONAL_PROVIDER_CONFIG_FILE=/zcode-home/provider_config.json' in argv
        assert 'ZCODE_DATA_BASE_DIR=/zcode-home' in argv
    with pytest.raises(ValueError, match='exact session ID'):
        agent.launch_argv('box', 'inspect', agent.default_model, 1, '', resume=True)
    with pytest.raises(ValueError, match='default_model'):
        agent.conversation('box', 'unconfigured-model', '')


def test_project_manifest_routes_glm_to_mistral_without_changing_native_agent(tmp_path):
    manifest = yaml.safe_load(MANIFEST.read_text())
    manifest['default_model'] = 'zai-glm-5-3'
    manifest['default_options']['api_base_url'] = 'https://api.mistral.ai/v1'
    manifest['whitelist'] = [r'^api\.mistral\.ai$']
    path = tmp_path / 'zcode-mistral.yaml'
    path.write_text(yaml.safe_dump(manifest))
    agent = agents.get(str(path))
    key = tmp_path / 'key'
    key.write_text('synthetic-mistral-key')
    profile = agent.prepare_api_profile(key, tmp_path / 'profile')
    native = json.loads((profile.directory / 'provider_config.json').read_text())['config']
    provider = native['providerConfigRules']['providerRules'][0]['config']
    assert provider['access'] == {'type': 'api-key', 'apiKey': 'synthetic-mistral-key'}
    assert provider['api'] == {'type': 'openai-chat-completions', 'baseUrl': 'https://api.mistral.ai/v1'}
    assert provider['personalModelIds'] == ['zai-glm-5-3']
    assert native['defaultModelSelection'] == {'providerId': 'openrua', 'modelId': 'zai-glm-5-3'}
    assert agent.whitelist == tuple(manifest['whitelist'])
    assert agent.conversation('box', agent.default_model, '').argv[-2:] == ['zcode', 'app-server']


def test_native_model_configuration_and_selection_options_are_owned_by_manifest(tmp_path):
    agent = agents.get(str(MANIFEST))
    model_config = {
        'properties': {'inputFormat': {'supportsImage': True}},
        'optionSpecs': {'reasoningLevel': {'values': ['none', 'high'],
                                         'map': '{"reasoning_effort": reasoningLevel}'},
                        'maxOutputTokens': {'map': '{"max_tokens": maxOutputTokens}'}},
    }
    agent.default_options = {**agent.default_options, 'model_config': model_config,
                             'model_options': {'reasoningLevel': 'none'}}
    key = tmp_path / 'key'; key.write_text('synthetic-key')
    profile = agent.prepare_api_profile(key, tmp_path / 'profile')
    native = json.loads((profile.directory / 'provider_config.json').read_text())['config']
    assert native['modelConfigRules'] == {'providerModelRules': [{
        'providerId': 'openrua', 'modelId': agent.default_model, 'config': model_config}],
        'manualProviderModelRules': []}
    assert native['defaultModelSelection']['options'] == {'reasoningLevel': 'none'}
    assert agent.default_options['model_config'] == model_config


@pytest.mark.parametrize('name', ['model_config', 'model_options'])
@pytest.mark.parametrize('value', [None, [], 'invalid', {'bad': float('nan')}, {1: 'not a string key'}])
def test_invalid_native_model_options_fail_before_reading_credentials(tmp_path, monkeypatch, name, value):
    from openrua.plugins.agents import zcode
    agent = agents.get(str(MANIFEST))
    agent.default_options = {**agent.default_options, name: value}
    monkeypatch.setattr(zcode, 'read_api_key', lambda path: pytest.fail('must validate options first'))
    with pytest.raises(ValueError, match=f'default_options.{name} must be a JSON object'):
        agent.prepare_api_profile(tmp_path / 'absent-key', tmp_path / 'profile')
    assert not (tmp_path / 'profile').exists()
