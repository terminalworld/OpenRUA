"""The external example uses the ordinary plugin and explicit-auth contracts."""
import json
from pathlib import Path

import pytest

from openrua import agents, config
from openrua.cli import build_parser
from openrua.cli.commands import config as configure
from openrua.testing import check_manifest

MANIFEST = Path(__file__).resolve().parents[2] / 'examples/plugins/zcode/agent.yaml'


def test_external_plugin_loads_and_does_not_appear_as_a_bundled_option():
    agent = check_manifest(MANIFEST)
    assert agent.name == 'zcode'
    assert all(m.name != 'zcode' for m in agents.manifests())
    assert not agent.inspect_login(Path('/missing')).available
    with pytest.raises(ValueError, match='native subscription'):
        agent.prepare_profile(Path('/missing'), Path('/unused'))


def test_explicit_api_profile_roundtrip_and_no_login_fallback(tmp_path):
    key = tmp_path / 'key'
    key.write_text('synthetic-zcode-key')
    args = build_parser().parse_args(['--home', str(tmp_path), 'config', 'set',
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
