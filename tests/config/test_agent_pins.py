"""A CLI version pin keeps the selected agent's authentication and model."""

import pytest

from openrua import agents, config
from openrua.cli import build_parser, preparation
from openrua.cli.commands import start


@pytest.fixture
def home(tmp_path):
    (tmp_path / 'config.yaml').write_text(
        'agents:\n  codex:\n    version: 0.162.0\n    model: saved-model\n'
        '    credentials_dir: /tmp/synthetic-native-profile\n'
        '    auth:\n      mode: api\n      key_file: /tmp/synthetic-provider-key\n'
        '    options:\n      effort: low\n')
    return tmp_path


def test_composed_pin_keeps_base_agent_preferences_and_overrides_version(home):
    composed = config.compose('panda', 'robosuite', '', home, agent='codex@0.163.0')
    selected = composed.cfg['agent']
    assert selected['auth'] == {'mode': 'api', 'key_file': '/tmp/synthetic-provider-key'}
    assert selected['model'] == 'saved-model'
    assert selected['credentials_dir'] == '/tmp/synthetic-native-profile'
    assert selected['options']['effort'] == 'low'
    assert agents.get(selected['name'], version=selected.get('version')).version == '0.163.0'
    command = preparation.image_commands(composed)[-1][1]
    assert command[command.index('--agent') + 1] == 'codex@0.163.0'


def test_start_with_a_pin_uses_the_saved_authentication(home):
    args = build_parser().parse_args(['--home', str(home), '--robot', 'panda', '--sim', 'robosuite',
                                     '--bench', '', '--agent', 'codex@0.163.0'])
    values, _ = start.values_for(args)
    assert values['agent'] == 'codex@0.163.0'
    assert values['auth_mode'] == 'api' and values['model'] == 'saved-model'


def test_same_agent_pin_keeps_benchmark_facts_but_other_agents_do_not(home):
    package = config.load_user_config(config.paths.package_config_path())
    user = config.load_user_config(home / 'config.yaml')
    benchmark = {'name': 'codex', 'model': 'benchmark-model', 'options': {'effort': 'high'}}
    selected = config.layer_agent(benchmark, package, user, agent='codex@0.163.0')
    assert selected['model'] == 'benchmark-model' and selected['options']['effort'] == 'high'
    assert selected['auth']['mode'] == 'api'
    other = config.layer_agent(benchmark, package, user, agent='claude-code')
    assert other.get('model') != 'benchmark-model' and other.get('auth', {}).get('mode') != 'api'


def test_default_pin_and_legacy_pin_specific_settings_keep_base_auth(home):
    with (home / 'config.yaml').open('a') as stream:
        stream.write('  codex@0.163.0:\n    model: legacy-pin-model\nagent: codex@0.163.0\n')
    selected = config.compose('panda', 'robosuite', '', home).cfg['agent']
    assert selected['model'] == 'legacy-pin-model' and selected['auth']['mode'] == 'api'
    from openrua.cli.agent_settings import resolve
    adapter, selected = resolve(home, None)
    assert adapter.version == '0.163.0' and selected['model'] == 'legacy-pin-model'
    assert selected['auth']['mode'] == 'api'


def test_custom_manifest_pin_uses_its_canonical_preferences(home, monkeypatch):
    import yaml
    manifest = home / 'custom.yaml'
    manifest.write_text(config.paths.find('agents', 'codex').read_text())
    (home / 'config.yaml').write_text(yaml.safe_dump({'agents': {
        str(manifest): {'model': 'custom-model', 'auth': {'mode': 'api', 'key_file': '/tmp/synthetic-provider-key'}}}}))
    monkeypatch.chdir(home)
    selected = config.compose('panda', 'robosuite', '', home, agent='./custom.yaml@0.163.0').cfg['agent']
    assert selected['model'] == 'custom-model' and selected['auth']['mode'] == 'api'


def test_pin_cannot_bypass_the_benchmarks_fixed_authentication(home, monkeypatch):
    from types import SimpleNamespace
    from openrua.errors import UsageError
    key = home / 'key'; key.write_text('synthetic-key')
    fixed = {'name': 'codex', 'auth': {'mode': 'native'}}
    monkeypatch.setattr(config, 'compose', lambda *a, **k: SimpleNamespace(cfg={'agent': {'name': 'codex@0.163.0'}}))
    monkeypatch.setattr(config, 'load_benchmark', lambda *a: (None, {'agent': fixed}))
    with pytest.raises(UsageError, match='prescribes different authentication'):
        start.validate_values(home, dict(robot='panda', sim='robosuite', bench='fixture', agent='codex@0.163.0',
                                         model='', auth_mode='api', api_key_file=str(key)))
