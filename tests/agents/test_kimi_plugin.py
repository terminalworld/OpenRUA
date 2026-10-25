"""Bundled Kimi profile isolation, configuration and native launch contracts."""
import json
from pathlib import Path

import pytest

from openrua import agents, config
from openrua.cli import build_parser
from openrua.cli.commands import config as configure
from openrua.testing import check_manifest

MANIFEST = Path(__file__).resolve().parents[2] / 'openrua/configs/agents/kimi.yaml'


def test_external_manifest_uses_the_existing_contract_and_is_available_by_name():
    agent = check_manifest(MANIFEST)
    assert agent.name == 'kimi'
    assert any(m.name == 'kimi' for m in agents.manifests())
    assert not agent.inspect_login(Path('/missing')).available
    with pytest.raises(ValueError, match='native'):
        agent.prepare_profile(Path('/missing'), Path('/unused'))


def test_api_configuration_and_private_profile_do_not_load_native_credentials(tmp_path):
    key = tmp_path / 'key'; key.write_text('synthetic-kimi-key')
    args = build_parser().parse_args(['--home', str(tmp_path), 'config', 'set', '--user',
        '--agent', str(MANIFEST), '--auth', 'api', '--api-key-file', str(key)])
    configure.run(args)
    assert config.compose('panda', 'robosuite', None, tmp_path).cfg['agent']['auth']['mode'] == 'api'
    agent = agents.get(str(MANIFEST))
    profile = agent.prepare_api_profile(key, tmp_path / 'profile')
    content = (profile.directory / 'config.toml').read_text()
    assert 'api_key = "synthetic-kimi-key"' in content
    assert 'model = "kimi-k2.6"' in content
    assert 'base_url = "https://api.moonshot.ai/v1"' in content
    assert not (profile.directory / 'credentials').exists()
    assert profile.read_secrets() == ['synthetic-kimi-key']
    assert (profile.directory / 'config.toml').stat().st_mode & 0o777 == 0o600
    assert profile.mounts == (f'{profile.directory}:/kimi-home',)
    with pytest.raises(ValueError, match='separate'):
        agent.prepare_api_profile(key, tmp_path)
    key.write_text('')
    with pytest.raises(ValueError, match='nonempty'):
        agent.prepare_api_profile(key, profile.directory)
    assert (profile.directory / 'config.toml').exists()


def test_all_launch_modes_use_native_kimi_and_exact_resume():
    agent = agents.get(str(MANIFEST))
    chat = agent.conversation('box', agent.default_model, 'http://proxy', session_id='saved')
    assert chat.protocol.begin().outbound[0]['params']['session_id'] == 'saved'
    assert chat.argv[-6:] == ['--home', '/kimi-home', '--cwd', '/workspace', '--', 'kimi']
    assert 'python3' in chat.argv
    headless = agent.launch_argv('box', 'inspect', agent.default_model, 2, 'http://proxy', session_id='saved', resume=True)
    assert headless[-2:] == ['--session', 'saved']
    assert 'KIMI_LOOP_MAX_STEPS_PER_TURN=2' in headless
    for argv in (chat.argv, headless, agent.interactive_argv('box', agent.default_model, 'http://proxy')):
        assert 'KIMI_CODE_HOME=/kimi-home' in argv
    with pytest.raises(ValueError, match='exact session ID'):
        agent.launch_argv('box', 'inspect', agent.default_model, 2, '', resume=True)
    with pytest.raises(ValueError, match='default_model'):
        agent.conversation('box', 'unknown', '')


def bridge(tmp_path):
    from openrua.plugins.agents.kimi_server import Bridge
    return Bridge(tmp_path / 'profile', tmp_path / 'workspace', ['kimi'])


def test_native_registry_is_not_assumed_to_mean_http_is_ready(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from openrua.plugins.agents import kimi_server
    b = bridge(tmp_path)
    instances = b.home / 'server/instances'
    instances.mkdir(parents=True)
    (instances / 'server.json').write_text(json.dumps({'pid': 12345, 'host': '127.0.0.1', 'port': 1234}))
    monkeypatch.setattr(kimi_server.subprocess, 'Popen', lambda *a, **kw: SimpleNamespace(pid=12345, poll=lambda: None))
    reads = []
    def request(path):
        reads.append(path)
        if len(reads) == 1: raise TimeoutError('native HTTP routes are still starting')
        return {'server_version': '2.1.1'}
    b.request = request
    b.start()
    assert reads == ['meta', 'meta']
    assert b.base == 'http://127.0.0.1:1234'


def test_bridge_rejects_resume_from_a_different_workspace_or_pending_native_work(tmp_path):
    b = bridge(tmp_path)
    b.request = lambda *a: {'id': 'saved', 'metadata': {'cwd': '/other'}}
    with pytest.raises(ValueError, match='different workspace'):
        b.dispatch('open', {'cwd': str(b.cwd), 'session_id': 'saved'})
    b = bridge(tmp_path)
    responses = iter([{'id': 'saved', 'metadata': {'cwd': str(b.cwd)}}, {'active': {'id': 'unfinished'}, 'queued': []}])
    b.request = lambda *a: next(responses)
    with pytest.raises(ValueError, match='pending work'):
        b.dispatch('open', {'cwd': str(b.cwd), 'session_id': 'saved'})


def test_bridge_does_not_retry_or_replay_uncertain_prompt_submission(tmp_path):
    b = bridge(tmp_path); b.session = 'saved'
    calls = []
    def request(*args):
        calls.append(args)
        raise TimeoutError('response lost after native acceptance')
    b.request = request
    with pytest.raises(TimeoutError):
        b.dispatch('submit', {'prompt_id': 'p', 'text': 'inspect', 'model': 'm'})
    assert len(calls) == 1


def test_completed_prompt_racing_with_cancel_uses_native_already_completed_code(tmp_path):
    b = bridge(tmp_path); b.session = 'saved'; b.prompt = 'prompt'
    calls = []
    def request(path, body, **options):
        calls.append((path, body, options))
        return {'aborted': False}
    b.request = request
    assert b.dispatch('cancel', {'prompt_id': 'prompt'}) == {'aborted': False}
    assert calls[0][2] == {'accepted_codes': (0, 40903)}
    assert b.prompt == 'prompt'  # transcript still determines the terminal result
