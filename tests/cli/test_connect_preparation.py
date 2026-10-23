"""A new real robot can be connected before any sandbox image is installed."""
import io
from types import SimpleNamespace

import pytest

from openrua.cli import build_parser, preparation
from openrua.cli.commands import connect
from openrua.errors import UnavailableError, UsageError
from openrua.terminal import launcher


@pytest.fixture
def images(monkeypatch):
    have, builds = set(), []
    monkeypatch.setattr(preparation, 'image_exists', lambda image: image in have)
    monkeypatch.setattr(preparation.subprocess, 'run',
                        lambda *a, **k: SimpleNamespace(returncode=0, stderr=''))

    def build(argv, **kwargs):
        builds.append(argv)
        have.add(argv[argv.index('--tag') + 1])
        return SimpleNamespace(stdout=io.StringIO('sandbox built\n'), wait=lambda: 0)

    monkeypatch.setattr(preparation.subprocess, 'Popen', build)
    return have, builds


@pytest.mark.parametrize('distro', ['jazzy', 'humble'])
def test_cli_connect_prepares_selected_image_without_robot_config(tmp_path, monkeypatch, images, distro):
    project = tmp_path / 'project'
    project.mkdir()
    monkeypatch.chdir(project)
    monkeypatch.setattr(connect.sys.stdin, 'isatty', lambda: True)
    monkeypatch.setattr(connect.sys.stdout, 'isatty', lambda: True)
    monkeypatch.setattr(connect.docker_setup, 'ensure', lambda: None)
    # Already signed in: login does not build an image on this path.
    monkeypatch.setattr(connect.login, 'ensure', lambda *a: None)
    launches = []

    def start(argv, *paths):
        assert f'openrua-sandbox-{distro}' in images[0]
        launches.append(argv)
        return 'client'

    monkeypatch.setattr(connect.service, 'start', start)
    monkeypatch.setattr(connect.start, 'open_terminal', lambda *a: 0)
    args = build_parser().parse_args(['--home', str(tmp_path), 'connect', '--agent', 'codex',
                                     '--distro', distro, '--trust-workspace'])
    assert connect.run(args) == 0
    assert len(images[1]) == 1 and len(launches) == 1
    command = images[1][0]
    assert command[-2:] == ['--agent', 'codex']
    assert command[command.index('--distro') + 1] == distro
    assert '--robot' not in command and '--sim' not in command
    assert launches[0][launches[0].index('--workspace') + 1] == str(project)
    assert not (tmp_path / 'robots').exists()
    assert connect.run(args) == 0
    assert len(images[1]) == 1 and len(launches) == 2


def test_connect_rebuilds_an_existing_image_missing_the_selected_agent(tmp_path, monkeypatch, images):
    images[0].add('openrua-sandbox-jazzy')

    def docker(argv, **kwargs):
        if argv[:2] == ['docker', 'run']:
            return SimpleNamespace(returncode=0 if images[1] else 127)
        return SimpleNamespace(returncode=0, stderr='')

    monkeypatch.setattr(preparation.subprocess, 'run', docker)
    preparation.prepare_connect(tmp_path, 'codex')
    assert len(images[1]) == 1
    command = images[1][0]
    assert command[command.index('--base-image') + 1] == 'openrua-sandbox-jazzy'
    preparation.prepare_connect(tmp_path, 'codex')
    assert len(images[1]) == 1


def test_api_connect_never_requests_native_sign_in(tmp_path, monkeypatch, images):
    key = tmp_path / 'provider.key'
    key.write_text('synthetic-provider-key')
    (tmp_path / 'config.yaml').write_text(
        'agent: codex\nagents:\n  codex:\n    auth:\n      mode: api\n      key_file: ' + str(key) + '\n')
    monkeypatch.setattr(connect.sys.stdin, 'isatty', lambda: True)
    monkeypatch.setattr(connect.sys.stdout, 'isatty', lambda: True)
    monkeypatch.setattr(connect.docker_setup, 'ensure', lambda: None)
    modes = []
    monkeypatch.setattr(connect.login, 'ensure', lambda home, agent, auth_mode='native': modes.append(auth_mode))
    monkeypatch.setattr(connect.service, 'start', lambda *a: 'client')
    monkeypatch.setattr(connect.start, 'open_terminal', lambda *a: 0)
    args = build_parser().parse_args(['--home', str(tmp_path), 'connect', '--agent', 'codex@0.163.0',
                                     '--workspace', str(tmp_path / 'project'), '--trust-workspace'])
    assert connect.run(args) == 0
    assert modes == ['api']
    assert images[1][0][-1] == 'codex@0.163.0'


def test_connect_saves_only_agent_choices_without_robot_configuration(tmp_path):
    from openrua import config
    from openrua.runner.connect import ConnectRequest, _agent
    key = tmp_path / 'provider.key'
    key.write_text('synthetic-provider-key')
    (tmp_path / 'config.yaml').write_text('robot: panda\nsimulator: robosuite\nbenchmark: capbench\n')
    connect.save_values(tmp_path, dict(agent='codex@0.163.0', model='chosen-model',
                                      auth_mode='api', api_key_file=str(key)))
    saved = config.load_user_config(tmp_path / 'config.yaml')
    assert (saved.robot, saved.simulator, saved.benchmark) == ('panda', 'robosuite', 'capbench')
    adapter, selected, model, _ = _agent(ConnectRequest(tmp_path, 'test', tmp_path / 'robot', agent='codex'))
    assert selected['auth'] == {'mode': 'api', 'key_file': str(key)}
    assert adapter.version == '0.163.0' and model == 'chosen-model'
    assert 'synthetic-provider-key' not in (tmp_path / 'config.yaml').read_text()


def test_invalid_connect_auth_does_not_modify_saved_choices(tmp_path):
    path = tmp_path / 'config.yaml'
    path.write_text('agent: claude-code\n')
    with pytest.raises(UsageError):
        connect.save_values(tmp_path, dict(agent='codex', model='', auth_mode='api',
                                          api_key_file=str(tmp_path / 'missing')))
    assert path.read_text() == 'agent: claude-code\n'


def test_connect_relative_manifest_keeps_auth_after_reopening_defaults(tmp_path, monkeypatch):
    from openrua.cli.agent_settings import resolve
    monkeypatch.chdir(tmp_path)
    manifest = tmp_path / 'custom.yaml'
    manifest.write_text(connect.paths.find('agents', 'codex').read_text())
    key = tmp_path / 'provider.key'
    key.write_text('synthetic-provider-key')
    connect.save_values(tmp_path, dict(agent='./custom.yaml', model='', auth_mode='api', api_key_file=str(key)))
    adapter, selected = resolve(tmp_path, None)
    assert selected['name'] == str(manifest)
    assert selected['auth']['mode'] == 'api'


def test_buildkit_missing_directory_has_storage_recovery_hint(tmp_path, monkeypatch, images):
    monkeypatch.setattr(preparation.subprocess, 'Popen', lambda *a, **k:
                        SimpleNamespace(stdout=io.StringIO('stat /docker/buildkit/containerd-overlayfs/cachemounts: '
                                                          'no such file or directory\n'), wait=lambda: 1))
    with pytest.raises(UnavailableError) as failure:
        preparation.prepare_connect(tmp_path, 'codex')
    assert 'docker buildx ls' in failure.value.hint
    assert 'docker ps' in failure.value.hint


def test_tui_connect_prepares_image_for_unsaved_selected_agent(tmp_path, monkeypatch, images):
    values = dict(robot='', sim='', bench='', agent='codex', model='', name='unused')
    monkeypatch.setattr(connect.service, 'start', lambda *a: 'client')
    result = launcher.setup(values, {}, 'config', lambda v: pytest.fail('no robot to save yet'),
                            None, None, {}, None, None,
                            lambda spec: {'action': 'connect', 'values': values},
                            sign_in=lambda v: None,
                            connect=lambda v: connect.launch(tmp_path, 'my-robot', agent=v['agent']))
    assert result[0] == 'client'
    assert images[1][0][-2:] == ['--agent', 'codex']


def test_connect_preparation_uses_configured_agent_and_pin(tmp_path, images):
    (tmp_path / 'config.yaml').write_text('agent: codex\nagents:\n  codex:\n    version: "0.163.0"\n')
    preparation.prepare_connect(tmp_path)
    assert images[1][0][-2:] == ['--agent', 'codex@0.163.0']


def test_connect_preparation_retains_custom_manifest_path(tmp_path, images):
    manifest = tmp_path / 'custom.yaml'
    manifest.write_text(connect.paths.find('agents', 'codex').read_text())
    preparation.prepare_connect(tmp_path, str(manifest) + '@0.163.0')
    assert images[1][0][-2:] == ['--agent', str(manifest) + '@0.163.0']


def test_failed_connect_build_keeps_log_and_never_starts_session(tmp_path, monkeypatch, images):
    monkeypatch.setattr(preparation.subprocess, 'Popen', lambda *a, **k:
                        SimpleNamespace(stdout=io.StringIO('download failed\n'), wait=lambda: 7))
    monkeypatch.setattr(connect.service, 'start', lambda *a: pytest.fail('must not launch'))
    with pytest.raises(UnavailableError, match='build exit status 7') as failure:
        connect.launch(tmp_path, 'my-robot', agent='codex')
    assert 'retry startup' in failure.value.hint
    assert next((tmp_path / 'preparation').glob('*.log')).read_text() == 'download failed\n'
    assert not (tmp_path / 'launches').exists()


def test_retained_session_is_rejected_before_preparation(tmp_path, monkeypatch):
    monkeypatch.setattr(connect.paths, 'sandbox_dir', lambda *a: tmp_path)
    monkeypatch.setattr(preparation, 'prepare_connect', lambda *a: pytest.fail('no preparation'))
    with pytest.raises(UsageError, match='retained or running work'):
        connect.launch(tmp_path, 'my-robot', name='retained')
