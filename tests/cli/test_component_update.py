"""A component update preserves its environment and promotes only a checked target."""
import json
from types import SimpleNamespace

import pytest

from openrua import agents
from openrua.cli import build_parser
from openrua.cli.commands import update
from openrua.errors import UsageError


def test_dependency_preparation_is_conditional_and_separate_from_the_cli(tmp_path):
    import subprocess
    file = tmp_path / 'component.yaml'
    file.write_text('name: component\ndefault_model: m\nprepare: exit 99\nprepare_check: "true"\ninstall: echo component@{version}\n')
    manifest = agents.manifest(str(file) + '@2.0')
    assert 'exit 99' not in manifest.install
    command = agents.preparation([manifest]) + ' && ' + manifest.install
    result = subprocess.run(['bash', '-c', command], capture_output=True, text=True)
    assert result.returncode == 0 and result.stdout.strip() == 'component@2.0'
    one = agents.fact_sha256(manifest, 'install')
    assert agents.fact_sha256(agents.manifest(str(file) + '@3.0'), 'install') == one
    file.write_text(file.read_text().replace('exit 99', 'exit 98'))
    assert agents.fact_sha256(agents.manifest(str(file) + '@2.0'), 'install') != one


def test_an_update_requires_an_explicit_target(tmp_path):
    args = build_parser().parse_args(['--home', str(tmp_path), 'update', '--agent', 'codex', '--image', 'tools'])
    with pytest.raises(UsageError, match='explicit target'):
        update.run(args)


@pytest.mark.parametrize('current', [False, True])
def test_update_retains_the_exact_base_and_does_not_rebuild_machine_install(tmp_path, monkeypatch, current):
    calls, built = [], []
    base = {'Id': 'sha256:original', 'Config': {'Env': ['ROS_DISTRO=humble'], 'User': '1003'}}
    def run(argv, **kwargs):
        calls.append(argv)
        if argv[1:3] == ['image', 'inspect']:
            return SimpleNamespace(returncode=0, stdout=json.dumps(base), stderr='')
        # A current native CLI needs no new image. Its runtime is already valid.
        return SimpleNamespace(returncode=0 if current else 1, stdout='', stderr='')
    monkeypatch.setattr(update.subprocess, 'run', run)
    monkeypatch.setattr(update.build, '_sandbox', lambda *a, **k: built.append((a, k)) or ('tools', 'sha256:new'))
    args = build_parser().parse_args(['--home', str(tmp_path), 'update', '--agent', 'codex@0.164.0', '--image', 'tools'])
    assert update.run(args) == 0
    if current:
        assert not built
    else:
        positional, keyword = built[0]
        assert positional[0] == ['codex@0.164.0']
        assert positional[2] == 'humble' and positional[4] == 'tools'
        assert len(positional) == 6  # no robot argument, hence no driver installation
        assert keyword['base_image'] == 'sha256:original'
    assert not any('build' in call or '--robot' in call for call in calls)


def test_a_current_cli_still_repairs_a_missing_runtime_dependency(tmp_path, monkeypatch):
    built = []
    def run(argv, **kwargs):
        if argv[1:3] == ['image', 'inspect']:
            return SimpleNamespace(returncode=0, stdout=json.dumps({'Id':'base','Config':{'Env':[]}}), stderr='')
        return SimpleNamespace(returncode=1 if 'process.versions.node' in argv[-1] else 0, stdout='', stderr='')
    monkeypatch.setattr(update.subprocess, 'run', run)
    monkeypatch.setattr(update.build, '_sandbox', lambda *a, **k: built.append((a,k)) or ('tools','new'))
    args = build_parser().parse_args(['--home',str(tmp_path),'update','--agent','codex@0.164.0','--image','tools'])
    assert update.run(args) == 0 and len(built) == 1


def test_an_unsupported_pinned_preview_version_is_a_usage_error(tmp_path):
    args = build_parser().parse_args(['--home',str(tmp_path),'update','--agent','kimi@9.0.0','--image','tools'])
    with pytest.raises(UsageError, match='supports Kimi Code 2.1.1 only'):
        update.run(args)


def test_plugin_checks_do_not_replace_the_exact_target_version_check():
    import subprocess
    class Component(agents.Agent):
        def sandbox_cli_check(self):
            return 'extra_check', 'true'
    component = Component(name='fixture', default_model='m', version='2.0.0', version_argv=('echo','1.0.0'))
    result = subprocess.run(['bash','-c',component.installation_check()[1]], capture_output=True)
    assert result.returncode != 0


@pytest.mark.parametrize('version', ['2.0;echo invalid', '$(echo invalid)', '2.0\ntrue'])
def test_release_identifiers_cannot_become_shell_commands(version):
    with pytest.raises(UsageError, match='invalid CLI version'):
        agents.manifest('codex@' + version)


def test_version_command_retains_argument_boundaries():
    import subprocess
    component = agents.Agent(name='fixture', default_model='m',
                             version_argv=('printf', '%s', '1.0.0 value; echoed'))
    result = subprocess.run(['bash','-c',component.version_probe()], capture_output=True, text=True)
    assert result.returncode == 0 and result.stdout == '1.0.0 value; echoed'


def test_installer_errors_keep_their_message_at_the_cli_boundary(monkeypatch):
    from openrua.errors import UnavailableError
    from openrua.sandbox import SandboxError
    def fail(args):
        raise SandboxError('fixture native install failure')
    monkeypatch.setattr(update, '_update', fail)
    with pytest.raises(UnavailableError, match='fixture native install failure'):
        update.run(SimpleNamespace(image='tools'))
