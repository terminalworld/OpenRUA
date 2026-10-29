"""Selecting models prepares only a CLI and owns every temporary process."""

import io
import os
import subprocess
import sys
from types import SimpleNamespace

import pytest

from openrua.cli import model_catalog, preparation
from openrua.cli.commands import login, start
from openrua.errors import UnavailableError


@pytest.fixture(autouse=True)
def no_existing_agent_images(monkeypatch):
    monkeypatch.setattr('openrua.cli.agent_images.find', lambda *a: None)


@pytest.mark.parametrize('intent', ['start', 'connect'])
@pytest.mark.parametrize('auth_mode', ['native', 'api'])
def test_model_lookup_never_resolves_or_prepares_a_robot(tmp_path, monkeypatch, intent, auth_mode):
    prepared, queries, logins, progress = [], [], [], []
    def unexpected(*args, **kwargs):
        pytest.fail('Model lookup must not resolve or prepare robot resources')
    monkeypatch.setattr(start, 'prepare', unexpected)
    monkeypatch.setattr(start, 'validate_values', unexpected)
    monkeypatch.setattr(preparation, 'prepare_connect', unexpected)
    monkeypatch.setattr(start.docker_setup, 'ensure', lambda **kw: None)
    monkeypatch.setattr(preparation, '_prepare_images',
                        lambda *args, **kwargs: prepared.append((args, kwargs)))
    monkeypatch.setattr(login, 'ensure', lambda *a, **kw: logins.append((a, kw)))
    monkeypatch.setattr(model_catalog, 'choices',
                        lambda *args: queries.append(args) or [{'value': 'native-model'}])
    values = dict(agent='codex@0.164.0', model='selected-model', auth_mode=auth_mode,
                  robot='/missing/robot.yaml', sim='/missing/simulator.yaml',
                  bench='/missing/benchmark.yaml')
    application = SimpleNamespace(progress=progress.append)
    rows = start.discover_models(tmp_path, values, intent, application, lambda title: title)
    assert rows == [{'value': 'native-model'}]
    image, argv = prepared[0][0][1][0]
    assert argv[:2] == ['build', 'agent'] and argv[3].endswith('codex@0.164.0')
    assert '--distro' not in argv and '--robot' not in argv
    assert prepared[0][1] == {'retain_tools': False}
    assert queries == [(tmp_path, values, image)]
    assert progress[-1] == '[prepare] Loading models'
    assert len(logins) == (auth_mode == 'native')
    if logins:
        assert logins[0][1]['image'] == image
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('agent', ['kimi', 'zcode'])
def test_agents_without_native_catalog_do_not_prepare_an_image(tmp_path, monkeypatch, agent):
    monkeypatch.setattr(preparation, 'prepare_agent', lambda *a, **k: pytest.fail('must not build'))
    with pytest.raises(UnavailableError, match='does not provide a model list'):
        start.discover_models(tmp_path, {'agent': agent}, 'start', None, None)


def test_agent_image_retries_failures_and_reuses_the_verified_cli(tmp_path, monkeypatch):
    built, have, broken = [], set(), [True]
    monkeypatch.setattr(preparation.subprocess, 'run',
                        lambda *a, **k: SimpleNamespace(returncode=0, stderr=''))
    monkeypatch.setattr(preparation, 'image_exists', lambda image: image in have)
    monkeypatch.setattr(preparation, 'cli_ready', lambda *a: True)
    def build(argv, **kwargs):
        built.append(argv)
        image = argv[argv.index('--tag') + 1]
        if not broken[0]: have.add(image)
        return SimpleNamespace(stdout=io.StringIO('Native installer output\n'),
                               wait=lambda: int(broken[0]))
    monkeypatch.setattr(preparation.subprocess, 'Popen', build)
    with pytest.raises(UnavailableError, match='build exit status 1'):
        preparation.prepare_agent(tmp_path, 'codex@0.164.0', lambda line: None)
    broken[0] = False
    image = preparation.prepare_agent(tmp_path, 'codex@0.164.0', lambda line: None)
    assert preparation.prepare_agent(tmp_path, 'codex@0.164.0', lambda line: None) == image
    assert len(built) == 2
    other = preparation.prepare_agent(tmp_path, 'codex@0.165.0', lambda line: None)
    assert other != image and len(built) == 3
    assert all(argv[argv.index('build') + 1] == 'agent' for argv in built)
    assert not (tmp_path / 'sandboxes').exists()


def test_existing_verified_image_is_used_without_building(tmp_path, monkeypatch):
    seen = []
    def existing(adapter, probe):
        seen.append((adapter.name, adapter.version, probe))
        return 'sha256:current-agent'
    monkeypatch.setattr('openrua.cli.agent_images.find', existing)
    monkeypatch.setattr(preparation, '_prepare_images', lambda *a, **k: pytest.fail('must reuse'))
    assert preparation.prepare_agent(tmp_path, 'codex@0.164.0', lambda line: None) == 'sha256:current-agent'
    assert seen[0][:2] == ('codex', '0.164.0') and seen[0][2]


def test_cancel_build_stops_the_installer_and_its_child_before_returning(tmp_path, monkeypatch):
    fixture = tmp_path / 'installer.py'
    child_pid = tmp_path / 'child.pid'
    fixture.write_text('''import subprocess, sys
child = subprocess.Popen([sys.executable, '-u', '-c',
    "import os,sys,time; from pathlib import Path; Path(sys.argv[1]).write_text(str(os.getpid())); print('CHILD_READY', flush=True); time.sleep(60)", sys.argv[1]])
try:
    child.wait()
finally:
    child.wait()
''')
    spawn = subprocess.Popen
    processes = []
    def build(argv, **kwargs):
        child = spawn([sys.executable, str(fixture), str(child_pid)], **kwargs)
        processes.append(child)
        return child
    monkeypatch.setattr(preparation.subprocess, 'run',
                        lambda *a, **k: SimpleNamespace(returncode=0, stderr=''))
    monkeypatch.setattr(preparation.subprocess, 'Popen', build)
    monkeypatch.setattr(preparation, '_ready', lambda *a: False)
    def progress(line):
        if line == 'CHILD_READY':
            raise KeyboardInterrupt()
    try:
        with pytest.raises(KeyboardInterrupt):
            preparation._prepare_images(tmp_path, [('fixture', ['build', 'agent'])], progress,
                                        retain_tools=False)
        assert processes[0].poll() is not None
        with pytest.raises(ProcessLookupError):
            os.kill(int(child_pid.read_text()), 0)
        assert 'CHILD_READY' in next((tmp_path / 'preparation').glob('*.log')).read_text()
    finally:
        for child in processes:
            if child.poll() is None:
                os.killpg(child.pid, 9)
                child.wait()


def test_cancel_build_does_not_hide_a_child_cleanup_failure(tmp_path, monkeypatch):
    fixture = tmp_path / 'installer.py'
    fixture.write_text('''import signal, sys, time
def failed_cleanup(*args):
    print('Could not remove the check container: daemon unavailable', flush=True)
    sys.exit(69)
signal.signal(signal.SIGINT, failed_cleanup)
print('READY', flush=True)
time.sleep(60)
''')
    spawn = subprocess.Popen
    processes = []
    def build(argv, **kwargs):
        child = spawn([sys.executable, str(fixture)], **kwargs)
        processes.append(child)
        return child
    monkeypatch.setattr(preparation.subprocess, 'run',
                        lambda *a, **k: SimpleNamespace(returncode=0, stderr=''))
    monkeypatch.setattr(preparation.subprocess, 'Popen', build)
    monkeypatch.setattr(preparation, '_ready', lambda *a: False)
    def progress(line):
        if line == 'READY': raise KeyboardInterrupt()
    try:
        with pytest.raises(UnavailableError, match='Could not confirm.*daemon unavailable') as caught:
            preparation._prepare_images(tmp_path, [('fixture', ['build', 'agent'])], progress,
                                        retain_tools=False)
        assert processes[0].poll() == 69
        assert '.log' in caught.value.hint
        assert 'daemon unavailable' in next((tmp_path / 'preparation').glob('*.log')).read_text()
    finally:
        for child in processes:
            if child.poll() is None:
                os.killpg(child.pid, 9)
                child.wait()
