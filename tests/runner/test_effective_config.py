"""Recorded parameters are the ones passed to the launcher, before it runs."""

from copy import deepcopy
import json
import multiprocessing
import os

import pytest
import yaml

from openrua.cli import build_parser
from openrua.config import paths
from openrua.runner import main, record, session, trial
from openrua.runner import lock
from tests.runner.test_project_exec import apparatus
from tests.runner.test_trial_real import _harness


@pytest.mark.parametrize('wall,effort', [(60, None), (17, 'low')])
def test_recorded_config_matches_launcher_and_preserves_input(
        monkeypatch, tmp_path, proxy_address, wall, effort):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    cfg['agent'] = {'name': 'codex', 'model': 'selected-model',
                    'options': {} if effort is None else {'effort': effort}}
    cfg['protocol'].update(active_wall_clock_minutes=240, max_turns=23)
    original = deepcopy(cfg)
    run = tmp_path / 'run'
    directory = run / 'trials/suite-0/seed0'
    observed = []

    def bring_up(config, dest, *args, **kwargs):
        return record.write_config(dest, config), handle, 41

    def segment(cmd, dest, env, timeout_s, meta):
        saved = yaml.safe_load((dest / 'config.yaml').read_text())
        prov = json.loads((dest / 'provenance.json').read_text())
        run_config = json.loads((run / 'config.json').read_text())
        assert saved == prov['config'] == run_config['config']
        assert prov['input_config'] == run_config['input_config'] == original
        assert prov['ros_domain'] == 41 and prov['agent_cli']['version'] == '9.9.9'
        assert prov['containers']['sandbox'] == cmd[cmd.index('--sandbox') + 1]
        assert prov['agent_proxy'] == cmd[cmd.index('--proxy') + 1] == proxy_address
        assert saved['protocol']['active_wall_clock_minutes'] == timeout_s / 60 == wall
        assert saved['agent']['model'] == cmd[cmd.index('--model') + 1] == 'selected-model'
        assert f'effort={effort or "high"}' in cmd
        assert saved['agent']['options']['effort'] == (effort or 'high')
        assert json.loads(cmd[cmd.index('--limits') + 1]) == saved['protocol']['limits']
        assert cmd[cmd.index('--max-turns') + 1] == '23'
        observed.append(saved)
        (dest / 'transcript.jsonl').write_text('{"type":"thread.started","thread_id":"fixture"}\n')
        meta.update(termination='self_finished', launcher_returncode=0)
        return 100.0, 101.0, False

    monkeypatch.setattr(trial, 'bring_up', bring_up)
    monkeypatch.setattr(session, 'run_segment', segment)
    rec = trial.run_trial(cfg, tmp_path / 'robot.yaml', run, 'suite', 0, 0,
                          session.agent_operator, wall, token_file=str(token),
                          task='inspect', home=tmp_path / 'home')
    assert rec['anomaly'] is None and len(observed) == 1
    assert rec['operator_meta']['options'] == observed[0]['agent']['options']
    assert cfg == original


def test_run_set_keeps_actual_trial_facts_and_overrides_in_all_records(
        monkeypatch, tmp_path, proxy_address):
    handle, token, _, _, _, _ = apparatus(monkeypatch, tmp_path)

    def bring_up(config, dest, *args, **kwargs):
        return record.write_config(dest, config), handle, 37

    monkeypatch.setattr(trial, 'bring_up', bring_up)
    monkeypatch.setitem(main.OPERATORS, 'agent', lambda _: {'termination': 'self_finished'})
    args = build_parser().parse_args([
        '--home', str(tmp_path / 'home'), 'bench', '--config', 'libero_pro',
        '--agent', 'codex', '--model', 'selected-model', '--operator', 'agent',
        '--run-id', 'effective', '--task-suite', 'libero_goal_task',
        '--wall-clock-min', '60', '--internet', 'open', '--web-search',
        '--runs-root', str(tmp_path / 'records')])
    args.token_file, args.credentials_dir = str(token), str(tmp_path / 'credentials')
    assert main.run(args) == 0
    directory = next((tmp_path / 'records').rglob('provenance.json')).parent
    prov = json.loads((directory / 'provenance.json').read_text())
    saved = yaml.safe_load((directory / 'config.yaml').read_text())
    run_config = json.loads(next((tmp_path / 'records').rglob('config.json')).read_text())
    assert saved == prov['config'] == run_config['config']
    assert saved['protocol']['active_wall_clock_minutes'] == 60
    assert saved['protocol']['limits']['internet'] == 'open'
    assert saved['protocol']['limits']['web_search'] is True
    assert saved['agent']['options']['effort'] == 'high'
    assert prov['input_config']['protocol']['active_wall_clock_minutes'] == 240
    assert prov['input_config']['agent']['options'] == {}
    assert prov['input_config']['protocol']['limits']['internet'] == 'model-only'
    assert prov['ros_domain'] == 37 and prov['agent_cli']['version'] == '9.9.9'


def test_process_exit_during_operator_keeps_effective_configuration(
        monkeypatch, tmp_path, proxy_address):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    cfg['agent'] = {'name': 'codex', 'options': {}}
    cfg['protocol']['active_wall_clock_minutes'] = 240
    directory = tmp_path / 'run/trials/suite-0/seed0'

    def bring_up(config, dest, *args, **kwargs):
        return record.write_config(dest, config), handle, 19

    monkeypatch.setattr(trial, 'bring_up', bring_up)
    child = multiprocessing.get_context('fork').Process(target=trial.run_trial, args=(
        cfg, tmp_path / 'robot.yaml', tmp_path / 'run', 'suite', 0, 0,
        lambda _: os._exit(23), 60), kwargs={
            'token_file': str(token), 'task': 'inspect', 'home': tmp_path / 'home'})
    child.start()
    try:
        child.join(10)
        assert child.exitcode == 23
    finally:
        if child.is_alive():
            child.kill()
            child.join()
    prov = json.loads((directory / 'provenance.json').read_text())
    assert prov['config'] == yaml.safe_load((directory / 'config.yaml').read_text())
    assert prov['input_config']['protocol']['active_wall_clock_minutes'] == 240
    assert prov['config']['protocol']['active_wall_clock_minutes'] == 60
    assert prov['config']['agent']['options']['effort'] == 'high'
    assert prov['config']['agent']['model'] == trial.agents.get('codex').default_model
    assert prov['ros_domain'] == 19 and prov['agent_cli']['version'] == '9.9.9'
    assert not (directory / 'result.json').exists()


def test_early_configuration_snapshots_redact_known_credentials(
        monkeypatch, tmp_path, proxy_address):
    _, cfg, token = _harness(monkeypatch, tmp_path)
    secret = 'abcdefghijklmnopqrstuvwxyz0123456789'
    cfg['agent']['options'] = {'diagnostic': secret}
    run = tmp_path / 'run'
    seen = []

    def inspect(ctx):
        for path in (run / 'config.json', ctx['trial_dir'] / 'provenance.json'):
            content = path.read_text()
            assert secret not in content and '[REDACTED]' in content
        seen.append(True)
        return {'termination': 'self_finished'}

    rec = trial.run_trial(cfg, tmp_path / 'robot.yaml', run, 'suite', 0, 0, inspect, 60,
                          token_file=str(token), task='inspect', home=tmp_path / 'home')
    assert rec['anomaly'] is None and seen == [True]


@pytest.mark.parametrize('inline_pin', [False, True])
def test_custom_manifest_and_cli_pin_reach_the_actual_launcher_process(
        monkeypatch, tmp_path, proxy_address, inline_pin):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    manifest = yaml.safe_load(paths.find('agents', 'codex').read_text())
    manifest.update(default_model='custom-model', default_options={'effort': 'max'},
                    entry_point='./hooks.py')
    (tmp_path / 'custom.yaml').write_text(yaml.safe_dump(manifest))
    (tmp_path / 'hooks.py').write_text(
        'import json, sys\n'
        'from openrua.plugins.agents.codex import Codex\n'
        'class Custom(Codex):\n'
        '    def launch_argv(self, model, options, **kwargs):\n'
        '        event = {"type": "fixture.launch", "model": model,\n'
        '                 "options": options, "version_pin": self.version}\n'
        '        return [sys.executable, "-c", "print(" + repr(json.dumps(event)) + ")"]\n'
        'HOOKS = Custom\n')
    monkeypatch.chdir(tmp_path)
    spec = './custom.yaml' + ('@1.2.3' if inline_pin else '')
    cfg['agent'] = {'name': spec, 'version': '9.9.9' if inline_pin else '1.2.3', 'options': {}}
    run = tmp_path / 'run'

    def bring_up(config, dest, *args, **kwargs):
        return record.write_config(dest, config), handle, 0

    monkeypatch.setattr(trial, 'bring_up', bring_up)
    rec = trial.run_trial(cfg, tmp_path / 'robot.yaml', run, 'suite', 0, 0,
                          session.agent_operator, 1, token_file=str(token),
                          task='inspect', home=tmp_path / 'home')
    assert rec['anomaly'] is None
    assert rec['operator_meta']['termination'] == 'self_finished'
    directory = run / 'trials/suite-0/seed0'
    actual = json.loads((directory / 'transcript.jsonl').read_text())
    assert actual == {'type': 'fixture.launch', 'model': 'custom-model',
                      'options': {'effort': 'max'}, 'version_pin': '1.2.3'}
    saved = yaml.safe_load((directory / 'config.yaml').read_text())
    assert saved['agent']['name'] == str(tmp_path / 'custom.yaml') + ('@1.2.3' if inline_pin else '')
    assert saved['agent']['version'] == '1.2.3'
    assert json.loads((directory / 'provenance.json').read_text())['agent_cli']['name'] == 'codex'


@pytest.mark.parametrize('phase', ['network', 'proxy', 'profile'])
def test_prerequisite_failure_keeps_prior_attempt_and_leaves_no_claim(
        monkeypatch, tmp_path, phase):
    _, cfg, token = _harness(monkeypatch, tmp_path)
    directory = tmp_path / 'run/trials/suite-0/seed0'
    directory.mkdir(parents=True)
    (directory / 'result.json').write_text('{"previous":true}\n')

    def unavailable(*args, **kwargs):
        raise RuntimeError('prerequisite unavailable')

    owner, method = {
        'network': (trial, 'ensure_internal_network'),
        'proxy': (trial, 'ensure_proxy'),
        'profile': (trial.agents, 'resolve_profile'),
    }[phase]
    monkeypatch.setattr(owner, method, unavailable)
    with pytest.raises(RuntimeError, match='prerequisite unavailable'):
        trial.run_trial(cfg, tmp_path / 'robot.yaml', tmp_path / 'run', 'suite', 0, 0,
                        lambda _: pytest.fail('operator must not start'), 1,
                        token_file=str(token), task='inspect', home=tmp_path / 'home')
    assert lock.holder(directory) is None
    assert not (directory / 'attempts').exists()
    assert (directory / 'result.json').read_text() == '{"previous":true}\n'
