"""Project exec reuses files across fresh runs; benchmarks retain isolated inputs."""

import hashlib
import json
import subprocess
import sys

import pytest

from openrua import agents
from openrua.cli import build_parser
from openrua.cli.commands import execute
from openrua.config import projects
from openrua.runner import main, session, trial
from openrua.sandbox import project
from tests.cli.test_exec import REAL
from tests.runner.test_trial_real import _harness


def apparatus(monkeypatch, tmp_path):
    handle, _, token = _harness(monkeypatch, tmp_path)
    calls, checks, contexts, prompts = [], [], [], []
    original_rpc = handle.rpc
    def rpc(command, **kwargs):
        original_rpc(command, **kwargs)
        return {'ok': True, 'success': False, 'language': 'inspect'}
    handle.rpc = rpc
    def bring_up(cfg, dest, sim, sandbox, suite, task_id, network, internet, mounts, *a, **kw):
        calls.append((dest, mounts, kw))
        return dest / 'config.yaml', handle, 0
    def preflight(cfg, sandbox, agent, **kw):
        checks.append(kw)
        return {'ok': True, 'checks': [], 'failed': []}
    def provenance(*args, **kw):
        prompts.append(kw['prompt'])
        return {'prompt_sha256': hashlib.sha256(kw['prompt'].encode()).hexdigest(),
                'workspace_template_sha256': kw['template_hash']}
    monkeypatch.setattr(trial, 'bring_up', bring_up)
    monkeypatch.setattr(trial, 'run_preflight', preflight)
    monkeypatch.setattr(main, 'not_ready', lambda backend: [])
    monkeypatch.setattr(main.record, 'provenance', provenance)
    return handle, token, calls, checks, contexts, prompts


@pytest.mark.parametrize('backend', ['sim', 'real'])
@pytest.mark.parametrize('external_records', [False, True])
def test_two_exec_tasks_share_project_and_keep_separate_records(
        tmp_path, monkeypatch, proxy_address, backend, external_records):
    handle, token, calls, checks, contexts, prompts = apparatus(monkeypatch, tmp_path)
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir(mode=0o750)
    monkeypatch.chdir(root)
    projects.trust(root, home)
    historical = {'machine.yaml': 'Previous observation\n', 'tools/custom.py': 'user tool\n',
                  'README.md': 'Project entry point\n', 'AGENTS.md': 'User instructions\n'}
    for name, content in historical.items():
        path = root / name
        path.parent.mkdir(exist_ok=True)
        path.write_text(content)
    (root / 'program.py').write_text(
        'from pathlib import Path\np = Path("count.txt")\n'
        'p.write_text(str(int(p.read_text()) + 1) if p.exists() else "1")\n')
    def operator(ctx):
        contexts.append(ctx)
        assert ctx['prompt_file'].read_text() == project.PROMPT
        subprocess.run([sys.executable, 'program.py'], cwd=root, check=True)
        return {'operator': 'fixture'}
    monkeypatch.setitem(main.OPERATORS, 'agent', operator)
    def run_set(cfg, source, args, user_home, label):
        args.token_file = str(token)
        args.credentials_dir = str(tmp_path / 'credentials')
        return main.run_set(cfg, source, args, user_home, label)
    monkeypatch.setattr(execute, 'run_set', run_set)
    profile = tmp_path / 'arm.yaml'
    profile.write_text(REAL)
    robot = str(profile) if backend == 'real' else 'panda'
    scene = [] if backend == 'real' else ['--sim', 'robosuite']
    records = ['--runs-root', str(tmp_path / 'records')] if external_records else []
    for words in ([robot, 'inspect', *scene], ['continue']):
        # Two runs with the same requested name must not replace either record.
        args = build_parser().parse_args(['--home', str(home), 'exec', *words,
                                         '--run-id', 'same', '--no-record', *records])
        assert execute.run(args) == 0
    assert (root / 'count.txt').read_text() == '2'
    assert len(contexts) == 2 and contexts[0]['profile_dir'] != contexts[1]['profile_dir']
    assert [ctx['task_language'] for ctx in contexts] == ['inspect', 'continue']
    assert calls[0][0] != calls[1][0]
    for dest, mounts, kw in calls:
        assert kw['workspace'] == root and kw['seed_workspace'] is False
        assert any(m.endswith(':/robot:ro') for m in mounts)
        rec = json.loads((dest / 'result.json').read_text())
        prov = json.loads((dest / 'provenance.json').read_text())
        assert rec['project'] == prov['project'] == str(root)
        assert 'workspace_template_sha256' not in rec and 'workspace_template_sha256' not in prov
        assert 'robot_materials_template_sha256' in rec
        assert prov['prompt_sha256'] == hashlib.sha256(project.PROMPT.encode()).hexdigest()
        assert rec['anomaly'] is None
    assert checks == [{'materials_root': '/robot'}] * 2
    assert prompts == [project.PROMPT] * 2
    assert (root.stat().st_mode & 0o777) == 0o750
    for name, content in historical.items():
        actual = (root / name).read_text()
        assert actual.startswith(content) if name == 'AGENTS.md' else actual == content
    assert (root / 'AGENTS.md').read_text().count('## OpenRUA robot project') == 1
    assert ('reset' in handle.calls) == (backend == 'sim')
    base = tmp_path / 'records' if external_records else root / '.openrua/runs'
    assert len(list(base.rglob('result.json'))) == 2


def test_bench_keeps_original_prompt_and_does_not_read_the_current_project(
        tmp_path, monkeypatch, proxy_address):
    _, token, calls, checks, _, prompts = apparatus(monkeypatch, tmp_path)
    root = tmp_path / 'project'
    root.mkdir()
    (root / 'AGENTS.md').write_text('Private project context\n')
    monkeypatch.chdir(root)
    seen = []
    monkeypatch.setitem(main.OPERATORS, 'agent', lambda ctx: seen.append(ctx) or {'operator': 'fixture'})
    args = build_parser().parse_args(['--home', str(tmp_path / 'home'), 'bench', '--config', 'libero_pro',
                                     '--run-id', 'fresh', '--task-suite', 'libero_goal_task',
                                     '--operator', 'agent'])
    args.token_file, args.credentials_dir = str(token), str(tmp_path / 'credentials')
    assert main.run(args) == 0
    assert prompts == [agents.PROMPT] and checks == [{}]
    assert 'prompt_file' not in seen[0]
    assert 'workspace' not in calls[0][2] and 'seed_workspace' not in calls[0][2]
    assert not (root / '.openrua').exists()
    assert (root / 'AGENTS.md').read_text() == 'Private project context\n'
    record = json.loads((calls[0][0] / 'result.json').read_text())
    assert 'project' not in record and record['workspace_template_sha256']


def test_launcher_receives_the_recorded_project_prompt_file(tmp_path):
    from types import SimpleNamespace
    path = tmp_path / 'prompt.txt'
    path.write_text(project.PROMPT)
    ctx = dict(sandbox='sandbox', task_language='inspect', proxy='', limits={}, prompt_file=path)
    argv = session.launcher_command(ctx, SimpleNamespace(name='fixture'), 'model', {}, 'native', tmp_path / 'transcript')
    assert argv[argv.index('--prompt-file') + 1] == str(path)


def test_project_startup_failure_returns_nonzero_and_keeps_the_record(tmp_path, monkeypatch, proxy_address):
    _, token, _, _, _, _ = apparatus(monkeypatch, tmp_path)
    root = tmp_path / 'project'
    root.mkdir()
    (root / 'program.py').write_text('print("maintained")\n')
    def failed(*args, **kwargs):
        raise RuntimeError('robot unavailable')
    monkeypatch.setattr(trial, 'bring_up', failed)
    args = build_parser().parse_args(['--home', str(tmp_path / 'home'), 'bench', '--config', 'libero_pro',
                                     '--run-id', 'failed', '--task-suite', 'libero_goal_task'])
    args.project, args.token_file = root, str(token)
    args.credentials_dir = str(tmp_path / 'credentials')
    assert main.run(args) == 1
    record = json.loads(next((root / '.openrua/runs').rglob('result.json')).read_text())
    assert record['termination'] == 'anomaly' and 'robot unavailable' in record['anomaly']
    assert (root / 'program.py').read_text() == 'print("maintained")\n'
