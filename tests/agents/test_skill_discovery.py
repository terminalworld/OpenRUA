"""Offline native discovery against installed CLIs, without credentials or models."""

import asyncio
import json
import subprocess
from uuid import uuid4

import pytest

from openrua import agents
from openrua.runner.skills import prepare
from openrua.sandbox import project
from tests.conftest import requires_image
from tests.test_skills import write_skill


@requires_image('openrua-sandbox-jazzy')
@pytest.mark.parametrize('agent_name', ['claude-code', 'codex'])
def test_native_discovery_after_new_process_and_profile(tmp_path, agent_name):
    async def run():
        root, home, materials = tmp_path / 'project', tmp_path / 'home', tmp_path / 'materials'
        adapter = agents.get(agent_name)
        source = tmp_path / 'empty-login'; source.mkdir()
        project.prepare({}, root, materials)
        original = 'User-authored native project instructions.\n'
        (root / 'CLAUDE.md').write_text(original)
        write_skill(root / '.agents/skills', 'native-probe')
        write_skill(home / 'skills', 'personal-probe')
        for index in (1, 2):
            profile = adapter.prepare_profile(source, tmp_path / f'profile-{index}', require_credentials=False,
                                              limits={'account_connectors': False})
            mounts = (*profile.mounts, *prepare(root, home, adapter, materials))
            if adapter.skills.instructions is not None:
                assert '/robot/skills.md' in (root / adapter.skills.instructions).read_text()
            assert (root / 'CLAUDE.md').read_text() == original
            name = 'openrua-skills-test-' + uuid4().hex[:10]
            argv = ['docker', 'run', '-d', '--name', name, '--network', 'none',
                    '-e', 'HOME=/home/robot', '-v', f'{root}:/workspace']
            for mount in mounts:
                argv.extend(['-v', mount])
            argv.extend(['openrua-sandbox-jazzy', 'sleep', 'infinity'])
            subprocess.run(argv, check=True, capture_output=True, timeout=30)
            try:
                connection = adapter.conversation(name, adapter.default_model, '', limits={'account_connectors': False})
                proc = await asyncio.create_subprocess_exec(*connection.argv, stdin=-1, stdout=-1, stderr=-1)
                async def send(frame):
                    proc.stdin.write((json.dumps(frame) + '\n').encode()); await proc.stdin.drain()
                async def until(predicate):
                    while True:
                        line = await asyncio.wait_for(proc.stdout.readline(), 25)
                        assert line, (await proc.stderr.read()).decode()[-2000:]
                        frame = json.loads(line)
                        if predicate(frame): return frame
                try:
                    if agent_name == 'codex':
                        await send({'id': 1, 'method': 'initialize', 'params': {'clientInfo': {'name': 'skill_test', 'version': '1'}}})
                        await until(lambda frame: frame.get('id') == 1)
                        await send({'method': 'initialized'})
                        await send({'id': 2, 'method': 'skills/list', 'params': {'cwds': ['/workspace'], 'forceReload': True}})
                        result = await until(lambda frame: frame.get('id') == 2)
                        assert 'error' not in result, result
                        discovered = {skill['name']: skill for entry in result['result']['data'] for skill in entry['skills']}
                        assert discovered['native-probe']['scope'] == 'repo'
                        assert discovered['personal-probe']['scope'] == 'user'
                    else:
                        await send(connection.protocol.begin().outbound[0])
                        result = await until(lambda frame: frame.get('type') == 'control_response')
                        discovered = {command['name']: command for command in result['response']['response']['commands']}
                        assert '(project)' in discovered['native-probe']['description']
                        assert '(user)' in discovered['personal-probe']['description']
                    if index == 2:
                        assert 'later-probe' in discovered
                finally:
                    proc.stdin.close()
                    try: await asyncio.wait_for(proc.wait(), 5)
                    except asyncio.TimeoutError:
                        proc.kill(); await proc.wait()
            finally:
                subprocess.run(['docker', 'rm', '-f', name], capture_output=True, timeout=30)
            if index == 1:
                write_skill(root / '.agents/skills', 'later-probe')
        assert (home / 'skills/personal-probe/SKILL.md').is_file()
        assert (root / 'AGENTS.md').read_text().count('## OpenRUA skills') == 1
    asyncio.run(run())
