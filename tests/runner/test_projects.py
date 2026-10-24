"""Persistent project files survive new native processes and conversation reopening."""

import asyncio
from dataclasses import replace
import sys
from types import SimpleNamespace

import pytest

from openrua.agents.conversation import Conversation
from openrua.config import compose
from openrua.runner import history, live, live_state, managed
from openrua.sandbox import project
from openrua.sessions.core import Session, initial_state
from openrua.sessions.store import SQLiteStore
from tests.runner.test_live import resources
from tests.sessions.test_execution import FakeProtocol, until


def test_project_files_and_instructions_survive_new_connections(tmp_path):
    root, first, second = tmp_path / 'project', tmp_path / 'first', tmp_path / 'second'
    root.mkdir()
    existing = {'README.md': 'Existing project\n', 'AGENTS.md': 'User instructions\n',
                'machine.yaml': 'Historical machine\n', 'scripts/grasp.py': 'print("maintained")\n'}
    for name, content in existing.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    cfg = compose('panda', 'robosuite', '', tmp_path / 'home').cfg
    project.prepare(cfg, root, first)
    instructions = (root / 'AGENTS.md').read_text()
    (first / 'machine.yaml').write_text('old connection')
    project.prepare(cfg, root, second)
    for name, content in existing.items():
        if name != 'AGENTS.md':
            assert (root / name).read_text() == content
    assert (root / 'AGENTS.md').read_text() == instructions
    assert instructions.startswith(existing['AGENTS.md']) and '/robot/README.md' in instructions
    assert (first / 'machine.yaml').read_text() == 'old connection'
    assert (second / 'machine.yaml').read_text() != 'old connection'
    assert 'python3 /robot/tools/' in (second / 'README.md').read_text()
    for name in ('10-machine.md', '20-perception.md', '30-action.md'):
        document = (second / 'docs' / name).read_text()
        assert 'open("/robot/machine.yaml")' in document
        assert 'open("machine.yaml")' not in document
    assert 'in the read-only robot materials. Read it first.' in (second / 'docs/10-machine.md').read_text()
    assert '`machine-discrepancies.yaml` at the workspace root' in (second / 'docs/10-machine.md').read_text()


def test_two_resource_sessions_mount_the_exact_same_project(resources, tmp_path, monkeypatch, proxy_address, agent_cli_ok):
    request, calls, machine = resources
    root = tmp_path / 'project'
    root.mkdir(mode=0o750)
    seen = []
    def bring_up(cfg, workdir, *args, **kwargs):
        seen.append((workdir, kwargs))
        return None, machine, 0
    monkeypatch.setattr(live, 'bring_up', bring_up)
    for name in ('first', 'second'):
        robot = live.open_robot(replace(request, name=name, project=root))
        facts = live_state.load(name, request.home)
        assert facts['workspace'] == facts['project'] == str(root.resolve())
        assert seen[-1][1]['workspace'] == root and seen[-1][1]['seed_workspace'] is False
        if name == 'first':
            (root / 'program.py').write_text('print("continuing")\n')
        else:
            assert (root / 'program.py').read_text() == 'print("continuing")\n'
        robot.power_off()
    assert root.stat().st_mode & 0o777 == 0o750
    assert seen[0][0] != seen[1][0]
    assert all(path.is_relative_to(root / '.openrua' / 'runs') for path, _ in seen)


def test_project_history_is_exact_and_all_projects_remain_available(tmp_path):
    first, second = tmp_path / 'a', tmp_path / 'a' / 'nested'
    for name, root in [('one', first), ('two', first), ('three', second), ('legacy', None)]:
        SQLiteStore(tmp_path / 'sandboxes' / name / 'conversation.sqlite', initial_state()).close()
        if root:
            live_state.save(name, tmp_path, project=str(root))
    assert {row['id'] for row in history.entries(tmp_path, first)} == {'one', 'two'}
    assert {row['id'] for row in history.entries(tmp_path, second)} == {'three'}
    assert {row['id'] for row in history.entries(tmp_path)} == {'one', 'two', 'three', 'legacy'}


def test_reopen_preserves_native_identity_and_never_dispatches_old_queue(tmp_path):
    store = SQLiteStore(tmp_path / 'conversation.sqlite', initial_state())
    session = Session(store)
    session.enqueue('user', 'old', 'do not replay this')
    store.update(lambda state: (state.update(native_session='native-id'), []))
    session.close()
    session.reopen()
    state = store.snapshot()['state']
    assert state['native_session'] == 'native-id'
    assert not state['closed'] and not state['connected']
    assert state['messages'][0]['status'] == 'withdrawn'
    assert session.take() is None
    store.close()


def test_continuing_resources_keeps_the_original_authentication(resources, tmp_path, monkeypatch, proxy_address, agent_cli_ok):
    request, calls, machine = resources
    request = replace(request, project=tmp_path / 'project')
    live.open_robot(request).power_off()
    facts = live_state.load(request.name, request.home, require_running=False)
    facts['auth_mode'] = 'api'
    live_state.save(request.name, request.home, **facts)
    monkeypatch.setattr(live.agents, 'api_key_file', lambda *a, **k: pytest.fail('do not replace retained authentication'))
    before = len([c for c in calls if not isinstance(c, str) and c[0] == 'profile'])
    robot = live.open_robot(replace(request, continuing=True))
    after = live_state.load(request.name, request.home)
    assert after['auth_mode'] == 'api'
    assert after['mounts'] == facts['mounts']
    assert len([c for c in calls if not isinstance(c, str) and c[0] == 'profile']) == before
    robot.power_off()


def test_new_conversation_and_resumed_conversation_share_project_with_real_children(tmp_path, monkeypatch):
    async def run():
        home, root = tmp_path / 'home', tmp_path / 'project'
        root.mkdir()
        program = tmp_path / 'native.py'
        program.write_text('''
import json, subprocess, sys
from pathlib import Path
root, identity = Path(sys.argv[1]), sys.argv[2]
def emit(kind, turn=None, **data):
    print(json.dumps(dict(kind=kind, turn_id=turn, data=data)), flush=True)
for line in sys.stdin:
    frame = json.loads(line)
    if 'begin' in frame:
        emit('ready', session_id=identity)
        continue
    turn = frame['submit']
    emit('turn_started', turn)
    if frame['text'] == 'write':
        (root / 'program.py').write_text('print("saved by the first conversation")\\n')
        (root / 'notes.md').write_text('Run program.py to continue.\\n')
        text = 'written'
    else:
        assert 'program.py' in (root / 'notes.md').read_text()
        text = subprocess.check_output([sys.executable, str(root / 'program.py')], text=True)
    emit('text_delta', turn, item_id=turn, text=text)
    emit('turn_finished', turn, status='completed')
''')
        identities = []
        def resources(request, **kwargs):
            live_state.save(request.name, home, status='running', project=str(root), workspace=str(root),
                            agent='fixture', sandbox=request.name, model='fixture', proxy='', options={})
            return SimpleNamespace(power_off=lambda: live_state.stopped(request.name, home), stopped=lambda: None)
        def conversation(sandbox, model, proxy, **kwargs):
            identity = kwargs.get('session_id', 'native-' + sandbox)
            identities.append(identity)
            return Conversation([sys.executable, '-u', str(program), str(root), identity], FakeProtocol())
        monkeypatch.setattr(managed, 'open_robot', resources)
        monkeypatch.setattr(managed.agents, 'get', lambda *a, **k: SimpleNamespace(conversation=conversation))
        for name, text, continuing in [('first', 'write', False), ('second', 'read', False), ('first', 'read', True)]:
            owner = await managed.start(live.RobotRequest(home=home, name=name, project=root, continuing=continuing))
            try:
                await until(lambda: owner.store.snapshot()['state']['connected'])
                if name == 'first' and not continuing:
                    with pytest.raises(Exception, match='already has a resource owner'):
                        await managed.start(live.RobotRequest(home=home, name=name, project=root, continuing=True))
                message = await owner.execution.command('enqueue', client_id='user', request_id=text, text=text)
                await until(lambda: next(m for m in owner.store.snapshot()['state']['messages']
                                        if m['id'] == message['id'])['status'] == 'completed')
                if text == 'read':
                    assert any('saved by the first conversation' in str(event)
                               for event in owner.store.events(0))
                await owner.end()
            finally:
                await owner.end()
                owner.store.close()
        assert identities == ['native-first', 'native-second', 'native-first']
        assert {r['id'] for r in history.entries(home, root)} == {'first', 'second'}
        assert (root / 'program.py').is_file()
    asyncio.run(run())


def test_empty_native_conversations_do_not_require_a_persisted_rollout(tmp_path):
    state = initial_state()
    state.update(closed=True, native_session='unused-native-thread')
    store = SQLiteStore(tmp_path / 'empty.sqlite', state)
    Session(store).reopen()
    assert store.snapshot()['state']['native_session'] is None
    assert store.snapshot()['state']['messages'] == []
    store.close()
