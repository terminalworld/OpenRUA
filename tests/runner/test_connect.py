"""Connecting a robot: the folder, the checks and the move, without docker."""

import asyncio
import contextlib
import io
from types import SimpleNamespace

import pytest
import yaml

from openrua.agents.conversation import Event
from openrua.runner import connect
from openrua.sessions.client import Client
from openrua.sessions.core import Session, initial_state
from openrua.sessions.store import SQLiteStore
from tests.sessions.test_execution import setup, until
from tests.sessions.test_http import running_server, stop

FAKE_ROBOT = """machine:
  backend:
    kind: real
    ros_distro: jazzy
    launch: python3 driver/up.py   # stops on SIGINT
    discovery:
      network: host
    build:
      fake-driver: driver
  robot:
    model: Fake arm
    description: two joints that only report where they are
  cameras:
    list: []
"""


def test_the_folder_starts_with_what_connecting_needs_and_keeps_what_is_there(tmp_path):
    folder = tmp_path / "arm"
    connect.scaffold(folder)
    for name in ("robot.yaml", "CONNECT.md", "driver", "cameras"):
        assert (folder / name).exists()
    (folder / "CONNECT.md").write_text("User connection notes\n")
    assert "TODO" in (folder / "robot.yaml").read_text()
    (folder / "robot.yaml").write_text(FAKE_ROBOT)
    connect.scaffold(folder)
    assert (folder / "robot.yaml").read_text() == FAKE_ROBOT
    assert (folder / "CONNECT.md").read_text() == "User connection notes\n"


def checks(tmp_path, robot=FAKE_ROBOT):
    folder = tmp_path / "arm"
    connect.scaffold(folder)
    (folder / "robot.yaml").write_text(robot)
    (folder / "driver/Dockerfile").write_text("FROM scratch\n")
    session = Session(SQLiteStore(tmp_path / "s.sqlite", initial_state()))
    request = connect.ConnectRequest(home=tmp_path / "home", name="c", folder=folder, project=folder)
    return connect.Checks(request, session, "claude-code"), session


def test_config_passes_only_without_a_todo_outside_comments(tmp_path):
    done, _ = checks(tmp_path)
    with open(tmp_path / "log", "w") as out:
        ok, summary = done._config(out)
    assert ok and "Fake arm" in summary
    done, _ = checks(tmp_path / "todo", FAKE_ROBOT.replace("model: Fake arm", "model: TODO"))
    with open(tmp_path / "log", "w") as out:
        ok, summary = done._config(out)
    assert not ok and summary.startswith("1 TODO left") and "robot.yaml:" in summary


@pytest.mark.parametrize('discovery', [
    {'discovery_server': '192.0.2.10:11811'},
    {'static_peers': ['192.0.2.11', '192.0.2.12']},
    {'network': 'host'},
])
@pytest.mark.parametrize('custom_image', [None, 'robot-tools:tested'])
def test_probe_uses_project_connection_even_before_robot_facts_are_complete(
        tmp_path, monkeypatch, discovery, custom_image):
    from openrua.cli import build_parser
    from openrua.cli.commands import probe
    from openrua.config.schema import sandbox_image

    backend = {'kind': 'real', 'ros_distro': 'humble', 'discovery': discovery,
               'launch': 'TODO', 'sandbox_install': 'apt-get install robot-messages'}
    if custom_image:
        backend['sandbox_image'] = custom_image
    done, _ = checks(tmp_path, yaml.safe_dump({
        'machine': {'backend': backend, 'robot': {'model': 'TODO'},
                    'cameras': {'list': [{'name': 'camera', 'type': 'not-written-yet.yaml'}]}}}))
    started, stopped = [], []
    monkeypatch.setattr(probe, 'sandbox_up', lambda *a, **kw: started.append(kw))
    monkeypatch.setattr(probe, 'sandbox_down', stopped.append)

    def graph(run):
        settings = started[-1]
        env = dict(item.split('=', 1) for item in settings['env'])
        observed = ({'discovery_server': env['ROS_DISCOVERY_SERVER']}
                    if env.get('ROS_DISCOVERY_SERVER') else
                    {'static_peers': settings['static_peer'].split(',')}
                    if settings.get('static_peer') else {'network': 'host'})
        # Each discovery path represents a distinct live graph.
        name = 'selected_robot' if observed == discovery else 'unrelated_robot'
        return {'topics': {'/joint_states': 'sensor_msgs/msg/JointState'},
                'actions': {}, 'services': [], 'urdf': {'name': name}}

    monkeypatch.setattr(probe, 'read_graph', graph)

    def command(argv, stdout, stderr):
        args = build_parser().parse_args(argv[3:])
        with contextlib.redirect_stdout(stdout):
            return args.fn(args)

    monkeypatch.setattr(connect.subprocess, 'call', command)
    ok, summary = done._probe(io.StringIO())
    assert ok, summary
    draft = yaml.safe_load((done.folder / 'robot.draft.yaml').read_text())['machine']
    assert draft['robot']['model'] == 'selected_robot'
    assert draft['backend']['discovery'] == discovery
    assert draft['backend']['ros_distro'] == 'humble'
    assert started[0]['image'] == (custom_image or sandbox_image('humble', backend['sandbox_install']))
    assert 'review' in summary.lower()
    assert stopped == [started[0]['name']]


@pytest.mark.parametrize('invalid', [
    {'discovery': {}},
    {'discovery': {'network': 'host', 'discovery_server': '192.0.2.10:11811'}},
    {'ros_distro': 'TODO'},
    {'ros_dist': 'humble'},
])
def test_probe_rejects_unknown_connection_instead_of_using_a_default_graph(
        tmp_path, monkeypatch, invalid):
    from openrua.errors import ConfigError
    data = yaml.safe_load(FAKE_ROBOT)
    data['machine']['backend'].update(invalid)
    done, _ = checks(tmp_path, yaml.safe_dump(data))
    called = []
    monkeypatch.setattr(connect.subprocess, 'call', lambda *a, **kw: called.append(a) or 0)
    with pytest.raises(ConfigError):
        done._probe(io.StringIO())
    assert not called


@pytest.mark.parametrize('build_succeeds', [True, False])
def test_robot_checks_prepare_an_existing_different_distro_image_missing_the_agent(tmp_path, monkeypatch, build_succeeds):
    import io
    done, _ = checks(tmp_path)
    done.agent = 'codex'
    backend = {'ros_distro': 'humble', 'sandbox_image': 'openrua-sandbox-humble'}
    commands = []
    monkeypatch.setattr(connect.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=0))
    def ready(image, command):
        assert image == 'openrua-sandbox-humble' and 'codex' in command
        return bool(commands) and build_succeeds
    monkeypatch.setattr(connect, 'cli_ready', ready)
    def build(out, *argv):
        commands.append(argv)
        return 0 if build_succeeds else 1
    monkeypatch.setattr(done, '_openrua', build)
    assert done._images(io.StringIO(), backend) == ([] if build_succeeds else ['openrua-sandbox-humble'])
    assert commands == [('build', 'sandbox', '--distro', 'humble', '--tag', 'openrua-sandbox-humble',
                         '--base-image', 'openrua-sandbox-humble', '--agent', 'codex')]


def test_a_check_runs_on_the_host_and_its_result_reaches_the_session(tmp_path, monkeypatch):
    done, session = checks(tmp_path)
    monkeypatch.setattr(connect, "host_report", lambda: (True, "docker: usable"))
    started = done.start("host")
    assert started["status"] == "running"
    for _ in range(200):
        result = session.store.snapshot()["state"]["checks"]["host"]
        if result["status"] != "running":
            break
        asyncio.run(asyncio.sleep(0.01))
    assert result == {"status": "passed", "summary": "docker: usable", "run": started["run"]}
    assert (tmp_path / "arm/.openrua/connect/c/host.log").read_text() == "docker: usable\n"
    with pytest.raises(ValueError, match="unknown step"):
        done.start("everything")


@pytest.mark.parametrize('host_ok', [True, False])
def test_connection_records_the_host_check_before_reporting_and_keeps_its_run(tmp_path, monkeypatch, host_ok):
    async def run():
        done, session = checks(tmp_path)
        commands, running = [], []

        async def command(operation, **params):
            commands.append((operation, params))

        flow = connect.ConnectFlow.__new__(connect.ConnectFlow)
        flow.request, flow.folder = done.request, done.folder
        flow.owned = SimpleNamespace(execution=SimpleNamespace(session=session, command=command))
        flow._task = None
        service = connect.paths.sandbox_dir(done.request.name, done.request.home) / 'connect'
        service.mkdir(parents=True)

        def report():
            current = session.store.snapshot()['state']['checks']['host']
            assert current['status'] == 'running' and current['run']
            running.append(current['run'])
            return host_ok, 'host fixture result'

        monkeypatch.setattr(connect, 'host_report', report)
        try:
            flow.prepare()
            await flow.begin(SimpleNamespace(url='http://127.0.0.1:1234', token='fixture'))
            current = session.store.snapshot()['state']['checks']['host']
            assert current == {'status': 'passed' if host_ok else 'failed',
                               'summary': 'host fixture result', 'run': running[0]}
            assert commands[0][0] == 'enqueue'
            assert 'host fixture result' in commands[0][1]['text']
            assert (connect.check_directory(done.request) / 'host.log').read_text() == 'host fixture result\n'
        finally:
            await flow.close()
            if flow._task is not None:
                await asyncio.gather(flow._task, return_exceptions=True)
            session.store.close()
    asyncio.run(run())


def test_done_names_what_has_not_passed_yet(tmp_path):
    done, session = checks(tmp_path)
    assert done.start("done")["status"] == "failed"
    for step in connect.STEPS:
        session.checked(step, "passed", "", "r")
    assert done.start("done")["status"] == "passed"


def test_the_session_keeps_each_checks_result_and_a_planned_handover_pauses_nothing(tmp_path):
    session = Session(SQLiteStore(tmp_path / "s.sqlite", initial_state()))
    session.native(Event("ready", data={"session_id": "native"}))
    session.checked("config", "failed", "1 TODO left", "r1")
    assert session.store.snapshot()["state"]["checks"]["config"]["status"] == "failed"
    with pytest.raises(ValueError):
        session.checked("config", "maybe")
    session.handed_over("moving to the robot's own terminal")
    state = session.store.snapshot()["state"]
    assert not state["connected"] and not state["paused"]


def test_an_owner_action_answers_on_its_own_path(tmp_path):
    async def run():
        asked = []
        async def check(body):
            asked.append(body)
            return {"status": "running", "run": "r"}
        store, session, native, execution, server = await running_server(
            tmp_path, actions={"check": check})
        client = Client(server.url, server.token)
        try:
            answer = await asyncio.to_thread(client.request, "/api/actions/check", {"step": "host"})
            assert answer["result"] == {"status": "running", "run": "r"} and asked == [{"step": "host"}]
            with pytest.raises(RuntimeError, match="unknown endpoint"):
                await asyncio.to_thread(client.request, "/api/actions/other", {})
        finally:
            await stop(store, execution, server)
    asyncio.run(run())


def test_closing_for_a_handover_records_no_lost_turn(tmp_path):
    async def run():
        store, session, native, execution = await setup(tmp_path)
        await execution.close(handover="moving")
        state = store.snapshot()["state"]
        assert not state["paused"] and state["pause_reason"] is None
        store.close()
    asyncio.run(run())


def test_done_asks_the_flow_to_move_once_the_turn_ends(tmp_path, monkeypatch):
    done, session = checks(tmp_path)
    for step in connect.STEPS:
        session.checked(step, "passed", "", "r")
    flow = connect.ConnectFlow.__new__(connect.ConnectFlow)
    flow.checks, flow._move = done, False
    result = asyncio.run(flow.check({"step": "done"}))
    assert result["status"] == "passed" and flow._move


def test_connecting_hands_the_robot_to_the_selected_project(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from openrua.config import projects
    async def run():
        root = tmp_path / 'project'
        folder = root / 'robot'
        connect.scaffold(folder)
        (folder / 'robot.yaml').write_text(FAKE_ROBOT)
        (folder / 'driver/Dockerfile').write_text('FROM scratch\n')
        flow = connect.ConnectFlow.__new__(connect.ConnectFlow)
        flow.request = connect.ConnectRequest(home=tmp_path / 'home', name='connecting', folder=folder, project=root)
        flow.folder, flow.agent = folder, 'codex@0.134.0'
        connect.state.save('connecting', flow.request.home, agent='codex', agent_version='0.134.0', permissions='ask')
        requests = []
        monkeypatch.setattr(connect, 'open_robot', lambda request, **kw: requests.append(request))
        monkeypatch.setattr(connect.subprocess, 'call', lambda *a, **k: 0)
        async def move(factory, reason):
            factory()
        async def say(text):
            pass
        flow.owned = SimpleNamespace(move_to=move, execution=SimpleNamespace(session=SimpleNamespace(robot_status=lambda *a, **kw: None)), maintenance=None,
                                     store=SimpleNamespace(snapshot=lambda: {'state': {'settings': {'model': 'chosen', 'permissions': 'ask'}}}))
        flow.server, flow._say = SimpleNamespace(), say
        await flow._move_to_robot()
        assert requests[0].project == root and requests[0].continuing
        assert projects.selection(root)['robot'] == str(folder / 'robot.yaml')
        assert projects.selection(root)['agent'] == 'codex@0.134.0'
    asyncio.run(run())


def test_connect_keeps_the_project_workspace_from_the_first_turn(tmp_path, monkeypatch):
    from pathlib import Path
    from openrua.runner import live_state
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir()
    (root / 'program.py').write_text('print("existing project")\n')
    request = connect.ConnectRequest(home=home, name='onboarding', folder=root / 'robot', project=root)
    class DockerSocket:
        def exists(self): return True
        def stat(self): return SimpleNamespace(st_gid=100)
        def __str__(self): return '/var/run/docker.sock'
    socket = DockerSocket()
    def path(value):
        return socket if str(value) == '/var/run/docker.sock' else Path(value)
    path.home = Path.home
    monkeypatch.setattr(connect, 'Path', path)
    monkeypatch.setattr(connect.shutil, 'which', lambda binary: '/usr/bin/docker')
    adapter = SimpleNamespace(name='fixture', title='Fixture', default_options={}, default_permissions='ask', permission_modes=('ask',),
                              prepare_profile=lambda *a, **kw: SimpleNamespace(mounts=()))
    monkeypatch.setattr(connect, '_agent', lambda r: (adapter, {}, 'fixture-model',
                                                     SimpleNamespace(sandbox=SimpleNamespace(run_args=[]))))
    monkeypatch.setattr(connect.agents, 'resolve_profile', lambda *a, **kw: tmp_path / 'credentials')
    monkeypatch.setattr(connect, 'check_agent_cli', lambda *a: None)
    launched = []
    monkeypatch.setattr(connect, 'sandbox_up', lambda cfg, directory, **kw: launched.append((directory, kw)))
    monkeypatch.setattr(connect, 'sandbox_down', lambda *a: None)
    resource = connect.open_connect(request)
    try:
        facts = live_state.load(request.name, home)
        assert launched[0][0] == root
        assert facts['workspace'] == facts['project'] == str(root)
        assert facts['robot'] == str(root / 'robot/robot.yaml')
        assert '/workspace/robot/CONNECT.md' in connect.opening_message(root / 'robot', 'fixture', root)
        assert (root / 'program.py').read_text() == 'print("existing project")\n'
        assert not (home / 'robots').exists()
    finally:
        resource.power_off()
