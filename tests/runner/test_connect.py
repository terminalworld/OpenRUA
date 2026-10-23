"""Connecting a robot: the folder, the checks and the move, without docker."""

import asyncio
from types import SimpleNamespace

import pytest

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
    for name in ("robot.yaml", "CONNECT.md", "driver", "cameras", connect.CHECKS_DIR):
        assert (folder / name).exists()
    assert str(folder) in (folder / "CONNECT.md").read_text()
    assert "TODO" in (folder / "robot.yaml").read_text()
    (folder / "robot.yaml").write_text(FAKE_ROBOT)
    connect.scaffold(folder)
    assert (folder / "robot.yaml").read_text() == FAKE_ROBOT
    assert (folder / ".gitignore").read_text().splitlines().count(f"{connect.CHECKS_DIR}/") == 1


def checks(tmp_path, robot=FAKE_ROBOT):
    folder = tmp_path / "arm"
    connect.scaffold(folder)
    (folder / "robot.yaml").write_text(robot)
    session = Session(SQLiteStore(tmp_path / "s.sqlite", initial_state()))
    request = connect.ConnectRequest(home=tmp_path / "home", name="c", folder=folder)
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
    assert commands == [('build', 'sandbox', '--distro', 'humble', '--tag', 'openrua-sandbox-humble', '--agent', 'codex')]


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
    assert (tmp_path / "arm" / connect.CHECKS_DIR / "host.log").read_text() == "docker: usable\n"
    with pytest.raises(ValueError, match="unknown step"):
        done.start("everything")


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
        root, folder = tmp_path / 'project', tmp_path / 'robot'
        root.mkdir(); folder.mkdir()
        flow = connect.ConnectFlow.__new__(connect.ConnectFlow)
        flow.request = connect.ConnectRequest(home=tmp_path / 'home', name='connecting', folder=folder, project=root)
        flow.folder, flow.agent = folder, 'codex@0.134.0'
        requests = []
        monkeypatch.setattr(connect, 'open_robot', lambda request: requests.append(request))
        monkeypatch.setattr(connect.subprocess, 'call', lambda *a, **k: 0)
        async def move(factory, reason):
            factory()
        async def say(text):
            pass
        flow.owned = SimpleNamespace(move_to=move, execution=object(),
                                     store=SimpleNamespace(snapshot=lambda: {'state': {'settings': {'model': 'chosen', 'permissions': 'ask'}}}))
        flow.server, flow._say = SimpleNamespace(), say
        await flow._move_to_robot()
        assert requests[0].project == root and requests[0].continuing
        assert projects.selection(root)['robot'] == str(folder / 'robot.yaml')
        assert projects.selection(root)['agent'] == 'codex@0.134.0'
    asyncio.run(run())
