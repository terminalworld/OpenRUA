"""A robot's cameras from the host: their topics, the tap's stream, the
view for a person watching, and the recording a real robot's run keeps."""

import asyncio
import io
import json
from types import SimpleNamespace

import pytest

from openrua.robot import cameras
from openrua.sessions.client import Client
from tests.sessions.test_http import running_server, stop

CFG = {"machine": {"cameras": {
    "types": {"cam": {"model": "C", "topics": {"color": "/{name}/color/image_raw"}}},
    "list": [{"name": "wrist", "type": "cam"}, {"name": "scene", "type": "cam"}]}}}


def test_named_cameras_and_a_scene_that_decides_its_own():
    assert cameras.camera_topics(CFG) == (
        {"wrist": "/wrist/color/image_raw", "scene": "/scene/color/image_raw"}, None)
    scene = {"machine": {"cameras": {"types": {"openrua-sim": {
        "model": "sim", "topics": {"color": "/{name}/color/image_raw"}}}, "list": None}}}
    assert cameras.camera_topics(scene) == ({}, "/{name}/color/image_raw")
    assert cameras.camera_topics({"machine": {}}) == ({}, None)


def _stream(*ticks) -> bytes:
    out = json.dumps({"t": 1.0, "cameras": {"wrist": "/w"}, "frames": []}).encode() + b"\n"
    for t, frames in ticks:
        out += json.dumps({"t": t, "frames": [{"name": n, "mime": "image/jpeg", "bytes": len(d)}
                                              for n, d in frames]}).encode() + b"\n"
        out += b"".join(d for _, d in frames)
    return out


def test_the_tap_keeps_the_latest_frame_and_records_every_tick(tmp_path):
    tap = cameras.CameraTap("box", {"wrist": "/w", "scene": "/s"}, record=tmp_path / "frames")
    (tmp_path / "frames").mkdir()
    tap._proc = SimpleNamespace(stdout=io.BytesIO(_stream(
        (2.0, [("wrist", b"A1"), ("scene", b"B1")]), (2.5, [("wrist", b"A2")]))))
    tap._read()
    assert tap.latest["wrist"]["data"] == b"A2" and tap.latest["scene"]["data"] == b"B1"
    index = [json.loads(line) for line in (tmp_path / "frames" / "index.jsonl").read_text().splitlines()]
    # the layout openrua demo reads: one line per tick, numbered from 0, with its wall time
    assert index == [{"step": 0, "t": 2.0, "files": ["000000_wrist.jpg", "000000_scene.jpg"]},
                     {"step": 1, "t": 2.5, "files": ["000001_wrist.jpg"]}]
    assert (tmp_path / "frames" / "000001_wrist.jpg").read_bytes() == b"A2"


def test_the_view_reads_only_the_camera_watched(monkeypatch):
    started = []

    class Tap:
        def __init__(self, sandbox, cams, fps):
            self.sandbox, self.cameras, self.latest = sandbox, cams, {}
            import threading
            self.lock = threading.Lock()
            self.running = True
        def start(self):
            started.append(sorted(self.cameras))
            self.latest = {n: {"t": 0.0, "mime": "image/jpeg", "data": b"x"} for n in self.cameras}
        def stop(self):
            self.running = False
        def status(self):
            return {"status": "recording", "errors": []}

    monkeypatch.setattr(cameras, "CameraTap", Tap)
    robot = SimpleNamespace(sandbox="box", cameras=({"wrist": "/w", "scene": "/s"}, None),
                            connection=SimpleNamespace(generation=0))
    view = cameras.CameraView(lambda: robot)
    try:
        assert view.names() == ["scene", "wrist"]
        assert view.frame("wrist")["data"] == "eA=="
        view.frame("wrist")
        view.frame("scene")
        assert started == [["wrist"], ["scene"]]   # one at a time, switched on request
        robot.connection.generation += 1
        view.frame('scene')
        assert started == [['wrist'], ['scene'], ['scene']]
        with pytest.raises(ValueError, match="no camera named 'door'"):
            view.frame("door")
    finally:
        view.close()


def test_the_session_serves_camera_names_and_frames(tmp_path):
    class View:
        def names(self):
            return ["wrist"]
        def frame(self, name):
            return {"name": name, "waiting": True}

    async def run():
        store, session, native, execution, server = await running_server(tmp_path, cameras=View())
        client = Client(server.url, server.token)
        try:
            assert await asyncio.to_thread(client.request, "/api/cameras") == {"cameras": ["wrist"]}
            assert await asyncio.to_thread(client.request, "/api/cameras/frame?name=wrist") == \
                {"name": "wrist", "waiting": True}
        finally:
            await stop(store, execution, server)
    asyncio.run(run())


def test_a_real_robots_run_is_recorded_unless_told_not_to():
    from openrua.cli.commands.execute import recording
    args = lambda record=None, no_record=False: SimpleNamespace(record=record, no_record=no_record)
    assert recording(args(), real=True) == ()            # all cameras: a real run is not replayed
    assert recording(args(), real=False) is None         # a simulated one can be, so opt in
    assert recording(args("wrist, scene"), real=False) == ("wrist", "scene")
    assert recording(args(no_record=True), real=True) is None


def test_native_stdout_diagnostics_cannot_corrupt_the_camera_protocol():
    import subprocess
    import sys
    child = subprocess.run([sys.executable, '-c', '''
import os
from openrua.robot.camera_tap import protocol_output
with protocol_output() as output:
    os.write(1, b'native DDS diagnostic\\n')
    output.write(b'{"cameras":{"front":"/front/color/image_raw"}}\\n')
'''], capture_output=True, check=True)
    assert json.loads(child.stdout)['cameras'] == {'front': '/front/color/image_raw'}
    assert child.stderr == b'native DDS diagnostic\n'


@pytest.mark.parametrize('code,stdout,stderr,match', [
    (1, b'', b'ROS setup failed', 'ROS setup failed'),
    (0, b'native diagnostic\n{}\n', b'protocol failed', 'no valid camera record'),
])
def test_camera_discovery_reports_the_actual_error(monkeypatch, code, stdout, stderr, match):
    monkeypatch.setattr(cameras.subprocess, 'run',
                        lambda *a, **k: SimpleNamespace(returncode=code, stdout=stdout, stderr=stderr))
    with pytest.raises(RuntimeError, match=match):
        cameras.list_cameras('box', '/{name}/color/image_raw')


def test_camera_process_reads_the_same_shell_setup_without_polluting_protocol(tmp_path, monkeypatch):
    import os
    import subprocess

    setup = tmp_path / 'robot-setup.sh'
    setup.write_text('export CAMERA_OVERLAY=ready\nprintf "setup diagnostic\\n"\n')
    script = tmp_path / 'camera.py'
    script.write_text('import json, os, sys\nprint(json.dumps({"overlay": os.environ["CAMERA_OVERLAY"], '
                      '"discovery": os.environ["ROS_DISCOVERY_SERVER"], '
                      '"super_client": os.environ["ROS_SUPER_CLIENT"], "args": sys.argv[1:]}))\n')
    monkeypatch.setattr(cameras, 'TAP', script)
    arg = '--camera=wrist=/camera with spaces; $(printf unwanted)'
    command = cameras._command('box', arg)
    result = subprocess.run(command[command.index('sh'):],
                            env={**os.environ, 'BASH_ENV': str(setup),
                                 'ROS_DISCOVERY_SERVER': '192.0.2.20:11811', 'ROS_SUPER_CLIENT': 'TRUE'},
                            capture_output=True, check=True)
    assert json.loads(result.stdout) == {'overlay': 'ready', 'args': [arg],
                                        'discovery': '192.0.2.20:11811', 'super_client': 'TRUE'}
    assert result.stderr == b'setup diagnostic\n'


def test_a_truncated_frame_keeps_prior_complete_frames_and_reports_failure(tmp_path):
    tap = cameras.CameraTap('box', {'wrist': '/w'}, record=tmp_path)
    stream = _stream((2.0, [('wrist', b'complete')]), (3.0, [('wrist', b'incomplete')]))[:-3]
    tap._proc = SimpleNamespace(stdout=io.BytesIO(stream))
    tap._read()
    status = tap.status()
    assert status['status'] == 'failed'
    assert 'received 7 of 10 bytes' in status['errors'][0]
    assert status['cameras']['wrist'] == {'count': 1, 'first_t': 2.0, 'last_t': 2.0}
    assert tap.latest['wrist']['data'] == b'complete'
    assert list(tmp_path.glob('*.jpg')) == [tmp_path / '000000_wrist.jpg']
    assert len((tmp_path / 'index.jsonl').read_text().splitlines()) == 1
    with pytest.raises(RuntimeError, match='incomplete frame'):
        tap.wait_for_frames(10)


@pytest.mark.parametrize('header', [b'not json\n', b'[]\n',
                                  b'{"frames":[{"name":"wrist","mime":"image/jpeg","bytes":-1}]}\n'])
def test_invalid_camera_protocol_is_reported(header):
    tap = cameras.CameraTap('box', {'wrist': '/w'})
    tap._proc = SimpleNamespace(stdout=io.BytesIO(header))
    tap._read()
    assert tap.status()['status'] == 'failed'
    assert tap.status()['missing_cameras'] == ['wrist']


def test_camera_stderr_is_preserved_with_a_bounded_status_tail(tmp_path):
    tap = cameras.CameraTap('box', {'wrist': '/w'}, record=tmp_path)
    diagnostic = b'X' * 24000 + b'\nROS setup failed\n'
    tap._read_errors(SimpleNamespace(stderr=io.BytesIO(diagnostic)))
    assert (tmp_path / 'camera.log').read_bytes() == diagnostic
    assert len(tap.status()['stderr'].encode()) <= 8192
    assert tap.status()['stderr'].endswith('ROS setup failed')


def _local_tap(monkeypatch, tmp_path, source, cameras_to_read=None):
    import subprocess

    script = tmp_path / 'camera.py'
    script.write_text(source)
    monkeypatch.setattr(cameras, 'TAP', script)
    original = subprocess.Popen
    def local(command, **kwargs):
        assert command[:2] == ['docker', 'exec']
        return original(command[command.index('sh'):], **kwargs)
    monkeypatch.setattr(cameras.subprocess, 'Popen', local)
    return cameras.CameraTap('box', cameras_to_read or {'wrist': '/w'}, record=tmp_path / 'frames')


def test_normal_camera_stop_reports_missing_frames_without_changing_task_policy(monkeypatch, tmp_path):
    tap = _local_tap(monkeypatch, tmp_path, 'import sys\nsys.stdin.buffer.read()\n')
    tap.start()
    assert tap.status()['status'] == 'recording'
    tap.stop()
    status = tap.status()
    assert status['status'] == 'incomplete'
    assert status['missing_cameras'] == ['wrist']
    assert not status['errors']
    assert status['stopped_at'] >= status['started_at']


def test_clean_camera_stop_retains_per_camera_first_and_last_frame_times(monkeypatch, tmp_path):
    import time

    stream = _stream((2.0, [('wrist', b'one')]), (3.0, [('wrist', b'two')]))
    tap = _local_tap(monkeypatch, tmp_path,
                     f'import sys\nsys.stdout.buffer.write({stream!r})\nsys.stdout.buffer.flush()\n'
                     'sys.stdin.buffer.read()\n')
    tap.start()
    deadline = time.monotonic() + 3
    while tap.ticks < 2 and time.monotonic() < deadline:
        time.sleep(0.01)
    tap.stop()
    status = tap.status()
    assert status['status'] == 'completed'
    assert status['cameras'] == {'wrist': {'count': 2, 'first_t': 2.0, 'last_t': 3.0}}
    assert status['missing_cameras'] == []
    assert not tap.running


def test_recording_wait_requires_a_complete_saved_frame_from_every_camera(monkeypatch, tmp_path):
    import threading

    initial = _stream((2.0, [('wrist', b'one')]), (2.5, [('scene', b'complete')]))
    tap = _local_tap(monkeypatch, tmp_path,
                     f'import sys\nsys.stdout.buffer.write({initial[:-3]!r})\nsys.stdout.buffer.flush()\n'
                     'sys.stdin.buffer.readline()\n'
                     f'sys.stdout.buffer.write({initial[-3:]!r})\nsys.stdout.buffer.flush()\n'
                     'sys.stdin.buffer.read()\n', {'wrist': '/w', 'scene': '/s'})
    finished = threading.Event()
    failures = []

    def wait():
        try:
            tap.wait_for_frames(3)
        except Exception as exc:
            failures.append(exc)
        finally:
            finished.set()

    tap.start()
    waiter = threading.Thread(target=wait)
    try:
        waiter.start()
        with tap._changed:
            assert tap._changed.wait_for(lambda: 'wrist' in tap._received, timeout=3)
        assert (tmp_path / 'frames/000000_wrist.jpg').read_bytes() == b'one'
        assert not (tmp_path / 'frames/000001_scene.jpg').exists()
        assert not finished.wait(0.05)
        tap._proc.stdin.write(b'finish frame\n')
        tap._proc.stdin.flush()
        assert finished.wait(3)
        assert not failures
        assert (tmp_path / 'frames/000001_scene.jpg').read_bytes() == b'complete'
        assert len((tmp_path / 'frames/index.jsonl').read_text().splitlines()) == 2
    finally:
        tap.stop()
        waiter.join(timeout=3)


def test_recording_wait_timeout_names_missing_camera_and_keeps_ownership(monkeypatch, tmp_path):
    stream = _stream((2.0, [('wrist', b'one')]))
    tap = _local_tap(monkeypatch, tmp_path,
                     f'import sys\nsys.stdout.buffer.write({stream!r})\nsys.stdout.buffer.flush()\n'
                     'sys.stdin.buffer.read()\n', {'wrist': '/w', 'scene': '/s'})
    tap.start()
    try:
        with tap._changed:
            assert tap._changed.wait_for(lambda: 'wrist' in tap._received, timeout=3)
        with pytest.raises(TimeoutError, match='No saved frame from scene after 0.01 seconds') as error:
            tap.wait_for_frames(0.01)
        assert str(tmp_path / 'frames/camera.log') in str(error.value)
        assert tap.running
    finally:
        tap.stop()


def test_recording_wait_reports_process_failure_without_waiting_for_the_deadline(monkeypatch, tmp_path):
    import time

    tap = _local_tap(monkeypatch, tmp_path,
                     'import sys\nsys.stderr.write("ROS setup failed\\n")\nraise SystemExit(4)\n')
    tap.start()
    try:
        started = time.monotonic()
        with pytest.raises(RuntimeError, match='Camera recording could not start'):
            tap.wait_for_frames(10)
        assert time.monotonic() - started < 3
    finally:
        tap.stop()
    assert 'ROS setup failed' in tap.status()['stderr']


def test_a_failed_index_write_cannot_satisfy_the_recording_wait(monkeypatch, tmp_path):
    from pathlib import Path

    original = Path.open

    class FailedIndex(io.StringIO):
        def flush(self):
            raise OSError('disk full')

    monkeypatch.setattr(Path, 'open', lambda path, *a, **kw:
                        FailedIndex() if path.name == 'index.jsonl' else original(path, *a, **kw))
    tap = cameras.CameraTap('box', {'wrist': '/w'}, record=tmp_path)
    tap._proc = SimpleNamespace(stdout=io.BytesIO(_stream((2.0, [('wrist', b'one')]))), poll=lambda: None)
    tap._read()
    assert (tmp_path / '000000_wrist.jpg').read_bytes() == b'one'
    assert tap.status()['missing_cameras'] == ['wrist']
    with pytest.raises(RuntimeError, match='disk full'):
        tap.wait_for_frames(10)


@pytest.mark.parametrize('timeout', [0, -1, float('inf'), float('nan'), True])
def test_recording_wait_rejects_an_invalid_deadline(tmp_path, timeout):
    tap = cameras.CameraTap('box', {'wrist': '/w'}, record=tmp_path)
    with pytest.raises(ValueError, match='positive number'):
        tap.wait_for_frames(timeout)


def test_recording_wait_remains_interruptible_in_the_main_thread(tmp_path):
    import subprocess
    import sys

    result = subprocess.run([sys.executable, '-c', '''
import os, signal, sys, threading, time
from pathlib import Path
from types import SimpleNamespace
from openrua.robot.cameras import CameraTap

tap = CameraTap('box', {'front': '/front'}, record=Path(sys.argv[1]))
tap._proc = SimpleNamespace(poll=lambda: None)
timer = threading.Timer(0.05, lambda: os.kill(os.getpid(), signal.SIGINT))
started = time.monotonic()
timer.start()
try:
    tap.wait_for_frames(30)
except KeyboardInterrupt:
    assert time.monotonic() - started < 3
else:
    raise AssertionError('The recording wait ignored the interrupt')
finally:
    timer.join()
''', str(tmp_path)], capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr.decode()


def test_camera_exit_retains_native_error_and_cannot_be_reported_complete(monkeypatch, tmp_path):
    tap = _local_tap(monkeypatch, tmp_path,
                     'import sys\nsys.stderr.write("ROS setup failed\\n")\nraise SystemExit(4)\n')
    tap.start()
    tap._proc.wait(timeout=3)
    tap._reader.join(timeout=3)
    tap.stop()
    status = tap.status()
    assert status['status'] == 'failed'
    assert 'ROS setup failed' in status['stderr']
    assert any('code 4' in error for error in status['errors'])
    assert 'ROS setup failed' in (tmp_path / 'frames/camera.log').read_text()


def test_forced_camera_stop_is_a_failed_recording():
    import subprocess

    calls = []
    def wait(timeout):
        calls.append(timeout)
        if len(calls) == 1:
            raise subprocess.TimeoutExpired('camera fixture', timeout)
        return -9
    tap = cameras.CameraTap('box', {'wrist': '/w'})
    tap._proc = SimpleNamespace(stdin=io.BytesIO(), wait=wait, kill=lambda: None, returncode=-9)
    with pytest.raises(RuntimeError, match='did not stop after its input was closed'):
        tap.stop()
    assert tap.status()['status'] == 'failed'
    assert tap.status()['stopped_at'] is not None


def test_unconfirmed_camera_stop_retains_its_process_for_cleanup_retry():
    import subprocess

    calls = []
    def wait(timeout):
        calls.append(timeout)
        if len(calls) <= 2:
            raise subprocess.TimeoutExpired('camera fixture', timeout)
        return 0
    tap = cameras.CameraTap('box', {'wrist': '/w'})
    proc = tap._proc = SimpleNamespace(stdin=io.BytesIO(), wait=wait, kill=lambda: None, returncode=None)
    with pytest.raises(RuntimeError, match='Could not confirm'):
        tap.stop()
    assert tap._proc is proc
    assert tap.status()['status'] == 'failed' and tap.status()['stopped_at'] is None
    tap.stop()
    assert tap._proc is None
    assert tap.status()['stopped_at'] is not None


def test_camera_stop_retains_a_reader_that_is_still_running():
    tap = cameras.CameraTap('box', {'wrist': '/w'})
    proc = tap._proc = SimpleNamespace(stdin=io.BytesIO(), wait=lambda **kw: 0, returncode=0)
    alive = iter([True, False])
    tap._reader = SimpleNamespace(join=lambda **kw: None, is_alive=lambda: next(alive))
    with pytest.raises(RuntimeError, match='frame reader did not stop'):
        tap.stop()
    assert tap._proc is proc and tap.status()['stopped_at'] is None
    tap.stop()
    assert tap._proc is None and tap.status()['stopped_at'] is not None


def test_camera_idle_cleanup_reports_failure_and_retries_without_losing_ownership(caplog):
    import threading

    calls = []
    def stop():
        calls.append('stop')
        if len(calls) == 1:
            raise RuntimeError('camera still running')
    waits = iter([False, False, True])
    view = cameras.CameraView.__new__(cameras.CameraView)
    view._stop = SimpleNamespace(wait=lambda _: next(waits))
    view._tap = SimpleNamespace(stop=stop)
    view._lock, view._asked, view.idle_s = threading.Lock(), -1, 0
    view._idle()
    assert calls == ['stop', 'stop']
    assert view._tap is None
    assert 'camera still running' in caplog.text


def test_failed_camera_view_keeps_ownership_until_cleanup_is_confirmed(monkeypatch):
    started = []
    can_stop = False
    class Tap:
        def __init__(self, sandbox, cams, fps):
            self.sandbox, self.cameras = sandbox, cams
            self.running = False
        def start(self):
            started.append(self.sandbox)
        def stop(self):
            if not can_stop:
                raise RuntimeError('Camera process still running')
        def status(self):
            return {'status': 'failed', 'errors': ['Camera process exited.'], 'stderr': 'ROS setup failed'}
    monkeypatch.setattr(cameras, 'CameraTap', Tap)
    view = cameras.CameraView(lambda: SimpleNamespace(sandbox='box', cameras=({'wrist': '/w'}, None)))
    try:
        for _ in range(2):
            with pytest.raises(RuntimeError, match='Camera process still running'):
                view.frame('wrist')
        assert started == ['box']
    finally:
        can_stop = True
        view.close()


def test_camera_view_recovers_on_the_next_request_after_confirmed_cleanup(monkeypatch):
    import threading

    started, stopped = [], []
    class Tap:
        def __init__(self, sandbox, cams, fps):
            self.sandbox, self.cameras = sandbox, cams
            self.running = bool(started)
            self.lock = threading.Lock()
            self.latest = {'wrist': {'t': 1.0, 'mime': 'image/jpeg', 'data': b'recovered'}}
        def start(self):
            started.append(self.sandbox)
        def stop(self):
            stopped.append(self.sandbox)
        def status(self):
            return ({'status': 'recording', 'errors': []} if self.running else
                    {'status': 'failed', 'errors': ['Camera process exited.'], 'stderr': 'ROS setup failed'})
    monkeypatch.setattr(cameras, 'CameraTap', Tap)
    view = cameras.CameraView(lambda: SimpleNamespace(sandbox='box', cameras=({'wrist': '/w'}, None)))
    try:
        with pytest.raises(RuntimeError, match='ROS setup failed'):
            view.frame('wrist')
        assert stopped == ['box'] and view._tap is None
        assert view.frame('wrist')['data'] == 'cmVjb3ZlcmVk'
        assert started == ['box', 'box']
    finally:
        view.close()
