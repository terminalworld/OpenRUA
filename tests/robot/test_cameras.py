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

    monkeypatch.setattr(cameras, "CameraTap", Tap)
    robot = SimpleNamespace(sandbox="box", cameras=({"wrist": "/w", "scene": "/s"}, None))
    view = cameras.CameraView(lambda: robot)
    try:
        assert view.names() == ["scene", "wrist"]
        assert view.frame("wrist")["data"] == "eA=="
        view.frame("wrist")
        view.frame("scene")
        assert started == [["wrist"], ["scene"]]   # one at a time, switched on request
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
