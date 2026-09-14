"""The bridge's recording: which cameras, at what size."""

from openrua.robot.sim.bridge.main import recording

CFG = {"machine": {"cameras": {"resolution": [640, 480], "record": ["agentview", "wrist"]}}}


def test_recording_defaults_to_the_profile(tmp_path):
    rec = recording(CFG, str(tmp_path / "frames"), None)
    assert rec.cameras == [("agentview", 640, 480), ("wrist", 640, 480)]


def test_recording_size_overrides_the_resolution(tmp_path):
    rec = recording(CFG, str(tmp_path / "frames"), "agentview", every=2, size="1280x960")
    assert rec.cameras == [("agentview", 1280, 960)]
    assert rec.every == 2


def test_no_directory_means_no_recording():
    assert recording(CFG, None, None) is None
