"""Camera success means an image file was saved, including nested output paths."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openrua.sandbox import workspace


@pytest.fixture
def camera(monkeypatch):
    image = np.ones((2, 3), dtype=np.float32)
    message = NS(encoding='bgr8', width=3, height=2, header=NS(frame_id='camera'))
    node = NS(create_subscription=lambda kind, topic, receive, qos: receive(message),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=0)), destroy_node=lambda: None)
    def write_image(path, pixels):
        try: Path(path).write_bytes(b'saved-image')
        except FileNotFoundError: return False
        return True
    for name, value in {
        'rclpy': NS(init=lambda: None, create_node=lambda name: node, shutdown=lambda: None),
        'sensor_msgs.msg': NS(Image=NS),
        'cv_bridge': NS(CvBridge=lambda: NS(imgmsg_to_cv2=lambda *a, **kw: image)),
        'cv2': NS(imwrite=write_image),
    }.items(): monkeypatch.setitem(sys.modules, name, value)
    path = Path(workspace.__file__).parent / 'workspace/tools/perception/cam_snap.py'
    spec = importlib.util.spec_from_file_location('cam_snap', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, message


@pytest.mark.parametrize('depth', [False, True])
def test_nested_camera_output_exists_before_success(camera, tmp_path, monkeypatch, capsys, depth):
    tool, message = camera
    if depth: message.encoding = '32FC1'
    out = tmp_path / 'observations' / 'today' / 'camera.png'
    monkeypatch.setattr(sys, 'argv', ['cam_snap', '/camera/image', str(out)])
    tool.main()
    assert out.read_bytes() == b'saved-image'
    assert str(out) in capsys.readouterr().out
    if depth: assert np.load(out.with_suffix('.npy')).shape == (2, 3)


@pytest.mark.parametrize('depth', [False, True])
def test_opencv_write_failure_is_not_reported_as_success(camera, tmp_path, monkeypatch, capsys, depth):
    tool, message = camera
    if depth: message.encoding = '32FC1'
    monkeypatch.setattr(tool.cv2, 'imwrite', lambda *args: False)
    monkeypatch.setattr(sys, 'argv', ['cam_snap', '/camera/image', str(tmp_path / 'camera.png')])
    with pytest.raises(OSError, match='could not save image'):
        tool.main()
    assert capsys.readouterr().out == ''
