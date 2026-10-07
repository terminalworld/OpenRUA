"""Depth measurements follow native ROS image layouts and REP 118 units."""
import importlib.util
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS

import pytest

from openrua.sandbox import workspace


@pytest.fixture
def tool(monkeypatch):
    for name, value in {
        'rclpy': NS(), 'sensor_msgs.msg': NS(CameraInfo=NS, Image=NS),
        'tf2_ros': NS(Buffer=NS, TransformListener=NS),
    }.items():
        monkeypatch.setitem(sys.modules, name, value)
    path = Path(workspace.__file__).parent / 'workspace/tools/perception/px2world.py'
    spec = importlib.util.spec_from_file_location('px2world', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def message(encoding='32FC1', big=False, padding=b''):
    code, factor = ('f', 1) if encoding == '32FC1' else ('H', 1000)
    order = '>' if big else '<'
    rows = [struct.pack(order + code * 2, factor * a, factor * b) + padding
            for a, b in [(1, 2), (3, 4)]]
    return NS(width=2, height=2, step=len(rows[0]), encoding=encoding,
              is_bigendian=int(big), data=b''.join(rows))


@pytest.mark.parametrize('encoding', ['32FC1', '16UC1'])
@pytest.mark.parametrize('big', [False, True])
@pytest.mark.parametrize('padding', [b'', b'\xff' * 4])
def test_pixel_uses_native_layout_and_returns_meters(tool, encoding, big, padding):
    image = message(encoding, big, padding)
    assert tool.depth_meters(image, 0, 1) == 3.0
    assert tool.depth_meters(image, 1, 1) == 4.0


@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf'), -float('inf')])
def test_invalid_float_depth_does_not_produce_coordinates(tool, value):
    image = message()
    image.data = struct.pack('<f', value) + image.data[4:]
    with pytest.raises(ValueError, match='no valid depth'):
        tool.depth_meters(image, 0, 0)


def test_zero_raw_depth_is_invalid(tool):
    image = message('16UC1')
    image.data = b'\x00\x00' + image.data[2:]
    with pytest.raises(ValueError, match='no valid depth'):
        tool.depth_meters(image, 0, 0)


@pytest.mark.parametrize('u,v', [(-1, 0), (2, 0), (0, -1), (0, 2)])
def test_pixel_outside_image_fails(tool, u, v):
    with pytest.raises(ValueError, match='outside'):
        tool.depth_meters(message(), u, v)


@pytest.mark.parametrize('change', [{'encoding': 'mono16'}, {'step': 1}, {'data': b'\x00'}])
def test_unsupported_or_broken_message_fails(tool, change):
    image = message()
    image.__dict__.update(change)
    with pytest.raises(ValueError):
        tool.depth_meters(image, 0, 0)
