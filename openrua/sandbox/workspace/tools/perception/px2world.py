#!/usr/bin/env python3
"""Convert one image pixel to world coordinates (depth + intrinsics + TF).

Usage: python3 tools/perception/px2world.py <camera> <u> <v>
Prints "x y z" in machine.yaml's frames.world for pixel column u, row v of
the camera's color image, using its CURRENT depth frame. The camera's
topics come from its machine.yaml entry; the frame its pixels are in is
the CameraInfo's header.frame_id. Needs depth aligned to color
(depth_aligned: true). Exits nonzero if the pixel has no valid depth or
TF has no path from the camera to the world frame. Supports 32FC1 meters
and 16UC1 millimeters per ROS REP 118. Pure geometry; which pixel is
worth asking about is your call.
"""
import struct
import sys
from pathlib import Path

import numpy as np
import yaml
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def depth_meters(image, u, v):
    """Read one REP 118 depth pixel using the Image message's byte layout."""
    formats = {"32FC1": ("f", 1.0), "16UC1": ("H", 0.001)}
    if image.encoding not in formats:
        raise ValueError(f"unsupported depth encoding {image.encoding!r}; "
                         "use a 32FC1 or 16UC1 depth image")
    if not (0 <= v < image.height and 0 <= u < image.width):
        raise ValueError(f"pixel ({u},{v}) outside {image.width}x{image.height}")
    code, scale = formats[image.encoding]
    size = struct.calcsize(code)
    if image.step < image.width * size or len(image.data) < image.step * image.height:
        raise ValueError("invalid depth image layout: check step, dimensions, and data length")
    endian = ">" if image.is_bigendian else "<"
    z = struct.unpack_from(endian + code, image.data, v * image.step + u * size)[0] * scale
    if not np.isfinite(z) or z <= 0:
        raise ValueError(f"no valid depth at ({u},{v}): {z}")
    return z


def _grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic} within {timeout}s")
    return got["m"]


def camera_entry(name: str) -> tuple[dict, str]:
    """The camera's machine.yaml entry (topics filled in) and the world frame."""
    root = Path(__file__).resolve().parents[2]
    m = yaml.safe_load((root / "machine.yaml").read_text())
    world = (m.get("frames") or {}).get("world")
    if not world:
        raise SystemExit("machine.yaml names no frames.world to express points in")
    sensors = m.get("sensors", [])
    entry = next((e for e in sensors if e["kind"] == "camera" and e["name"] == name), None)
    if entry is None:
        pattern = next((e for e in sensors if e["kind"] == "camera_set"), None)
        if pattern is None:
            names = [e["name"] for e in sensors if e["kind"] == "camera"]
            raise SystemExit(f"no camera {name!r} in machine.yaml; cameras: {', '.join(names)}")
        entry = {k: (v.replace("<name>", name) if isinstance(v, str) else v)
                 for k, v in pattern.items()}
    if not entry.get("depth"):
        raise SystemExit(f"camera {name} has no depth stream (machine.yaml)")
    if entry.get("depth_aligned") is False:
        raise SystemExit(f"camera {name}'s depth is not aligned to its color image; "
                         "use its depth stream's own camera_info and frame instead")
    return entry, world


def main() -> None:
    if len(sys.argv) != 4:
        raise SystemExit(__doc__)
    cam, u, v = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    entry, world = camera_entry(cam)
    rclpy.init()
    node = rclpy.create_node("px2world")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)

    depth = _grab(node, entry["depth"], Image)
    info = _grab(node, entry["info"], CameraInfo)
    try:
        z = depth_meters(depth, u, v)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    fx, fy = info.k[0], info.k[4]
    cx, cy = info.k[2], info.k[5]
    p_cam = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])

    frame = info.header.frame_id
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform(world, frame, rclpy.time.Time()):
            break
    else:
        raise SystemExit(f"TF has no transform from {frame} to {world} within 10 s: "
                         "this camera is not placed in the robot's TF tree (check "
                         f"ros2 run tf2_ros tf2_echo {world} {frame})")
    t = tfbuf.lookup_transform(world, frame, rclpy.time.Time())
    q = t.transform.rotation
    x, y, zz, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + zz * zz), 2 * (x * y - zz * w), 2 * (x * zz + y * w)],
        [2 * (x * y + zz * w), 1 - 2 * (x * x + zz * zz), 2 * (y * zz - x * w)],
        [2 * (x * zz - y * w), 2 * (y * zz + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    p = T @ p_cam
    print(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
