"""Grab a robot's camera frames from its ROS 2 graph: a standalone
program, run inside the robot's sandbox (where the graph and rclpy are),
that writes each camera's latest frame to stdout a few times a second.

    python3 camera_tap.py --camera wrist=/wrist_cam/color/image_raw --fps 2
    python3 camera_tap.py --pattern '/{name}/color/image_raw' --list

The first record names the cameras (``{"t", "cameras": {name: topic},
"frames": []}``); then each tick is one record: a JSON line ``{"t": <unix
time>, "frames": [{"name", "mime", "bytes"}, ...]}`` followed by the
frames' bytes in that order. A camera that also publishes ``<topic>/compressed`` is read from
there (its JPEG passes through untouched); otherwise its raw image is
scaled to ``--width`` and encoded as JPEG. ``--pattern`` names cameras
by the topics that match it when the program starts. The program runs at
a low CPU priority, so a robot's control loop on the same computer comes
first, and exits when its stdin closes (the reader is gone).
"""

from __future__ import annotations

import argparse
import io
import json
import os
import re
import sys
import threading
import time


def discover(node, pattern: str) -> dict[str, str]:
    """Camera name -> topic for every topic on the graph matching
    ``pattern`` ({name} stands for the camera's name)."""
    head, _, tail = pattern.partition("{name}")
    shape = re.compile("^" + re.escape(head) + "([^/]+)" + re.escape(tail) + "$")
    # Discovery fills in over a moment: look until the set has stopped
    # growing for a second and a half (ten seconds at most).
    deadline, settled, found = time.monotonic() + 10.0, None, {}
    while time.monotonic() < deadline:
        before = len(found)
        for topic, _ in node.get_topic_names_and_types():
            m = shape.match(topic)
            if m:
                found[m.group(1)] = topic
        if len(found) != before or settled is None and found:
            settled = time.monotonic()
        if settled is not None and time.monotonic() - settled > 1.5:
            break
        time.sleep(0.2)
    return found


def jpeg(msg, width: int) -> bytes:
    """A raw sensor_msgs/Image as a JPEG at most ``width`` pixels wide."""
    import numpy as np
    from PIL import Image

    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f"unsupported encoding {msg.encoding}")
    pixels = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = pixels[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding.startswith("bgr"):
        pixels = pixels[:, :, [2, 1, 0]]
    elif channels == 4:
        pixels = pixels[:, :, :3]
    image = Image.fromarray(pixels.squeeze() if channels == 1 else pixels)
    if image.width > width:
        image = image.resize((width, max(1, image.height * width // image.width)))
    out = io.BytesIO()
    image.save(out, format="JPEG", quality=80)
    return out.getvalue()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--camera", action="append", default=[], metavar="NAME=TOPIC",
                    help="a camera and its color image topic (repeatable)")
    ap.add_argument("--pattern", default=None,
                    help="cameras are the topics matching this, {name} the camera's name")
    ap.add_argument("--fps", type=float, default=2.0, help="ticks per second")
    ap.add_argument("--width", type=int, default=640, help="widest frame a raw image is scaled to")
    ap.add_argument("--list", action="store_true",
                    help="write the record that names the cameras, then exit")
    args = ap.parse_args()
    try:
        os.nice(10)
    except OSError:
        pass

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image

    rclpy.init()
    node = rclpy.create_node("openrua_camera_tap")
    cameras = dict(c.split("=", 1) for c in args.camera)
    if args.pattern:
        cameras.update(discover(node, args.pattern))
    out = sys.stdout.buffer
    if args.list:
        out.write((json.dumps({"t": time.time(), "cameras": cameras, "frames": []}) + "\n").encode())
        out.flush()
        node.destroy_node()
        rclpy.try_shutdown()
        return 0
    published = {t for t, _ in node.get_topic_names_and_types()}
    latest: dict[str, tuple[str, object]] = {}
    lock = threading.Lock()

    def keep(name, kind):
        def take(msg):
            with lock:
                latest[name] = (kind, msg)
        return take

    for name, topic in cameras.items():
        if f"{topic}/compressed" in published:
            node.create_subscription(CompressedImage, f"{topic}/compressed",
                                     keep(name, "compressed"), qos_profile_sensor_data)
        else:
            node.create_subscription(Image, topic, keep(name, "raw"), qos_profile_sensor_data)

    gone = threading.Event()
    # The reader closing stdin is the signal to stop: no orphan grabbers.
    threading.Thread(target=lambda: (sys.stdin.buffer.read(), gone.set()), daemon=True).start()

    def tick():
        with lock:
            taken = dict(latest)
            latest.clear()
        frames, blobs = [], []
        for name, (kind, msg) in sorted(taken.items()):
            try:
                if kind == "compressed":
                    data = bytes(msg.data)
                    mime = "image/png" if "png" in msg.format else "image/jpeg"
                else:
                    data, mime = jpeg(msg, args.width), "image/jpeg"
            except Exception as exc:  # noqa: BLE001; one camera's bad frame is not the tap's end
                print(f"camera_tap: {name}: {exc}", file=sys.stderr)
                continue
            frames.append({"name": name, "mime": mime, "bytes": len(data)})
            blobs.append(data)
        if not frames:
            return
        try:
            out.write((json.dumps({"t": time.time(), "frames": frames}) + "\n").encode())
            for blob in blobs:
                out.write(blob)
            out.flush()
        except (BrokenPipeError, OSError):
            gone.set()

    node.create_timer(1.0 / args.fps, tick)
    # The first record names the cameras, frames or not.
    out.write((json.dumps({"t": time.time(), "cameras": cameras, "frames": []}) + "\n").encode())
    out.flush()
    try:
        while rclpy.ok() and not gone.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
