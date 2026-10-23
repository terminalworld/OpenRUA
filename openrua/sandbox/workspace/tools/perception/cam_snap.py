#!/usr/bin/env python3
"""Save one frame from a camera to a PNG.

Usage: python3 tools/perception/cam_snap.py <camera|/topic> [out.png] [--depth]
                                            [--timeout S]
<camera> is a name from machine.yaml's camera entries: its color stream,
or its depth stream with --depth. A name starting with / is a topic, read
as given. A depth image is saved as a visualization PNG plus its native
values in a .npy (units per the entry's depth_encoding). Exits nonzero if
no frame arrives within --timeout seconds (default 15).
"""
import argparse
from pathlib import Path

import rclpy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2


def camera_topic(name: str, stream: str) -> str:
    """The topic of one stream of a camera machine.yaml names."""
    root = Path(__file__).resolve().parents[2]
    m = yaml.safe_load((root / "machine.yaml").read_text())
    sensors = m.get("sensors", [])
    for e in sensors:
        if e["kind"] == "camera" and e["name"] == name:
            if not e.get(stream):
                raise SystemExit(f"camera {name} has no {stream} stream (machine.yaml)")
            return e[stream]
    pattern = next((e for e in sensors if e["kind"] == "camera_set"), None)
    if pattern and pattern.get(stream):
        return pattern[stream].replace("<name>", name)
    names = [e["name"] for e in sensors if e["kind"] == "camera"]
    raise SystemExit(f"no camera {name!r} in machine.yaml"
                     + (f"; cameras: {', '.join(names)}" if names else "; it lists none"))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("camera", help="camera name from machine.yaml, or a /topic")
    ap.add_argument("out", nargs="?", default=None, help="PNG path (default: <camera>.png)")
    ap.add_argument("--depth", action="store_true", help="the camera's depth stream")
    ap.add_argument("--timeout", type=float, default=15.0, help="seconds to wait for a frame")
    args = ap.parse_args()
    topic = (args.camera if args.camera.startswith("/")
             else camera_topic(args.camera, "depth" if args.depth else "color"))
    stem = args.camera.strip("/").split("/")[0] + ("_depth" if args.depth else "")
    out = args.out or f"{stem}.png"
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("cam_snap")
    got: list = []
    node.create_subscription(Image, topic, got.append, 1)
    end = node.get_clock().now().nanoseconds / 1e9 + args.timeout
    while not got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not got:
        raise SystemExit(f"no frame on {topic} within {args.timeout:.0f} s "
                         "(ros2 topic info shows whether anyone publishes it)")
    msg = got[0]
    if "FC" in msg.encoding or "16UC" in msg.encoding:  # depth image
        import numpy as np
        depth = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
        np.save(out.rsplit(".", 1)[0] + ".npy", depth)  # preserve the native values and units
        finite = depth[np.isfinite(depth)]
        lo, hi = (finite.min(), finite.max()) if finite.size else (0.0, 1.0)
        if not cv2.imwrite(out, ((depth - lo) / max(hi - lo, 1e-6) * 255).clip(0, 255)
                           .astype("uint8")):
            raise OSError(f"could not save image to {out}")
        print(f"{out} (visualization; raw {msg.encoding} values in .npy, "
              f"frame {msg.header.frame_id})")
    else:
        if not cv2.imwrite(out, CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")):
            raise OSError(f"could not save image to {out}")
        print(f"{out} ({msg.width}x{msg.height}, frame {msg.header.frame_id})")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
