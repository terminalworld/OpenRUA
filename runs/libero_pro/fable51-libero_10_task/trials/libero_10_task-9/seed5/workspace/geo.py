#!/usr/bin/env python3
"""Geometry helpers: camera TFs, pixel->world, point clouds from saved depth.

Usage:
  python3 geo.py tf                      # dump world->camera TFs to cams.json
  python3 geo.py px <cam> <u> <v> [...]  # world xyz for pixels (uses <cam>_depth.npy)
  python3 geo.py cloud <cam>             # print table-height stats + object blobs
"""
import json
import sys

import numpy as np

K_DEFAULT = dict(fx=579.4112549695428, fy=579.4112549695428, cx=320.0, cy=240.0)
K_BY_CAM = {"robot0_eye_in_hand": dict(fx=312.77408948188935, fy=312.77408948188935, cx=320.0, cy=240.0)}


def K_for(cam):
    return K_BY_CAM.get(cam, K_DEFAULT)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def fetch_tf():
    import rclpy
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    node = rclpy.create_node("geo_tf")
    out = {}

    def cb(m):
        for t in m.transforms:
            tr, q = t.transform.translation, t.transform.rotation
            out[f"{t.header.frame_id}->{t.child_frame_id}"] = dict(
                t=[tr.x, tr.y, tr.z], q=[q.x, q.y, q.z, q.w])
            if t.header.frame_id == "world":
                out[t.child_frame_id] = out[f"{t.header.frame_id}->{t.child_frame_id}"]

    node.create_subscription(TFMessage, "/tf", cb, 50)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.5)
        if "birdview_optical_frame" in out and "panda_link0" in out:
            break
    rclpy.shutdown()
    json.dump(out, open("cams.json", "w"), indent=1)
    print(json.dumps(out, indent=1))


def cam_T(cam):
    c = json.load(open("cams.json"))[f"{cam}_optical_frame"]
    T = np.eye(4)
    T[:3, :3] = quat_R(*c["q"])
    T[:3, 3] = c["t"]
    return T


def px_to_world(cam, depth, u, v):
    K = K_for(cam)
    z = depth[v, u]
    p = np.array([(u - K["cx"]) * z / K["fx"], (v - K["cy"]) * z / K["fy"], z, 1.0])
    return (cam_T(cam) @ p)[:3]


def cloud(cam, depth):
    K = K_for(cam)
    h, w = depth.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    z = depth
    pts = np.stack([(uu - K["cx"]) * z / K["fx"], (vv - K["cy"]) * z / K["fy"], z,
                    np.ones_like(z)], -1).reshape(-1, 4)
    return (cam_T(cam) @ pts.T).T[:, :3].reshape(h, w, 3)


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "tf":
        fetch_tf()
    elif cmd == "px":
        cam = sys.argv[2]
        depth = np.load(f"{cam}_depth.npy")
        args = list(map(int, sys.argv[3:]))
        for u, v in zip(args[::2], args[1::2]):
            print(u, v, np.round(px_to_world(cam, depth, u, v), 4))
    elif cmd == "cloud":
        cam = sys.argv[2]
        depth = np.load(f"{cam}_depth.npy")
        P = cloud(cam, depth)
        np.save(f"{cam}_cloud.npy", P)
        zs = P[..., 2]
        print("z percentiles", np.percentile(zs[np.isfinite(zs)], [1, 5, 25, 50, 75, 95, 99]))
