#!/usr/bin/env python3
"""Segment objects above the table from the birdview depth+color and print
world-frame centroids, heights and footprints."""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

CAM = sys.argv[1] if len(sys.argv) > 1 else "birdview"


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("locate")
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{CAM}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(grab(node, f"/{CAM}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{CAM}/color/camera_info", CameraInfo)
    tf = None
    while tf is None:
        m = grab(node, "/tf", TFMessage)
        for t in m.transforms:
            if t.child_frame_id == f"{CAM}_optical_frame":
                tf = t
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1).reshape(-1, 3)
    q = tf.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    tr = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
    Pw = (R @ P.T).T + tr
    Pw = Pw.reshape(H, W, 3)
    np.save(f"snaps/{CAM}_world.npy", Pw)
    z = Pw[..., 2]
    # table height estimate: mode of z over central region
    zz = z[np.isfinite(z)]
    hist, edges = np.histogram(zz, bins=400, range=(0, 2))
    table_z = edges[np.argmax(hist)]
    print(f"table_z ~ {table_z:.3f}")
    mask = (z > table_z + 0.01) & (z < table_z + 0.5) & np.isfinite(z)
    # exclude robot: anything near base x<-0.2 ... print all blobs anyway
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15:
            continue
        sel = lab == i
        pts = Pw[sel]
        col = color[sel].mean(0)
        x0, y0, x1, y1 = pts[:, 0].min(), pts[:, 1].min(), pts[:, 0].max(), pts[:, 1].max()
        print(f"blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i, cv2.CC_STAT_AREA]} "
              f"centroid=({pts[:, 0].mean():.3f},{pts[:, 1].mean():.3f}) top_z={pts[:, 2].max():.3f} "
              f"xrange=[{x0:.3f},{x1:.3f}] yrange=[{y0:.3f},{y1:.3f}] bgr={col.astype(int)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
