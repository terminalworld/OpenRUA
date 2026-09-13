#!/usr/bin/env python3
"""Build world-frame point clouds from the named cameras; save as npy (N x 6: xyz bgr)."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def grab(node, cams):
    got = {}
    subs = []
    for c in cams:
        subs.append(node.create_subscription(Image, f"/{c}/depth/image_raw", lambda m, c=c: got.setdefault(c + "d", m), 1))
        subs.append(node.create_subscription(Image, f"/{c}/color/image_raw", lambda m, c=c: got.setdefault(c + "c", m), 1))
        subs.append(node.create_subscription(CameraInfo, f"/{c}/color/camera_info", lambda m, c=c: got.setdefault(c + "i", m), 1))
    while len(got) < 3 * len(cams):
        rclpy.spin_once(node, timeout_sec=0.2)
    for s in subs:
        node.destroy_subscription(s)
    return got


def main():
    cams = sys.argv[1:] or ["birdview", "agentview", "frontview", "sideview"]
    rclpy.init(); node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    got = grab(node, cams)
    br = CvBridge()
    for c in cams:
        frame = f"{c}_optical_frame"
        while not buf.can_transform("world", frame, rclpy.time.Time()):
            rclpy.spin_once(node, timeout_sec=0.2)
        t = buf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation; R = quat_R(q.x, q.y, q.z, q.w)
        T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        d = br.imgmsg_to_cv2(got[c + "d"], "passthrough").astype(np.float64)
        col = br.imgmsg_to_cv2(got[c + "c"], "bgr8")
        k = got[c + "i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        H, W = d.shape
        vv, uu = np.mgrid[0:H, 0:W]
        ok = np.isfinite(d) & (d > 0.05) & (d < 5)
        X = (uu - cx) * d / fx; Y = (vv - cy) * d / fy
        P = np.stack([X, Y, d], -1)[ok] @ R.T + T
        out = np.concatenate([P, col[ok].astype(np.float64), np.stack([uu, vv], -1)[ok]], 1)
        np.save(f"cloud_{c}.npy", out)
        print(c, out.shape, "cam at", T)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
