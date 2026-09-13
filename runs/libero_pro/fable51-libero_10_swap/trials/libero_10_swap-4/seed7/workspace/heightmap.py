#!/usr/bin/env python3
"""Project a camera's depth frame to world, then report blobs above the
table between zmin and zmax (mugs/plates), with color samples.

Usage: python3 heightmap.py <camera> [zmin zmax]
"""
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2

sys.path.insert(0, "/workspace")
from locate import grab, quat_to_R  # noqa: E402


def main():
    cam = sys.argv[1]
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.44
    zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 0.70
    rclpy.init()
    node = rclpy.create_node("heightmap")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    bridge = CvBridge()
    color = grab(node, f"/{cam}/color/image_raw", Image)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    img = bridge.imgmsg_to_cv2(color, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    tr = np.array([t.transform.translation.x, t.transform.translation.y,
                   t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    pc = np.stack([(us - cx) * depth / fx, (vs - cy) * depth / fy, depth], -1)
    pw = pc @ R.T + tr
    np.save(f"{cam}_world.npy", pw)
    z = pw[..., 2]
    mask = ((z > zmin) & (z < zmax)).astype(np.uint8)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 15:
            continue
        m = lab == i
        pts = pw[m]
        col = img[m].mean(0)[::-1].astype(int)  # RGB
        x0, y0, bw, bh = stats[i, :4]
        print(f"blob {i}: px({cent[i][0]:.0f},{cent[i][1]:.0f}) "
              f"box({x0},{y0},{bw}x{bh}) area={area} "
              f"world x={pts[:,0].mean():.3f}[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] "
              f"y={pts[:,1].mean():.3f}[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
              f"zmax={pts[:,2].max():.3f} rgb={col}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
