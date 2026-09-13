#!/usr/bin/env python3
"""Grab depth+color+info+TF for a camera; save world xyz map to <cam>_xyz.npy and color to <cam>.png.
Usage: python3 geo.py <cam>"""
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time; end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

def quat_R(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def main():
    cam = sys.argv[1]
    rclpy.init(); node = rclpy.create_node("geo")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time; end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation; R = quat_R(q.x, q.y, q.z, q.w)
    p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    pc = np.stack([(u-cx)*depth/fx, (v-cy)*depth/fy, depth], -1)
    xyz = pc @ R.T + p0
    np.save(f"{cam}_xyz.npy", xyz); cv2.imwrite(f"{cam}.png", color)
    print("saved", cam, "cam pos", p0, "depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()

main()
