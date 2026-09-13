"""Dump a camera's full depth frame as world-frame points: <cam>_xyz.npy (H,W,3)."""
import sys, numpy as np, rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
cam = sys.argv[1]
rclpy.init(); n = rclpy.create_node("cloud"); b = Buffer(); TransformListener(b, n)
got = {}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
while "d" not in got or "i" not in got or not b.can_transform("world", f"{cam}_optical_frame", Time()):
    rclpy.spin_once(n, timeout_sec=0.2)
d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
H, W = d.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
t = b.lookup_transform("world", f"{cam}_optical_frame", Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
pw = pc @ R.T + T
np.save(f"{cam}_xyz.npy", pw)
print(cam, pw.shape, "depth range", np.nanmin(d), np.nanmax(d))
