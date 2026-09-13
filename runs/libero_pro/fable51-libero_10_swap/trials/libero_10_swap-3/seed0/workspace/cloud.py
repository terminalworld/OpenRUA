"""Grab color+depth+info from a camera, save npz with world-frame points."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
buf = Buffer(); TransformListener(buf, node)
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
while not all(k in got for k in "dci") or not buf.can_transform("world", f"{cam}_optical_frame", rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float32)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = buf.lookup_transform("world", f"{cam}_optical_frame", rclpy.time.Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
H, W = depth.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
pc = np.stack([(u-cx)*depth/fx, (v-cy)*depth/fy, depth], -1)
pw = pc @ R.T + T
np.savez(f"{cam}_cloud.npz", pw=pw, color=color, depth=depth)
print("saved", pw.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
