"""Batch pixel->world for one camera. Usage: px2w_batch.py <cam> u,v [u,v ...]
Uses TF captured by spinning up to 15s (world->cam_optical published slowly)."""
import sys, struct, time, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = sys.argv[1]; pix = [tuple(map(int, a.split(','))) for a in sys.argv[2:]]
rclpy.init(); n = rclpy.create_node("px2w")
got = {}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def cb(m):
    for t in m.transforms:
        if t.header.frame_id == "world" and t.child_frame_id == f"{cam}_optical_frame":
            got["tf"] = t.transform
n.create_subscription(TFMessage, "/tf", cb, 50)
t0 = time.time()
while not all(k in got for k in ("d", "i", "tf")) and time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.1)
d, info, t = got["d"], got["i"], got["tf"]
q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
for u, v in pix:
    zc = float(depth[v, u])
    p = R @ np.array([(u-cx)*zc/fx, (v-cy)*zc/fy, zc]) + T
    print(f"px({u},{v}) depth={zc:.4f} -> world ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
