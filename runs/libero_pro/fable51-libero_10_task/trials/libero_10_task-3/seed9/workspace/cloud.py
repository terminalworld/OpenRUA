"""Grab depth+info for a camera, save world-frame xyz array (H,W,3) as <cam>_xyz.npy"""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(TFMessage, "/tf_static", lambda m: [got.setdefault("t", t) for t in m.transforms if t.child_frame_id == f"{cam}_optical_frame"], qos)
node.create_subscription(TFMessage, "/tf", lambda m: [got.setdefault("t", t) for t in m.transforms if t.child_frame_id == f"{cam}_optical_frame"], 100)
while not all(k in got for k in "dit"): rclpy.spin_once(node, timeout_sec=0.2)
d, info, t = got["d"], got["i"], got["t"]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
v, u = np.mgrid[0:d.height, 0:d.width]
X = (u - cx) * depth / fx; Y = (v - cy) * depth / fy; Z = depth
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],[2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
P = np.stack([X, Y, Z], -1) @ R.T + T
np.save(f"snaps/{cam}_xyz.npy", P.astype(np.float32))
print(cam, P.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
