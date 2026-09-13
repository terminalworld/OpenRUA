"""Batch pixel->world for one camera using a fresh depth frame. usage: cam u,v u,v ..."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
cam = sys.argv[1]
pts = [tuple(int(x) for x in a.split(",")) for a in sys.argv[2:]]
rclpy.init(); node = rclpy.create_node("px2wb")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
def cb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
node.create_subscription(TFMessage, "/tf_static", cb, qos); node.create_subscription(TFMessage, "/tf", cb, 100)
import time; end = time.time()+20
while not all(k in got for k in ("d","i","tf")) and time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
d, info, t = got["d"], got["i"], got["tf"]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
q = t.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T = np.eye(4); T[:3,:3]=R; T[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
for (u,v) in pts:
    Z = depth[v,u]
    p = T @ np.array([(u-cx)*Z/fx, (v-cy)*Z/fy, Z, 1.0])
    print(f"({u},{v}) depth={Z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
