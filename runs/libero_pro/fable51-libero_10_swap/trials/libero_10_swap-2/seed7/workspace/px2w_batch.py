"""Usage: px2w_batch.py <camera> u,v [u,v ...]  -> world xyz for each pixel of current depth frame"""
import struct, sys
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def main():
    cam = sys.argv[1]; pts = [tuple(map(int,a.split(","))) for a in sys.argv[2:]]
    rclpy.init(); n = rclpy.create_node("px2wb")
    got = {}
    n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
    n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
    qos = QoSProfile(depth=10); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    def tfcb(m):
        for t in m.transforms:
            if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
    n.create_subscription(TFMessage, "/tf_static", tfcb, qos)
    n.create_subscription(TFMessage, "/tf", tfcb, 10)
    while len(got) < 3: rclpy.spin_once(n, timeout_sec=0.2)
    d, i, t = got["d"], got["i"], got["tf"]
    depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
    fx, fy, cx, cy = i.k[0], i.k[4], i.k[2], i.k[5]
    R = quat_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    T = np.array([t.translation.x, t.translation.y, t.translation.z])
    for u, v in pts:
        z = depth[v, u]
        p = R @ np.array([(u-cx)*z/fx, (v-cy)*z/fy, z]) + T
        print(f"({u},{v}) depth={z:.3f} world=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
    rclpy.shutdown()
main()
