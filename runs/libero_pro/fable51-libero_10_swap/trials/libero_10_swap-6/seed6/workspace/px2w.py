"""px2w.py <camera> u,v [u,v ...]  -> world xyz for each pixel (one depth frame)."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = sys.argv[1]
pix = [tuple(int(a) for a in p.split(",")) for p in sys.argv[2:]]
rclpy.init(); node = rclpy.create_node("px2w")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["t"] = t.transform
node.create_subscription(TFMessage, "/tf", tfcb, 10)
while len(got) < 3: rclpy.spin_once(node, timeout_sec=0.2)
d, info, t = got["d"], got["i"], got["t"]
print("TF", cam, round(t.translation.x,3), round(t.translation.y,3), round(t.translation.z,3), round(t.rotation.x,3), round(t.rotation.y,3), round(t.rotation.z,3), round(t.rotation.w,3))
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
np.save(f"{cam}_depth.npy", depth)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
for (u, v) in pix:
    zc = depth[v, u]
    pc = np.array([(u-cx)*zc/fx, (v-cy)*zc/fy, zc])
    pw = R @ pc + T
    print(f"({u},{v}) depth={zc:.3f} world=({pw[0]:.3f}, {pw[1]:.3f}, {pw[2]:.3f})")
