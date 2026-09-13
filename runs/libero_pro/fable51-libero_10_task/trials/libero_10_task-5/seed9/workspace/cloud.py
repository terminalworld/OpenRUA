"""Grab depth+info for a camera, transform to world, save npz of (H,W,3) world xyz."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from cv_bridge import CvBridge
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
node.create_subscription(TFMessage, "/tf_static", tfcb, qos)
node.create_subscription(TFMessage, "/tf", tfcb, 100)
while not all(k in got for k in "dcitf".replace("tf","")) or "tf" not in got:
    rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
H, W = depth.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
X = (u - cx) * depth / fx; Y = (v - cy) * depth / fy; Z = depth
t = got["tf"]; q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
P = np.stack([X, Y, Z], -1) @ R.T + np.array([t.translation.x, t.translation.y, t.translation.z])
np.savez(f"/workspace/img/{cam}_cloud.npz", xyz=P, depth=depth, color=color)
print("saved", P.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
rclpy.shutdown()
