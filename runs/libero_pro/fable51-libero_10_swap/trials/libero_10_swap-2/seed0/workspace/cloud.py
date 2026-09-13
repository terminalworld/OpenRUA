import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
qos = QoSProfile(depth=100); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
def cb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
while not all(k in got for k in "dci") or "tf" not in got: rclpy.spin_once(node, timeout_sec=0.2)
b = CvBridge()
depth = b.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float32)
color = b.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
tr = got["tf"]; q = tr.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
X = (uu-cx)*depth/fx; Y = (vv-cy)*depth/fy
P = np.stack([X,Y,depth],-1) @ R.T + t
np.save(f"{cam}_world.npy", P); np.save(f"{cam}_color.npy", color)
print("saved", P.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
