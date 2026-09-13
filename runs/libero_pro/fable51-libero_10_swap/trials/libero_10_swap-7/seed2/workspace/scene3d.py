"""Project a camera's depth frame to world points; cluster things above the table."""
import sys, numpy as np, rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("scene3d")
buf = Buffer(); TransformListener(buf, node)
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
while len(got) < 3 or not buf.can_transform("world", f"{cam}_optical_frame", Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = buf.lookup_transform("world", f"{cam}_optical_frame", Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
Z = depth
X = (uu - cx) * Z / fx; Y = (vv - cy) * Z / fy
P = np.stack([X, Y, Z], -1) @ R.T + T
np.save(f"{cam}_world.npy", P)
valid = np.isfinite(Z) & (Z > 0)
zs = P[..., 2][valid]
print("z percentiles", np.percentile(zs, [1, 5, 25, 50, 75, 95, 99]))
# table height estimate = mode of z
hist, edges = np.histogram(zs, bins=200)
table_z = edges[np.argmax(hist)]
print("table_z ~", table_z)
above = valid & (P[..., 2] > table_z + 0.01) & (P[..., 2] < table_z + 0.5)
# connected components on the mask
import cv2
n, lab = cv2.connectedComponents(above.astype(np.uint8))
for i in range(1, n):
    m = lab == i
    if m.sum() < 30: continue
    pts = P[m]
    col = color[m].mean(0)[::-1]
    us, vs = uu[m], vv[m]
    print(f"blob {i}: n={m.sum()} px u[{us.min()},{us.max()}] v[{vs.min()},{vs.max()}] "
          f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] rgb={col.astype(int)}")
rclpy.shutdown()
