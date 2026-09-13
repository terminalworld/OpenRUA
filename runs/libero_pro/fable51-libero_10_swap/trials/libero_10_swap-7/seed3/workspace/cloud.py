"""World-frame point cloud from a camera's depth + color; print blobs above table."""
import sys, struct, time
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import cv2

cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
node.create_subscription(TFMessage, "/tf", tfcb, 100)
end = time.time() + 20
while len(got) < 4 and time.time() < end: rclpy.spin_once(node, timeout_sec=0.2)
assert len(got) == 4, got.keys()
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = got["tf"]; q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
pc = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1)
pw = pc @ R.T + T
np.save(f"snaps/{cam}_world.npy", pw)
np.save(f"snaps/{cam}_color.npy", color)
print("depth range", np.nanmin(depth), np.nanmax(depth))
zs = pw[..., 2]
# table height = mode of z in central region
hist, edges = np.histogram(zs[np.isfinite(zs)], bins=400)
table_z = edges[np.argmax(hist)]
print("dominant z (table?)", table_z)
if len(sys.argv) > 2:
    lo = float(sys.argv[2])
    mask = (zs > lo).astype(np.uint8)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15: continue
        m = lab == i
        p = pw[m]
        print(f"blob {i}: px centroid=({cent[i][0]:.0f},{cent[i][1]:.0f}) area={stats[i,4]} "
              f"x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] "
              f"z[{p[:,2].min():.3f},{p[:,2].max():.3f}] mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) "
              f"color={color[m].mean(0).astype(int)}")
