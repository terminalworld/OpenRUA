#!/usr/bin/env python3
"""Locate mugs/plates from the birdview camera: colour segmentation +
depth + intrinsics + TF -> world coordinates. Prints one line per blob."""
import sys
import numpy as np, rclpy, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

CAM = sys.argv[1] if len(sys.argv) > 1 else "birdview"


def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


rclpy.init()
node = rclpy.create_node("locate")
tfbuf = Buffer(); TransformListener(tfbuf, node)
br = CvBridge()
color = br.imgmsg_to_cv2(grab(node, f"/{CAM}/color/image_raw", Image), "bgr8")
depth = br.imgmsg_to_cv2(grab(node, f"/{CAM}/depth/image_raw", Image), "passthrough").astype(float)
info = grab(node, f"/{CAM}/color/camera_info", CameraInfo)
frame = f"{CAM}_optical_frame"
while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation
R = quat_R(q.x, q.y, q.z, q.w)
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
pc = np.stack([(uu - cx) * depth / fx, (vv - cy) * depth / fy, depth], -1)
pw = pc @ R.T + tr  # world xyz per pixel
np.save(f"{CAM}_world.npy", pw)
cv2.imwrite(f"{CAM}_color.png", color)

# table height estimate: the mode of z over the table region
zs = pw[..., 2]
finite = np.isfinite(zs)
hist, edges = np.histogram(zs[finite], bins=400)
table_z = edges[np.argmax(hist)]
print(f"table_z ~ {table_z:.3f}")

# anything above the table by >8mm and not the robot: connected blobs
above = finite & (zs > table_z + 0.008)
above = above.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(above)
for i in range(1, n):
    x, y, w, h, a = stats[i]
    if a < 30:
        continue
    m = lab == i
    P = pw[m]
    c = color[m].mean(0)[::-1]  # rgb
    print(f"blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) area={a} "
          f"world xy=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) "
          f"zmax={P[:,2].max():.3f} zmin={P[:,2].min():.3f} rgb={c.astype(int)}")
rclpy.shutdown()
