import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = "birdview"
rclpy.init(); node = rclpy.create_node("hm")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def cb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
node.create_subscription(TFMessage, "/tf", cb, 100)
import time; end = time.time()+20
while not all(k in got for k in ("d","i","tf")) and time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
d, info, t = got["d"], got["i"], got["tf"]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
print("fx", fx, "fy", fy, "cx", cx, "cy", cy)
q = t.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tt = np.array([t.translation.x,t.translation.y,t.translation.z])
vv, uu = np.mgrid[0:d.height, 0:d.width]
P = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1) @ R.T + tt
np.save("birdview_world.npy", P)
Z = P[...,2]
# objects above table (0.90) but below robot arm region (x < -0.35 is robot)
mask = (Z > 0.93) & (P[...,0] > -0.35)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    m = lab == i
    pts = P[m]
    print(f"blob {i}: area={stats[i,4]} px, centroid px=({cents[i][0]:.0f},{cents[i][1]:.0f})  x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f}")
