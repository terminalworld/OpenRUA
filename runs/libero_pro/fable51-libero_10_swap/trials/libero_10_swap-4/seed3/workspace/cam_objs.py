"""Segment raised objects in any camera's depth image; print world extents & rim circle fits.
Usage: cam_objs.py <cam> [zmin_rim=0.50]"""
import sys, time, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = sys.argv[1]; zrim = float(sys.argv[2]) if len(sys.argv) > 2 else 0.50
rclpy.init(); n = rclpy.create_node("co")
got = {}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
n.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def cb(m):
    for t in m.transforms:
        if t.header.frame_id == "world" and t.child_frame_id == f"{cam}_optical_frame":
            got["tf"] = t.transform
n.create_subscription(TFMessage, "/tf", cb, 50)
t0 = time.time()
while not all(k in got for k in ("d", "i", "tf", "c")) and time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.1)
d, info, t = got["d"], got["i"], got["tf"]
q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],[2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
print("cam at", np.round(T, 4))
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
vv, uu = np.mgrid[0:d.height, 0:d.width]
P = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1) @ R.T + T
color = np.frombuffer(got["c"].data, dtype=np.uint8).reshape(d.height, d.width, -1)[..., :3]
Z = P[..., 2]
np.save(f"{cam}_P.npy", P)
def fit(pts):
    A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]
    b = (pts**2).sum(1)
    (a, bb, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    return a, bb, np.sqrt(c + a*a + bb*bb)
zlo = float(sys.argv[3]) if len(sys.argv) > 3 else 0.435; zhi = float(sys.argv[4]) if len(sys.argv) > 4 else 0.62
mask = ((Z > zlo) & (Z < zhi) & np.isfinite(Z)).astype(np.uint8)
nlab, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, nlab):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    sel = lab == i; pts = P[sel]; col = color[sel].mean(0)
    print(f"blob{i}: area={stats[i,4]} px=({cents[i][0]:.0f},{cents[i][1]:.0f}) xy=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) "
          f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] rgb={col.round(0)}")
    rim = sel & (Z > zrim)
    if rim.sum() > 10:
        a, bb, rr = fit(P[rim][:, :2])
        print(f"   rim fit (z>{zrim}): center=({a:.4f},{bb:.4f}) r={rr:.4f} n={rim.sum()} zmax={P[rim][:,2].max():.4f}")
