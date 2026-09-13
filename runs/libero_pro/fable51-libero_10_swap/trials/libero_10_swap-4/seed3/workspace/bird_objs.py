"""Segment objects above the table in the birdview depth image; print world centroid/extents."""
import time, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = "birdview"
rclpy.init(); n = rclpy.create_node("bo")
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
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
vv, uu = np.mgrid[0:d.height, 0:d.width]
P = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1) @ R.T + T
color = np.frombuffer(got["c"].data, dtype=np.uint8).reshape(d.height, d.width, -1)[..., :3]
Z = P[..., 2]
mask = ((Z > 0.435) & (Z < 0.7)).astype(np.uint8)
nlab, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, nlab):
    if stats[i, cv2.CC_STAT_AREA] < 15: continue
    sel = lab == i
    pts = P[sel]
    col = color[sel].mean(0)
    print(f"blob{i}: px area={stats[i,4]} centroid px=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
          f"world xy=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) "
          f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"zmax={pts[:,2].max():.3f} zmin={pts[:,2].min():.3f} rgb={col.round(0)}")

print("--- rim fits (z>0.50) ---")
rim = ((Z > 0.50) & (Z < 0.7)).astype(np.uint8)
nlab, lab, stats, cents = cv2.connectedComponentsWithStats(rim)
for i in range(1, nlab):
    if stats[i, cv2.CC_STAT_AREA] < 10: continue
    pts = P[lab == i][:, :2]
    # algebraic circle fit
    A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]
    b = (pts**2).sum(1)
    (a, bb, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    r = np.sqrt(c + a*a + bb*bb)
    print(f"rim{i}: n={len(pts)} center=({a:.3f},{bb:.3f}) r={r:.3f} zmax={P[lab==i][:,2].max():.3f}")

print("--- rim fits excluding handle side ---")
def fit(pts):
    A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]
    b = (pts**2).sum(1)
    (a, bb, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    return a, bb, np.sqrt(c + a*a + bb*bb)
sel = (Z > 0.50) & (P[...,0] < -0.09) & (P[...,0] > -0.18) & (P[...,1] < -0.16) & (P[...,1] > -0.25)
print("white body-side rim:", fit(P[sel][:, :2]), len(P[sel]))
sel = (Z > 0.50) & (P[...,0] < -0.18) & (P[...,0] > -0.28) & (P[...,1] > 0.0) & (P[...,1] < 0.08)
print("yellow body-side rim:", fit(P[sel][:, :2]), len(P[sel]))
# save a depth-coded zoom for inspection
zc = np.clip((Z - 0.42) / 0.16, 0, 1)
img = cv2.applyColorMap((zc*255).astype(np.uint8), cv2.COLORMAP_JET)
img[Z < 0.435] = color[Z < 0.435][..., ::-1] // 2
crop = cv2.resize(img[210:300, 240:420], None, fx=6, fy=6, interpolation=cv2.INTER_NEAREST)
cv2.imwrite("bird_depth_zoom.png", crop)
