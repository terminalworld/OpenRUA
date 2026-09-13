import time, numpy as np, rclpy, sys
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = sys.argv[1] if len(sys.argv) > 1 else "agentview"
rclpy.init(); n = rclpy.create_node("ah")
got = {}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def cb(m):
    for t in m.transforms:
        if t.header.frame_id == "world" and t.child_frame_id == f"{cam}_optical_frame":
            got["tf"] = t.transform
n.create_subscription(TFMessage, "/tf", cb, 50)
t0 = time.time()
while not all(k in got for k in ("d", "i", "tf")) and time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.1)
d, info, t = got["d"], got["i"], got["tf"]
q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],[2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.translation.x, t.translation.y, t.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
vv, uu = np.mgrid[0:d.height, 0:d.width]
P = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1) @ R.T + T
np.save(f"{cam}_P.npy", P)
def box(name, x0, x1, y0, y1):
    s = (P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&np.isfinite(P[...,2])
    zs = P[s][:,2]
    if len(zs)==0: print(name, "none"); return
    print(f"{name}: n={len(zs)} z min={zs.min():.4f} p5={np.percentile(zs,5):.4f} med={np.median(zs):.4f} p95={np.percentile(zs,95):.4f} max={zs.max():.4f}")
box("table front", 0.05, 0.25, -0.1, 0.1)
box("left plate", -0.05, 0.03, -0.35, -0.28)
box("right plate", -0.05, 0.03, 0.27, 0.34)
box("white mug", -0.18, -0.08, -0.24, -0.14)
box("yellow mug", -0.28, -0.18, -0.01, 0.08)
box("red mug", -0.11, -0.01, 0.05, 0.15)
# white mug: width along y at several heights (handle at +y so use x-extent instead)
for z0 in np.arange(0.43, 0.56, 0.01):
    s = (P[...,2]>z0)&(P[...,2]<z0+0.01)&(P[...,0]>-0.20)&(P[...,0]<-0.06)&(P[...,1]>-0.26)&(P[...,1]<-0.12)
    if s.sum() > 3:
        print(f"  white z {z0:.2f}: x[{P[s][:,0].min():.3f},{P[s][:,0].max():.3f}] y[{P[s][:,1].min():.3f},{P[s][:,1].max():.3f}] n={s.sum()}")
