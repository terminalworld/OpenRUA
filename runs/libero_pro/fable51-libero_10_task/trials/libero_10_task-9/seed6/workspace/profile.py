import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = sys.argv[1]
x0,x1,y0,y1 = map(float, sys.argv[2:6])
rclpy.init(); node = rclpy.create_node("prof")
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
q = t.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tt = np.array([t.translation.x,t.translation.y,t.translation.z])
vv, uu = np.mgrid[0:d.height, 0:d.width]
P = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1) @ R.T + tt
np.save(f"{cam}_world.npy", P)
X,Y,Z = P[...,0],P[...,1],P[...,2]
m = (X>x0)&(X<x1)&(Y>y0)&(Y<y1)&(Z>0.905)
pts = P[m]
print(f"n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
for zlo in np.arange(0.90, 1.03, 0.01):
    mm = m & (Z>=zlo)&(Z<zlo+0.01)
    if mm.sum()<3: continue
    p = P[mm]
    print(f"z={zlo:.2f}: n={mm.sum():4d} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ywidth={p[:,1].max()-p[:,1].min():.3f} xwidth={p[:,0].max()-p[:,0].min():.3f}")
