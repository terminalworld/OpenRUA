import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
sys.path.insert(0, "/workspace")
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
rclpy.init(); n = rclpy.create_node("hm")
got = {}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
qos = QoSProfile(depth=10); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
n.create_subscription(TFMessage, "/tf_static", tfcb, qos); n.create_subscription(TFMessage, "/tf", tfcb, 10)
while len(got) < 3: rclpy.spin_once(n, timeout_sec=0.2)
d, i, t = got["d"], got["i"], got["tf"]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx, fy, cx, cy = i.k[0], i.k[4], i.k[2], i.k[5]
R = quat_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
T = np.array([t.translation.x, t.translation.y, t.translation.z])
vv, uu = np.mgrid[0:d.height, 0:d.width]
P = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1) @ R.T + T
np.save("/workspace/P_%s.npy" % cam, P)
Z = P[...,2]
mask = ((Z > 0.905) & (Z < 1.3) & (P[...,0] > -0.4) & (P[...,0] < 0.4)).astype(np.uint8)
nlab, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for k in range(1, nlab):
    if stats[k, cv2.CC_STAT_AREA] < 30: continue
    m = lab == k
    print(f"comp {k}: area={stats[k,4]} px bbox(u,v,w,h)={stats[k,:4]} x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] y[{P[m][:,1].min():.3f},{P[m][:,1].max():.3f}] zmax={Z[m].max():.3f} centroid=({P[m][:,0].mean():.3f},{P[m][:,1].mean():.3f})")
rclpy.shutdown()
