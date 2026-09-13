"""Segment objects above the table from birdview depth; print world-frame clusters."""
import rclpy, numpy as np, sys
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from scipy import ndimage

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]

rclpy.init(); node=rclpy.create_node("scene")
cam = sys.argv[1] if len(sys.argv)>1 else "birdview"
d = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(float)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
col = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
H,W = d.shape
v,u = np.mgrid[0:H,0:W]
X=(u-cx)*d/fx; Y=(v-cy)*d/fy; Z=d
# birdview: world = (-0.2,0,3.0), q=(0.7071,0.7071,0,0): R maps cam->world
q=np.array([0.7071,0.7071,0,0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
P = np.stack([X,Y,Z],-1) @ R.T + np.array([-0.2,0,3.0])
wz = P[...,2]
np.save("birdview_world.npy", P)
mask = (wz > 0.905) & (wz < 1.3) & np.isfinite(wz)
lab, n = ndimage.label(mask)
for i in range(1,n+1):
    m = lab==i
    if m.sum() < 15: continue
    pts = P[m]
    c = pts.mean(0); mn=pts.min(0); mx=pts.max(0)
    us,vs = u[m], v[m]
    print(f"cluster {i}: n={m.sum()} px=({us.mean():.0f},{vs.mean():.0f}) centroid=({c[0]:.3f},{c[1]:.3f}) "
          f"x[{mn[0]:.3f},{mx[0]:.3f}] y[{mn[1]:.3f},{mx[1]:.3f}] ztop={mx[2]:.3f} color={col[m].mean(0).astype(int)}")
