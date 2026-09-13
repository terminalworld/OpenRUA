import numpy as np, rclpy, struct, sys
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
rclpy.init(); node = rclpy.create_node("an")
def grab(topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
cam = sys.argv[1] if len(sys.argv)>1 else "birdview"
d = CvBridge().imgmsg_to_cv2(grab(f"/{cam}/depth/image_raw", Image), "passthrough").astype(float)
info = grab(f"/{cam}/color/camera_info", CameraInfo)
fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
print("intrinsics", fx,fy,cx,cy, d.shape)
np.save(f"{cam}_depth.npy", d)
# birdview: cam at (-0.2,0,3), optical z down, image x -> world +y? check via known transform: q=(0.707,0.707,0,0)
# R = rotx(180)*... simply: world_x = -0.2 + (v-cy)*z/fy ; world_y = (u-cx)*z/fx ; world_z = 3 - z  (verified against px2world)
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
Z=3-d; X=-0.2+(vs-cy)*d/fy; Y=(us-cx)*d/fx
np.save("bird_xyz.npy", np.stack([X,Y,Z],-1))
# objects above table: Z>0.905
mask = Z>0.905
import scipy.ndimage as ndi
lab,n=ndi.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<20: continue
    print(f"blob {i}: n={m.sum()} X[{X[m].min():.3f},{X[m].max():.3f}] Y[{Y[m].min():.3f},{Y[m].max():.3f}] Zmax={Z[m].max():.3f} centroid=({X[m].mean():.3f},{Y[m].mean():.3f}) px=({us[m].mean():.0f},{vs[m].mean():.0f})")
