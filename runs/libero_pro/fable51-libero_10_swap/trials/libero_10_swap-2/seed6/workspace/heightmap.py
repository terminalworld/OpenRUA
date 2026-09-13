"""Build a world-frame height map from a top-down camera's depth frame and report object blobs."""
import sys, numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2
cam = sys.argv[1] if len(sys.argv)>1 else "birdview"
rclpy.init(); node = rclpy.create_node("hm")
tfbuf = Buffer(); TransformListener(tfbuf, node)
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
frame = f"{cam}_optical_frame"
end = time.time()+20
while time.time()<end and not ("d" in got and "i" in got and "c" in got and tfbuf.can_transform("world", frame, rclpy.time.Time())):
    rclpy.spin_once(node, timeout_sec=0.2)
d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
info = got["i"]; fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
H,W = d.shape
u,v = np.meshgrid(np.arange(W), np.arange(H))
pc = np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)
pw = pc @ R.T + T
np.save(f"{cam}_world.npy", pw)
hz = pw[...,2]
print("height range", np.nanmin(hz), np.nanmax(hz))
mask = ((hz > 0.905) & (hz < 1.5)).astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA] < 15: continue
    m = lab==i
    xs, ys, zs = pw[...,0][m], pw[...,1][m], hz[m]
    print(f"blob {i}: area={stats[i,cv2.CC_STAT_AREA]} px centroid=({cent[i][0]:.0f},{cent[i][1]:.0f}) "
          f"x=[{xs.min():.3f},{xs.max():.3f}] y=[{ys.min():.3f},{ys.max():.3f}] zmax={zs.max():.3f} xmean={xs.mean():.3f} ymean={ys.mean():.3f}")
