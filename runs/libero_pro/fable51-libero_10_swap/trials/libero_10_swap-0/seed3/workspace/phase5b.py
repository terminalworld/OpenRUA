from rob import *
from refine import refine
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
r = Robot()
r.gripper(0.04)
ok = r.move_tcp(0.19, 0.31, 0.66, yaw=0.0, secs=5)
if not ok: log("retry"); ok = r.move_tcp(0.19, 0.31, 0.66, yaw=0.0, secs=5)
got={}
s=r.node.create_subscription(Image,"/robot0_eye_in_hand/color/image_raw",lambda m:got.setdefault('c',m),1)
while 'c' not in got: rclpy.spin_once(r.node,timeout_sec=0.2)
cv2.imwrite("eih_tom.png", CvBridge().imgmsg_to_cv2(got['c'],"bgr8"))
res = refine(r, 0.22, 0.34, 0.435, 0.51, rad=0.13)
c, pts = res
xy = pts[:, :2] - pts[:, :2].mean(0)
w, v = np.linalg.eigh(xy.T @ xy)
axis = v[:, 1]; ang2 = math.degrees(math.atan2(axis[1], axis[0]))
log(f"center {c.round(4)} extents x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] axis {ang2:.1f} deg eig {np.sqrt(w/len(pts)).round(4)}")
# top-ridge points only (z > 0.48) give the axis line cleanly
top = pts[pts[:,2] > 0.48]
xy = top[:, :2] - top[:, :2].mean(0); w, v = np.linalg.eigh(xy.T @ xy); axis = v[:, 1]
log(f"ridge n={len(top)} center {top.mean(0).round(4)} axis {math.degrees(math.atan2(axis[1], axis[0])):.1f} deg")
np.save("tom_pts.npy", pts)
log("PHASE5B DONE")
