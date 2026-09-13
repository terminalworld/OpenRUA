import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import time
cam = sys.argv[1] if len(sys.argv)>1 else "birdview"
rclpy.init(); node = rclpy.create_node("loc")
tfbuf = Buffer(); TransformListener(tfbuf, node)
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
end=time.time()+20
while time.time()<end and not ("d" in got and "c" in got and "i" in got and tfbuf.can_transform("world", f"{cam}_optical_frame", rclpy.time.Time())):
    rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx,fy,cx,cy = k[0],k[4],k[2],k[5]
t = tfbuf.lookup_transform("world", f"{cam}_optical_frame", rclpy.time.Time())
q = t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
H,W = depth.shape
vv,uu = np.mgrid[0:H,0:W]
P = np.stack([(uu-cx)*depth/fx,(vv-cy)*depth/fy,depth],-1) @ R.T + T
np.save(f"{cam}_world.npy", P)
print("depth range", np.nanmin(depth), np.nanmax(depth))
# table height estimate: median z of central region
zc = P[...,2]
print("median z center", np.median(zc[200:400, 200:450]))
for (u,v) in [(328,318),(283,268),(353,270),(320,284),(320,200),(320,400)]:
    print((u,v), "depth", depth[v,u], "world", P[v,u].round(4), "bgr", color[v,u])
