"""Grab depth+info+TF for a camera once; save world-XYZ map as npy."""
import sys, numpy as np, rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
cam = sys.argv[1]
rclpy.init(); n = rclpy.create_node("dm"); buf=Buffer(); TransformListener(buf,n)
got={}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d",m),1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i",m),1)
while len(got)<2 or not buf.can_transform("world", f"{cam}_optical_frame", Time()):
    rclpy.spin_once(n, timeout_sec=0.2)
d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
k = got["i"].k; fx,fy,cx,cy = k[0],k[4],k[2],k[5]
H,W = d.shape
u,v = np.meshgrid(np.arange(W), np.arange(H))
X = (u-cx)*d/fx; Y=(v-cy)*d/fy; Z=d
t = buf.lookup_transform("world", f"{cam}_optical_frame", Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr = np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
P = np.stack([X,Y,Z],-1) @ R.T + tr
np.save(f"{cam}_xyz.npy", P)
print(cam, d.shape, "fx",fx,"depth range",np.nanmin(d),np.nanmax(d))
