import rclpy, numpy as np, sys, cv2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
cam = sys.argv[1]
rclpy.init(); n=rclpy.create_node("cloud"); b=Buffer(); TransformListener(b,n)
got={}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d",m),1)
n.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c",m),1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i",m),1)
while len(got)<3 or not b.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()):
    rclpy.spin_once(n, timeout_sec=0.2)
br=CvBridge()
d=br.imgmsg_to_cv2(got["d"],"passthrough").astype(np.float64)
c=br.imgmsg_to_cv2(got["c"],"bgr8")
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
t=b.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
H,W=d.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1).reshape(-1,3)
Pw=(P@R.T+T).reshape(H,W,3)
np.save(f"{cam}_xyz.npy",Pw); np.save(f"{cam}_bgr.npy",c)
print("intrinsics",fx,fy,cx,cy,"size",W,H)
zs=Pw[...,2]; print("z range",np.nanmin(zs),np.nanmax(zs))
# table height guess: mode of z
hist,edges=np.histogram(zs[np.isfinite(zs)],bins=200)
i=hist.argmax(); print("dominant z",edges[i],edges[i+1])
