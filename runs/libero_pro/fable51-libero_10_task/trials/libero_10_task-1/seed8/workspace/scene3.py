import numpy as np, rclpy, sys, time, cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
cam=sys.argv[1]; zthr=float(sys.argv[2]) if len(sys.argv)>2 else 0.44
rclpy.init(); n=rclpy.create_node("scene3"); b=Buffer(); TransformListener(b,n)
got={}
n.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
n.create_subscription(Image,f"/{cam}/color/image_raw",lambda m:got.setdefault("c",m),1)
n.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
t0=time.time()
while len(got)<3 or not b.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()):
    rclpy.spin_once(n,timeout_sec=0.2)
    if time.time()-t0>30: raise SystemExit("timeout")
br=CvBridge()
d=br.imgmsg_to_cv2(got["d"],"passthrough").astype(np.float32)
c=br.imgmsg_to_cv2(got["c"],"bgr8")
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
tr=b.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()).transform
q=tr.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t=np.array([tr.translation.x,tr.translation.y,tr.translation.z])
H,W=d.shape; us,vs=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1)@R.T+t
np.save(f"{cam}_world.npy",P); np.save(f"{cam}_depth.npy",d); cv2.imwrite(f"{cam}.png",c)
zw=P[...,2]
mask=(zw>zthr)&np.isfinite(zw)&(zw<0.75)&(d<2.5)
num,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,num):
    if stats[i,4]<10: continue
    m=lab==i; pts=P[m]; col=c[m].mean(0)
    print(f"blob {i}: area {stats[i,4]} px({cents[i][0]:.0f},{cents[i][1]:.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] bgr {col.astype(int)}")
