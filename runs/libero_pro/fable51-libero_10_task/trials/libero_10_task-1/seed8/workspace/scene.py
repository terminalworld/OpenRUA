import numpy as np, rclpy, struct, sys
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
cam = sys.argv[1] if len(sys.argv)>1 else "birdview"
rclpy.init(); n=rclpy.create_node("scene")
got={}
n.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
n.create_subscription(Image,f"/{cam}/color/image_raw",lambda m:got.setdefault("c",m),1)
n.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
while len(got)<3: rclpy.spin_once(n,timeout_sec=0.2)
br=CvBridge()
d=br.imgmsg_to_cv2(got["d"],"passthrough").astype(np.float32)
c=br.imgmsg_to_cv2(got["c"],"bgr8")
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
np.save(f"{cam}_depth.npy",d)
print("depth shape",d.shape,"min",np.nanmin(d),"max",np.nanmax(d))
# for birdview: cam at (-0.2,0,3.0), optical z down. q=(0.707,0.707,0,0): R = rotation
# compute world from quaternion
q=np.array([0.707107,0.707107,0,0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t=np.array([-0.2,0,3.0])
H,W=d.shape
us,vs=np.meshgrid(np.arange(W),np.arange(H))
X=(us-cx)*d/fx; Y=(vs-cy)*d/fy
P=np.stack([X,Y,d],-1)@R.T+t
np.save(f"{cam}_world.npy",P)
zw=P[...,2]
# table height: mode of z in the central region
cen=zw[200:300,200:450]
hist,edges=np.histogram(cen[np.isfinite(cen)],bins=200)
tz=edges[np.argmax(hist)]
print("table z ~",tz)
mask=(zw>tz+0.01)&np.isfinite(zw)
mask[:, :]&=True
num,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,num):
    if stats[i,4]<15: continue
    m=lab==i
    pts=P[m]
    col=c[m].mean(0)
    print(f"blob {i}: px area {stats[i,4]} centroid px ({cents[i][0]:.0f},{cents[i][1]:.0f}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax {pts[:,2].max():.3f} mean xyz ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f},{pts[:,2].mean():.3f}) bgr {col.astype(int)}")
