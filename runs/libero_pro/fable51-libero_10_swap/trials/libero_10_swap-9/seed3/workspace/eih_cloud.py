import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
cam="robot0_eye_in_hand"
rclpy.init(); node=rclpy.create_node("cloud"); buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
node.create_subscription(Image,f"/{cam}/color/image_raw",lambda m:got.setdefault("c",m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
while len(got)<3 or not buf.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
d=got["d"]; info=got["i"]; c=got["c"]
depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
col=np.frombuffer(c.data,dtype=np.uint8).reshape(c.height,c.width,-1)[:,:,:3]
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
t=buf.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
print("cam at",np.round(T,3),"optical z axis",np.round(R[:,2],3))
vs,us=np.mgrid[0:d.height,0:d.width]
Z=depth
P=np.stack([(us-cx)*Z/fx,(vs-cy)*Z/fy,Z],-1)@R.T+T
np.save("eih_world.npy",P); np.save("eih_col.npy",col)
ok=np.isfinite(Z)&(Z>0.05)&(Z<3)
W=P[ok]
print("world bbox x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f"%(W[:,0].min(),W[:,0].max(),W[:,1].min(),W[:,1].max(),W[:,2].min(),W[:,2].max()))
# interior analysis: points inside microwave footprint x in (-0.265,-0.005), y>-0.325
inside=W[(W[:,0]>-0.26)&(W[:,0]<-0.01)&(W[:,1]>-0.33)&(W[:,1]<-0.10)]
print("n inside",len(inside))
# floor: z histogram
h,e=np.histogram(inside[:,2],bins=np.arange(0.85,1.20,0.01))
for hh,ee in zip(h,e): 
    if hh>0: print(f"  z {ee:.2f}: {hh}")
# back wall: y histogram of points with z between 0.95 and 1.05
mid=inside[(inside[:,2]>0.95)&(inside[:,2]<1.05)]
h,e=np.histogram(mid[:,1],bins=np.arange(-0.33,-0.10,0.01))
print("y hist (z .95-1.05):"); 
for hh,ee in zip(h,e):
    if hh>0: print(f"  y {ee:.2f}: {hh}")
h,e=np.histogram(mid[:,0],bins=np.arange(-0.27,0.0,0.01))
print("x hist (z .95-1.05):")
for hh,ee in zip(h,e):
    if hh>0: print(f"  x {ee:.2f}: {hh}")
