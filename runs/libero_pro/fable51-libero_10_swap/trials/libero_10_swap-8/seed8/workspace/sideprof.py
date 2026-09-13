import numpy as np, rclpy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
def R_of(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rclpy.init(); node=rclpy.create_node("sp")
got={}
def grab(topic,T):
    got.pop('m',None)
    s=node.create_subscription(T,topic,lambda m: got.setdefault('m',m),1)
    while 'm' not in got: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']
cam="sideview"; t=np.array([-0.0565,1.2761,1.488]); q=(0.0099,0.8064,-0.5912,-0.0069)
d=CvBridge().imgmsg_to_cv2(grab(f"/{cam}/depth/image_raw",Image),"passthrough").astype(float)
info=grab(f"/{cam}/color/camera_info",CameraInfo)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1)@R_of(q).T+t
np.save("side_world.npy",P)
for name,(x0,x1,y0,y1) in {"potB":(-0.13,0.0,0.13,0.31),"potA":(-0.26,-0.13,-0.30,-0.12)}.items():
    m=(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.895)
    pts=P[m]; print(name,len(pts))
    for z in np.arange(0.89,1.07,0.01):
        s=pts[(pts[:,2]>=z)&(pts[:,2]<z+0.01)]
        if len(s)>3: print(f"  z {z:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] w={s[:,0].max()-s[:,0].min():.3f} y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
