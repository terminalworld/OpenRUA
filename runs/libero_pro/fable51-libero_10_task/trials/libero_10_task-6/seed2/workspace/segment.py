#!/usr/bin/env python3
"""Segment objects above the table in a camera's depth image; print world-frame blobs."""
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cam=sys.argv[1]; table_z=float(sys.argv[2]) if len(sys.argv)>2 else 0.426
rclpy.init(); node=rclpy.create_node('seg')
buf=Buffer(); TransformListener(buf,node)
depth=grab(node,f'/{cam}/depth/image_raw',Image)
color=grab(node,f'/{cam}/color/image_raw',Image)
info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
C=CvBridge().imgmsg_to_cv2(color,'bgr8')
frame=f'{cam}_optical_frame'
while not buf.can_transform('world',frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',frame,rclpy.time.Time()); q=t.transform.rotation
R=qR(q.x,q.y,q.z,q.w); tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=D.shape
vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D],-1).reshape(-1,3)
Wp=(P@R.T+tr).reshape(H,W,3)
np.save(f'{cam}_world.npy',Wp)
mask=((Wp[...,2]>table_z+0.008)&(Wp[...,2]<table_z+0.35)&np.isfinite(D)).astype(np.uint8)
# limit to table area
mask&=((np.abs(Wp[...,0])<0.6)&(np.abs(Wp[...,1])<0.6)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<15: continue
    m=lab==i
    pts=Wp[m]; col=C[m].mean(0)
    x,y,w,h=stats[i,:4]
    print(f'blob{i}: px area={stats[i,4]} bbox=({x},{y},{w},{h}) ctr_px=({cent[i][0]:.0f},{cent[i][1]:.0f}) '
          f'world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] '
          f'zmax={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) bgr={col.astype(int)}')
rclpy.shutdown()
