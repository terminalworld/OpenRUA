#!/usr/bin/env python3
"""cam3d.py <camera> [u v]...  : world xyz for pixels; also table plane estimate & world->panda_link0."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def grab(node, topic, T, timeout=30):
    got={}
    sub=node.create_subscription(T, topic, lambda m: got.setdefault('m',m),1)
    import time; t0=time.time()
    while 'm' not in got and time.time()-t0<timeout: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(sub); return got.get('m')

def tfmat(t):
    q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]; return T

cam=sys.argv[1]; px=[(int(sys.argv[i]),int(sys.argv[i+1])) for i in range(2,len(sys.argv)-1,2)]
rclpy.init(); node=rclpy.create_node('cam3d'); buf=Buffer(); TransformListener(buf,node)
depth=CvBridge().imgmsg_to_cv2(grab(node,f'/{cam}/depth/image_raw',Image),'passthrough').astype(float)
info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
import time; t0=time.time()
while time.time()-t0<20:
    rclpy.spin_once(node,timeout_sec=0.2)
    if buf.can_transform('world',f'{cam}_optical_frame',rclpy.time.Time()) and buf.can_transform('world','panda_link0',rclpy.time.Time()): break
T=tfmat(buf.lookup_transform('world',f'{cam}_optical_frame',rclpy.time.Time()))
Tb=tfmat(buf.lookup_transform('world','panda_link0',rclpy.time.Time()))
print('world->panda_link0 t=',Tb[:3,3].round(4)); print('cam pos',T[:3,3].round(4))
print('depth shape',depth.shape,'min/max',np.nanmin(depth),np.nanmax(depth))
H,W=depth.shape
vv,uu=np.mgrid[0:H,0:W]
Z=depth; X=(uu-cx)*Z/fx; Y=(vv-cy)*Z/fy
P=np.stack([X,Y,Z,np.ones_like(Z)],-1)@T.T
np.save(f'/workspace/img/{cam}_world.npy',P[...,:3])
zs=P[...,2][np.isfinite(Z)&(Z>0)]
hist,edges=np.histogram(zs,bins=60)
top=np.argsort(hist)[-4:]
print('dominant world-z bins:',[(round(edges[i],3),hist[i]) for i in sorted(top)])
for u,v in px:
    print(f'px({u},{v}) depth={Z[v,u]:.4f} world={P[v,u,:3].round(4)}')
rclpy.shutdown()
