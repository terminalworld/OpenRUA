#!/usr/bin/env python3
"""Grab depth+color+info from a camera, save world-frame XYZ array (H,W,3) as <cam>_xyz.npy and color png."""
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
cam = sys.argv[1]
rclpy.init(); n=rclpy.create_node('cloud'); buf=Buffer(); TransformListener(buf,n)
got={}
def sub(topic,T,key): n.create_subscription(T,topic,lambda m: got.setdefault(key,m),1)
sub(f'/{cam}/depth/image_raw',Image,'d'); sub(f'/{cam}/color/image_raw',Image,'c'); sub(f'/{cam}/color/camera_info',CameraInfo,'i')
frame=f'{cam}_optical_frame'
while len(got)<3 or not buf.can_transform('world',frame,rclpy.time.Time()): rclpy.spin_once(n,timeout_sec=0.2)
br=CvBridge()
depth=br.imgmsg_to_cv2(got['d'],'passthrough').astype(np.float64)
color=br.imgmsg_to_cv2(got['c'],'bgr8')
k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
X=(u-cx)*depth/fx; Y=(v-cy)*depth/fy; Z=depth
t=buf.lookup_transform('world',frame,rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
P=np.stack([X,Y,Z],-1)@R.T+tr
np.save(f'{cam}_xyz.npy',P); cv2.imwrite(f'{cam}.png',color)
print('saved',P.shape, 'depth range',np.nanmin(depth),np.nanmax(depth))
