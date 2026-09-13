"""Build world-coordinate maps for a camera's current depth frame: <cam>_wx/wy/wz.npy"""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
cam=sys.argv[1]
rclpy.init(); n=rclpy.create_node("cammap"); b=Buffer(); TransformListener(b,n)
got={}
n.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault('d',m),1)
n.create_subscription(Image,f"/{cam}/color/image_raw",lambda m:got.setdefault('c',m),1)
n.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault('i',m),1)
while not all(k in got for k in 'dci') or not b.can_transform('world',f'{cam}_optical_frame',rclpy.time.Time()):
    rclpy.spin_once(n,timeout_sec=0.2)
d=CvBridge().imgmsg_to_cv2(got['d'],'passthrough').astype(np.float64)
c=CvBridge().imgmsg_to_cv2(got['c'],'bgr8')
import cv2; cv2.imwrite(f'{cam}.png',c)
k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
t=b.lookup_transform('world',f'{cam}_optical_frame',rclpy.time.Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
H,W=d.shape; v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)
Wp=P@R.T+T
np.save(f'{cam}_wx.npy',Wp[...,0]); np.save(f'{cam}_wy.npy',Wp[...,1]); np.save(f'{cam}_wz.npy',Wp[...,2])
print(cam, 'cam pos',T, 'saved', d.shape)
rclpy.shutdown()
