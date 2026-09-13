import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']

def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node('pc')
buf=Buffer(); TransformListener(buf,node)
depth=grab(node,f'/{cam}/depth/image_raw',Image)
color=grab(node,f'/{cam}/color/image_raw',Image)
info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
D=CvBridge().imgmsg_to_cv2(depth,'passthrough').astype(np.float64)
C=CvBridge().imgmsg_to_cv2(color,'bgr8')
frame=f'{cam}_optical_frame'
while not buf.can_transform('world',frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',frame,rclpy.time.Time())
q=t.transform.rotation; R=qR(q.x,q.y,q.z,q.w)
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=D.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D],-1)
Wp=P@R.T+tr
np.save(f'{cam}_world.npy',Wp)
print('cam pos',tr, 'depth range',np.nanmin(D),np.nanmax(D))
print('K',fx,fy,cx,cy,'size',W,H)
rclpy.shutdown()
