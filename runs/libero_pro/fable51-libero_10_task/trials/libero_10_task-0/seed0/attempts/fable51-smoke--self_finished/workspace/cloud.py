import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
cam=sys.argv[1]
rclpy.init(); n=rclpy.create_node('cloud')
got={}
n.create_subscription(Image,f'/{cam}/depth/image_raw',lambda m:got.setdefault('d',m),1)
n.create_subscription(Image,f'/{cam}/color/image_raw',lambda m:got.setdefault('c',m),1)
n.create_subscription(CameraInfo,f'/{cam}/color/camera_info',lambda m:got.setdefault('i',m),1)
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(TFMessage,'/tf_static',lambda m:[got.setdefault(('tf',t.child_frame_id),t.transform) for t in m.transforms],qos)
while not all(k in got for k in ['d','c','i',('tf',cam+'_optical_frame')]): rclpy.spin_once(n,timeout_sec=0.3)
d=CvBridge().imgmsg_to_cv2(got['d'],'passthrough').astype(np.float32)
c=CvBridge().imgmsg_to_cv2(got['c'],'bgr8')
k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
t=got[('tf',cam+'_optical_frame')]
q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
H,W=d.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)
Pw=P@R.T+T
np.save(f'{cam}_xyz.npy',Pw); np.save(f'{cam}_bgr.npy',c)
print('depth range',np.nanmin(d),np.nanmax(d))
print('world z range',np.nanmin(Pw[...,2]),np.nanmax(Pw[...,2]))
# table height: median z of central region
print('median z centre', np.nanmedian(Pw[200:480,:,2]))
