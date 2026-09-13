"""cloud.py <camera> : save world-frame point cloud (HxWx3) as <camera>_xyz.npy plus color png"""
import sys, numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from cv_bridge import CvBridge
import cv2
cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node('cloud')
got={}
node.create_subscription(Image,f'/{cam}/depth/image_raw',lambda m:got.setdefault('d',m),1)
node.create_subscription(Image,f'/{cam}/color/image_raw',lambda m:got.setdefault('c',m),1)
node.create_subscription(CameraInfo,f'/{cam}/color/camera_info',lambda m:got.setdefault('i',m),1)
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
tfs={}
def cb(msg):
    for t in msg.transforms: tfs[t.child_frame_id]=t.transform
node.create_subscription(TFMessage,'/tf_static',cb,qos)
node.create_subscription(TFMessage,'/tf',cb,10)
t0=time.time()
while (len(got)<3 or f'{cam}_optical_frame' not in tfs) and time.time()-t0<30: rclpy.spin_once(node,timeout_sec=0.2)
b=CvBridge()
depth=b.imgmsg_to_cv2(got['d'],'passthrough').astype(np.float64)
color=b.imgmsg_to_cv2(got['c'],'bgr8')
k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
X=(u-cx)*depth/fx; Y=(v-cy)*depth/fy; Z=depth
t=tfs[f'{cam}_optical_frame']; q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
P=np.stack([X,Y,Z],-1)@R.T+T
np.save(f'{cam}_xyz.npy',P); cv2.imwrite(f'{cam}.png',color)
print('saved',cam, 'depth range',np.nanmin(depth),np.nanmax(depth))
