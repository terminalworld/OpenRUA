"""Grab color+depth+info for a camera, return world-frame XYZ per pixel."""
import numpy as np, rclpy, yaml
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(node, topic, typ, timeout=30):
    got={}
    s=node.create_subscription(typ, topic, lambda m: got.setdefault('m',m), 1)
    import time; t0=time.time()
    while 'm' not in got and time.time()-t0<timeout: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s)
    if 'm' not in got: raise SystemExit('no msg on '+topic)
    return got['m']

def cam_tf(node, cam, timeout=10):
    qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    got={}
    def cb(m):
        for t in m.transforms:
            if t.child_frame_id==f'{cam}_optical_frame' and t.header.frame_id=='world': got['t']=t
    s=node.create_subscription(TFMessage,'/tf_static',cb,qos)
    s2=node.create_subscription(TFMessage,'/tf',cb,10)
    import time; t0=time.time()
    while 't' not in got and time.time()-t0<timeout: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); node.destroy_subscription(s2)
    t=got['t'].transform; q=t.rotation
    T=np.eye(4); T[:3,:3]=quat_R(q.x,q.y,q.z,q.w); T[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    return T

def cloud(cam, node=None):
    own = node is None
    if own:
        rclpy.init(); node=rclpy.create_node('cloud_'+cam)
    br=CvBridge()
    color=br.imgmsg_to_cv2(grab(node,f'/{cam}/color/image_raw',Image),'bgr8')
    depth=br.imgmsg_to_cv2(grab(node,f'/{cam}/depth/image_raw',Image),'passthrough').astype(np.float32)
    info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
    T=cam_tf(node,cam)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    h,w=depth.shape
    u,v=np.meshgrid(np.arange(w),np.arange(h))
    pc=np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth,np.ones_like(depth)],-1)
    W=(pc.reshape(-1,4)@T.T)[:,:3].reshape(h,w,3)
    if own: node.destroy_node(); rclpy.shutdown()
    return color, depth, W, (fx,fy,cx,cy), T
