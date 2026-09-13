"""Grab color+depth+info from a camera, return world-frame point cloud helpers."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from cv_bridge import CvBridge
import cv2

def quat_R(x,y,z,w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def grab(node, topic, typ, timeout=20):
    got={}
    sub=node.create_subscription(typ, topic, lambda m: got.setdefault("m",m), 1)
    import time; t0=time.time()
    while "m" not in got and time.time()-t0<timeout: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got: raise SystemExit(f"no msg on {topic}")
    return got["m"]

def cam_T(node, cam):
    seen={}
    def cb(m):
        for t in m.transforms: seen[(t.header.frame_id,t.child_frame_id)]=t.transform
    qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    s=node.create_subscription(TFMessage,"/tf_static",cb,qos)
    s2=node.create_subscription(TFMessage,"/tf",cb,100)
    import time; t0=time.time()
    key=("world",f"{cam}_optical_frame")
    while key not in seen and time.time()-t0<10: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); node.destroy_subscription(s2)
    tr=seen[key]; T=np.eye(4)
    T[:3,:3]=quat_R(tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w)
    T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
    return T

def cloud(cam, node=None):
    own = node is None
    if own:
        rclpy.init(); node=rclpy.create_node("percep")
    color=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/color/image_raw",Image),"bgr8")
    depth=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/depth/image_raw",Image),"passthrough").astype(np.float32)
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    T=cam_T(node,cam)
    if own: rclpy.shutdown()
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    h,w=depth.shape
    u,v=np.meshgrid(np.arange(w),np.arange(h))
    X=(u-cx)*depth/fx; Y=(v-cy)*depth/fy
    P=np.stack([X,Y,depth,np.ones_like(depth)],-1) @ T.T
    return color, depth, P[...,:3]

if __name__=="__main__":
    cam=sys.argv[1]
    color,depth,P=cloud(cam)
    np.save(f"snaps/{cam}_P.npy",P); cv2.imwrite(f"snaps/{cam}.png",color)
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    for a in sys.argv[2:]:
        u,v=map(int,a.split(","))
        print(f"px({u},{v}) -> world {P[v,u]}")
