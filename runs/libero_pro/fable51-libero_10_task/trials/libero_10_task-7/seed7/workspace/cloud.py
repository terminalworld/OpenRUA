"""Grab color+depth+info from a camera, produce world-frame point cloud; save npz."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(node, topic, T, timeout=30):
    got={}
    sub=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    import time; t0=time.time()
    while "m" not in got and time.time()-t0<timeout: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

def main():
    cam=sys.argv[1]
    rclpy.init(); node=rclpy.create_node("cloud")
    tfs={}
    qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    def cb(m):
        for t in m.transforms: tfs[(t.header.frame_id,t.child_frame_id)]=t.transform
    node.create_subscription(TFMessage,"/tf_static",cb,qos)
    node.create_subscription(TFMessage,"/tf",cb,100)
    depth=grab(node,f"/{cam}/depth/image_raw",Image)
    color=grab(node,f"/{cam}/color/image_raw",Image)
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    import time; t0=time.time()
    while ("world",f"{cam}_optical_frame") not in tfs and time.time()-t0<10: rclpy.spin_once(node,timeout_sec=0.2)
    t=tfs[("world",f"{cam}_optical_frame")]
    R=quat_R(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w); p=np.array([t.translation.x,t.translation.y,t.translation.z])
    br=CvBridge()
    D=br.imgmsg_to_cv2(depth,"passthrough").astype(np.float64)
    C=br.imgmsg_to_cv2(color,"bgr8")
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    H,W=D.shape
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    X=(u-cx)*D/fx; Y=(v-cy)*D/fy
    P=np.stack([X,Y,D],-1)@R.T+p
    np.savez(f"{cam}_cloud.npz",P=P,C=C,D=D)
    print("saved", f"{cam}_cloud.npz", "depth range", np.nanmin(D), np.nanmax(D))
    rclpy.shutdown()
main()
