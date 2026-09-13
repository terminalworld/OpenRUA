import rclpy, numpy as np, struct, sys
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]

def cam_model(cam):
    own=not rclpy.ok()
    if own: rclpy.init()
    node=rclpy.create_node("scene_"+str(np.random.randint(1e6)))
    buf=Buffer(); TransformListener(buf,node)
    depth=grab(node,f"/{cam}/depth/image_raw",Image)
    color=grab(node,f"/{cam}/color/image_raw",Image)
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    frame=f"{cam}_optical_frame"
    while not buf.can_transform("world",frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
    t=buf.lookup_transform("world",frame,rclpy.time.Time())
    q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    tt=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    D=CvBridge().imgmsg_to_cv2(depth,"passthrough").astype(np.float64)
    C=CvBridge().imgmsg_to_cv2(color,"bgr8")
    K=np.array(info.k).reshape(3,3)
    node.destroy_node()
    if own: rclpy.shutdown()
    return D,C,K,R,tt

def to_world(D,K,R,tt):
    h,w=D.shape
    u,v=np.meshgrid(np.arange(w),np.arange(h))
    Z=D
    X=(u-K[0,2])*Z/K[0,0]; Y=(v-K[1,2])*Z/K[1,1]
    P=np.stack([X,Y,Z],-1)@R.T+tt
    return P

if __name__=="__main__":
    cam=sys.argv[1]
    D,C,K,R,tt=cam_model(cam)
    P=to_world(D,K,R,tt)
    np.save(f"snaps/{cam}_P.npy",P); cv2.imwrite(f"snaps/{cam}_c.png",C)
    print("K",K.tolist()); print("depth range",np.nanmin(D),np.nanmax(D))
    for a in sys.argv[2:]:
        u,v=map(int,a.split(","))
        print((u,v),"->",np.round(P[v,u],4), "bgr",C[v,u])
