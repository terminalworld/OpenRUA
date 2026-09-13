import numpy as np, rclpy, sys
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rclpy.init(); node=rclpy.create_node("cloud")
buf=Buffer(); TransformListener(buf,node)
for cam in sys.argv[1:]:
    depth=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/depth/image_raw",Image),"passthrough").astype(np.float64)
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    while not buf.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
    t=buf.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time())
    q=t.transform.rotation; R=quat_R(q.x,q.y,q.z,q.w); o=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    H,W=depth.shape; v,u=np.mgrid[0:H,0:W]
    pc=np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth],-1)
    np.save(f"{cam}_world.npy",pc@R.T+o)
    print(cam,"saved")
