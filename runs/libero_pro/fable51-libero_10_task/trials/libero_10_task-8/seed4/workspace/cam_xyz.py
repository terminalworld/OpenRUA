"""Save world-frame XYZ per pixel for a camera: <cam>_xyz.npy"""
import numpy as np, rclpy, sys
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node("cx"); buf=Buffer(); TransformListener(buf,node)
def grab(topic,T):
    got={}; s=node.create_subscription(T,topic,lambda m:got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
d=CvBridge().imgmsg_to_cv2(grab(f"/{cam}/depth/image_raw",Image),"passthrough").astype(float)
info=grab(f"/{cam}/color/camera_info",CameraInfo)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
frame=f"{cam}_optical_frame"
while not buf.can_transform("world",frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform("world",frame,rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
pc=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1)
P=pc@R.T+tr
np.save(f"{cam}_xyz.npy",P); print(cam,"cam at",tr, "saved")
