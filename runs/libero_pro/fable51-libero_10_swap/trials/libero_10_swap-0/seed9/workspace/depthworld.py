"""Compute world xyz for every pixel of a camera's depth frame; save npy."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node("dw")
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m: got.setdefault("d",m),1)
node.create_subscription(Image,f"/{cam}/color/image_raw",lambda m: got.setdefault("c",m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m: got.setdefault("i",m),1)
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id==f"{cam}_optical_frame": got["tf"]=t
node.create_subscription(TFMessage,"/tf",tfcb,10)
while not all(k in got for k in "d c i tf".split()): rclpy.spin_once(node,timeout_sec=0.2)
d=CvBridge().imgmsg_to_cv2(got["d"],"passthrough").astype(np.float64)
c=CvBridge().imgmsg_to_cv2(got["c"],"bgr8")
K=np.array(got["i"].k).reshape(3,3)
t=got["tf"].transform; q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
H,W=d.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
X=(u-K[0,2])*d/K[0,0]; Y=(v-K[1,2])*d/K[1,1]
P=np.stack([X,Y,d],-1)@R.T+T
np.save(f"{cam}_world.npy",P); np.save(f"{cam}_rgb.npy",c)
print("K",K.tolist()); print("shape",d.shape, "depth range",np.nanmin(d),np.nanmax(d))
print("center world",P[H//2,W//2])
rclpy.shutdown()
