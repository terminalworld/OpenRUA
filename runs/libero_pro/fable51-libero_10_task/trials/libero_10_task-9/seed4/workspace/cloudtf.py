"""Snapshot a camera's depth+color+TF and save a world-frame point cloud.
Usage: python3 cloudtf.py <camera> [tag]"""
import sys, time, numpy as np, cv2, rclpy
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge
from rlib import quat_to_R
cam=sys.argv[1]; tag=sys.argv[2] if len(sys.argv)>2 else cam
rclpy.init(); node=rclpy.create_node("cloudtf")
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
node.create_subscription(Image,f"/{cam}/color/image_raw",lambda m:got.setdefault("c",m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
frame=f"{cam}_optical_frame"
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id==frame and t.header.frame_id=="world": got["tf"]=t.transform
node.create_subscription(TFMessage,"/tf",tfcb,50)
node.create_subscription(TFMessage,"/tf_static",tfcb,QoSProfile(depth=50,durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0=time.time()
while not all(k in got for k in "dci") or "tf" not in got:
    rclpy.spin_once(node,timeout_sec=0.2)
    if time.time()-t0>30: raise SystemExit(f"timeout; have {list(got)}")
d=CvBridge().imgmsg_to_cv2(got["d"],"passthrough").astype(np.float32)
c=CvBridge().imgmsg_to_cv2(got["c"],"bgr8")
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
tr=got["tf"]; R=quat_to_R((tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w))
t=np.array([tr.translation.x,tr.translation.y,tr.translation.z])
v,u=np.mgrid[0:d.shape[0],0:d.shape[1]]
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1).reshape(-1,3)@R.T+t
np.save(f"snaps/{tag}_world.npy",P.reshape(d.shape+(3,))); cv2.imwrite(f"snaps/{tag}.png",c)
print("cam at",t,"optical z axis",R[:,2],"saved snaps/%s"%tag)
