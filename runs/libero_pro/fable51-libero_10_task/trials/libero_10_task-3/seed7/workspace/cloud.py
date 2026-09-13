"""Usage: cloud.py <camera> -> saves <camera>_cloud.npy (H,W,3) world xyz, and <camera>_rgb.npy"""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def main():
    cam=sys.argv[1]
    rclpy.init(); node=rclpy.create_node("cloud")
    got={}
    node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d",m),1)
    node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c",m),1)
    node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i",m),1)
    qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def tfcb(m):
        for t in m.transforms:
            if t.child_frame_id==f"{cam}_optical_frame" and t.header.frame_id=="world": got["t"]=t
    node.create_subscription(TFMessage,"/tf_static",tfcb,qos); node.create_subscription(TFMessage,"/tf",tfcb,100)
    while not all(k in got for k in "dict"): rclpy.spin_once(node,timeout_sec=0.2)
    d=got["d"]; info=got["i"]; t=got["t"]; c=got["c"]
    depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
    rgb=np.frombuffer(c.data,dtype=np.uint8).reshape(c.height,c.width,-1)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    R=quat_R(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
    T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    v,u=np.mgrid[0:d.height,0:d.width]
    pc=np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth],-1)
    pw=pc@R.T+T
    np.save(f"{cam}_cloud.npy",pw); np.save(f"{cam}_rgb.npy",rgb)
    print(cam, pw.shape, "enc", c.encoding)
    rclpy.shutdown()
main()
