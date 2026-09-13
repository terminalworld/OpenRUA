"""Usage: pxw.py <camera> u,v [u,v ...]  -> world xyz for each pixel (uses TF from world->camera optical frame)."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def main():
    cam=sys.argv[1]; pxs=[tuple(map(int,a.split(","))) for a in sys.argv[2:]]
    rclpy.init(); node=rclpy.create_node("pxw")
    got={}
    node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d",m),1)
    node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i",m),1)
    qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def tfcb(m):
        for t in m.transforms:
            if t.child_frame_id==f"{cam}_optical_frame" and t.header.frame_id=="world": got["t"]=t
    node.create_subscription(TFMessage,"/tf_static",tfcb,qos); node.create_subscription(TFMessage,"/tf",tfcb,100)
    while not all(k in got for k in "dit"): rclpy.spin_once(node,timeout_sec=0.2)
    d=got["d"]; info=got["i"]; t=got["t"]
    depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    R=quat_R(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
    T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    for u,v in pxs:
        z=depth[v,u]
        p=R@np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])+T
        print(f"px({u},{v}) depth={z:.4f} world=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})")
    rclpy.shutdown()
main()
