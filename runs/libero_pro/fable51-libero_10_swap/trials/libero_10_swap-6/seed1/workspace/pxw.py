"""pxw.py <camera> u,v [u,v ...] : print world xyz for several pixels using one grab."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy

def R_of(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(node, topic, T):
    got={}
    sub=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub); return got["m"]

def cam_pose(node, frame):
    got={}
    def cb(m):
        for t in m.transforms:
            if t.child_frame_id==frame and t.header.frame_id=="world": got["t"]=t.transform
    qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    s1=node.create_subscription(TFMessage,"/tf_static",cb,qos); s2=node.create_subscription(TFMessage,"/tf",cb,100)
    while "t" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    t=got["t"]; T=np.eye(4); T[:3,:3]=R_of((t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))
    T[:3,3]=[t.translation.x,t.translation.y,t.translation.z]; return T

def main():
    cam=sys.argv[1]; pts=[tuple(map(int,a.split(","))) for a in sys.argv[2:]]
    rclpy.init(); node=rclpy.create_node("pxw")
    depth=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
    T=cam_pose(node,f"{cam}_optical_frame")
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    for u,v in pts:
        z=float(D[v,u]); p=T@np.array([(u-cx)*z/fx,(v-cy)*z/fy,z,1])
        print(f"({u},{v}) depth={z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()
main()
