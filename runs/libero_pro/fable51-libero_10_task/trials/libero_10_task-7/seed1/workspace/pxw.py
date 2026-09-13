"""Usage: python3 pxw.py <camera> u,v [u,v ...]  -> world xyz for each pixel (depth+intrinsics+TF)."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
def grab(node, topic, T, qos=1):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), qos)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node("pxw")
depth=grab(node,f"/{cam}/depth/image_raw",Image)
info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
frame=f"{cam}_optical_frame"
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
seen={}
def cb(m):
    for t in m.transforms: seen[t.child_frame_id]=t
node.create_subscription(TFMessage,"/tf_static",cb,qos)
node.create_subscription(TFMessage,"/tf",cb,100)
import time; end=time.time()+4
while time.time()<end and frame not in seen: rclpy.spin_once(node,timeout_sec=0.2)
tf=seen[frame]
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
R=qR(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w)
t=np.array([tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z])
for a in sys.argv[2:]:
    u,v=map(int,a.split(","))
    z=float(D[v,u])
    p=R@np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])+t
    print(f"({u},{v}) depth={z:.3f} world= {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
np.save(f"{cam}_depth.npy", D)
