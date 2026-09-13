#!/usr/bin/env python3
"""px_batch.py <camera> u,v [u,v ...]  -> world xyz per pixel (one grab)."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
cam = sys.argv[1]; pts=[tuple(map(int,a.split(","))) for a in sys.argv[2:]]
rclpy.init(); node=rclpy.create_node("pxb"); buf=Buffer(); TransformListener(buf,node)
def grab(topic,T):
    got={}; s=node.create_subscription(T,topic,lambda m:got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
d=grab(f"/{cam}/depth/image_raw",Image); info=grab(f"/{cam}/color/camera_info",CameraInfo)
depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
np.save(f"{cam}_depth.npy",depth)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
frame=f"{cam}_optical_frame"
while not buf.can_transform("world",frame,Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform("world",frame,Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.eye(4);T[:3,:3]=R;T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
for u,v in pts:
    Z=float(depth[v,u]); p=T@np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z,1.0])
    print(f"({u},{v}) depth={Z:.4f} world= {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
rclpy.shutdown()
