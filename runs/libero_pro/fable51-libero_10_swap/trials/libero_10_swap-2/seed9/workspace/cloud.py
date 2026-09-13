#!/usr/bin/env python3
"""cloud.py <camera> -> saves <camera>_cloud.npy (H,W,3) world xyz + <camera>_depth.npy"""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node("cloud"); buf=Buffer(); TransformListener(buf,node)
def grab(topic,T):
    got={}; s=node.create_subscription(T,topic,lambda m:got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
d=grab(f"/{cam}/depth/image_raw",Image); info=grab(f"/{cam}/color/camera_info",CameraInfo)
depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
frame=f"{cam}_optical_frame"
while not buf.can_transform("world",frame,Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform("world",frame,Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tt=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
v,u=np.mgrid[0:d.height,0:d.width]
P=np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth],-1)
W=P@R.T+tt
np.save(f"{cam}_cloud.npy",W.astype(np.float32)); np.save(f"{cam}_depth.npy",depth)
print(cam, "cam pos", tt, "fx",fx, "size", d.width, d.height)
rclpy.shutdown()
