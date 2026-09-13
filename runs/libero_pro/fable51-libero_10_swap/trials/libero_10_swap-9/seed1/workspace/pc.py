#!/usr/bin/env python3
"""Dump world-frame points for a camera: python3 pc.py <cam> [step]. Saves <cam>_pc.npy (H,W,3) world xyz."""
import struct, sys
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

def grab(node, topic, T, timeout=30.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time; end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("pc")
buf = Buffer(); TransformListener(buf, node)
d = grab(node, f"/{cam}/depth/image_raw", Image)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
frame = f"{cam}_optical_frame"
import time; end=time.time()+20
while time.time()<end and not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q=t.transform.rotation; R=qR(q.x,q.y,q.z,q.w)
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
v,u = np.mgrid[0:d.height,0:d.width]
X=(u-cx)*depth/fx; Y=(v-cy)*depth/fy
P = np.stack([X,Y,depth],-1) @ R.T + tr
np.save(f"{cam}_pc.npy", P)
print("cam pos", tr, "saved", P.shape)
rclpy.shutdown()
