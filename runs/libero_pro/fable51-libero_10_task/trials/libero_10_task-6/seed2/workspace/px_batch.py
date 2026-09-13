#!/usr/bin/env python3
"""px_batch.py <camera> u,v [u,v ...] -> world xyz per pixel (uses TF from /tf)."""
import struct, sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']

def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node('pxb')
buf=Buffer(); TransformListener(buf,node)
depth=grab(node,f'/{cam}/depth/image_raw',Image)
info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
frame=f'{cam}_optical_frame'
while not buf.can_transform('world',frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',frame,rclpy.time.Time()); q=t.transform.rotation
R=qR(q.x,q.y,q.z,q.w); tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
for a in sys.argv[2:]:
    u,v=map(int,a.split(','))
    z=float(D[v,u])
    p=np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])
    w=R@p+tr
    print(f'({u},{v}) depth={z:.3f} -> world {w[0]:.3f} {w[1]:.3f} {w[2]:.3f}')
rclpy.shutdown()
