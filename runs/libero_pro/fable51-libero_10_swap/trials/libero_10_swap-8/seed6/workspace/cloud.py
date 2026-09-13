#!/usr/bin/env python3
"""Grab depth+info+TF for a camera, save world-frame point cloud as npy (H,W,3)."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge

def grab(node, topic, typ):
    got={}
    sub=node.create_subscription(typ, topic, lambda m: got.setdefault("m",m), 1)
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.2)
        if "m" in got: break
    node.destroy_subscription(sub)
    return got["m"]

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def main():
    cam=sys.argv[1]
    rclpy.init(); node=rclpy.create_node("cloud")
    tfs={}
    def cb(m):
        for t in m.transforms: tfs[t.child_frame_id]=t
    node.create_subscription(TFMessage,"/tf",cb,10)
    depth=grab(node,f"/{cam}/depth/image_raw",Image)
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    frame=f"{cam}_optical_frame"
    for _ in range(50):
        if frame in tfs: break
        rclpy.spin_once(node,timeout_sec=0.2)
    t=tfs[frame].transform
    R=quat_R(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)
    p0=np.array([t.translation.x,t.translation.y,t.translation.z])
    D=CvBridge().imgmsg_to_cv2(depth,"passthrough").astype(np.float64)
    H,W=D.shape
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    X=(u-cx)*D/fx; Y=(v-cy)*D/fy
    P=np.stack([X,Y,D],-1)@R.T+p0
    np.save(f"{cam}_cloud.npy",P)
    print(cam, "cam pos", p0.round(3), "depth range", np.nanmin(D), np.nanmax(D), "shape", D.shape)
    rclpy.shutdown()
main()
