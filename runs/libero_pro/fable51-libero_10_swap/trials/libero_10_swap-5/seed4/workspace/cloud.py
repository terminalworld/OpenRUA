import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from cv_bridge import CvBridge
import cv2

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(cam):
    rclpy.init(); node = rclpy.create_node("cloud")
    got = {}
    node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
    node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
    node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
    qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    def cb(m):
        for t in m.transforms:
            if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
    node.create_subscription(TFMessage, "/tf_static", cb, qos)
    node.create_subscription(TFMessage, "/tf", cb, 100)
    while not all(k in got for k in "dci") or "tf" not in got:
        rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()
    d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
    c = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
    k = got["i"].k; tr = got["tf"]
    T = np.eye(4); T[:3,:3] = quat_R(tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w)
    T[:3,3] = [tr.translation.x,tr.translation.y,tr.translation.z]
    H,W = d.shape
    u,v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u-k[2])*d/k[0]; Y = (v-k[5])*d/k[4]
    P = np.stack([X,Y,d,np.ones_like(d)],-1) @ T.T
    return P[...,:3], c, d

if __name__ == "__main__":
    cam = sys.argv[1]
    P, c, d = grab(cam)
    np.save(f"{cam}_xyz.npy", P)
    z = P[...,2]
    print("z range", np.nanmin(z), np.nanmax(z))
    # histogram of z
    hist, edges = np.histogram(z[np.isfinite(z)], bins=60)
    for h,e in zip(hist, edges): 
        if h>50: print(f"{e:.3f} {h}")
