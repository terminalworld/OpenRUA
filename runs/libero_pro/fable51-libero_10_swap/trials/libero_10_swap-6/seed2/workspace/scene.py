"""Grab color+depth+info for a camera, save, and provide pixel->world."""
import sys, struct, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge

def quat_R(x,y,z,w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def grab(cam):
    rclpy.init(); node = rclpy.create_node("scene")
    got = {}
    node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
    node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
    node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
    def tfcb(m):
        for t in m.transforms:
            if t.child_frame_id == f"{cam}_optical_frame" and t.header.frame_id == "world":
                got["tf"] = t.transform
    node.create_subscription(TFMessage, "/tf", tfcb, 10)
    while not all(k in got for k in "cdi") or "tf" not in got:
        rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()
    br = CvBridge()
    color = br.imgmsg_to_cv2(got["c"], "bgr8")
    depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float32)
    K = np.array(got["i"].k).reshape(3,3)
    t = got["tf"]; T = np.eye(4)
    T[:3,:3] = quat_R(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)
    T[:3,3] = [t.translation.x,t.translation.y,t.translation.z]
    return color, depth, K, T

def px2world(u, v, depth, K, T):
    z = depth[v,u]
    p = np.array([(u-K[0,2])*z/K[0,0], (v-K[1,2])*z/K[1,1], z, 1.0])
    return (T@p)[:3]

def cloud(depth, K, T):
    h,w = depth.shape
    vs,us = np.mgrid[0:h,0:w]
    z = depth
    X = (us-K[0,2])*z/K[0,0]; Y = (vs-K[1,2])*z/K[1,1]
    P = np.stack([X,Y,z,np.ones_like(z)],-1) @ T.T
    return P[...,:3]

if __name__ == "__main__":
    cam = sys.argv[1]
    color, depth, K, T = grab(cam)
    cv2.imwrite(f"snaps/{cam}.png", color); np.save(f"snaps/{cam}_depth.npy", depth)
    np.save(f"snaps/{cam}_K.npy", K); np.save(f"snaps/{cam}_T.npy", T)
    print("K", K.tolist()); print("T", T.tolist())
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    for a in sys.argv[2:]:
        u,v = map(int, a.split(","))
        print(f"px({u},{v}) depth={depth[v,u]:.4f} world={px2world(u,v,depth,K,T)}")
