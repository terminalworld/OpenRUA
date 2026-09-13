"""Grab color+depth+info from a camera, save world-frame point cloud as npz."""
import sys, time, numpy as np, rclpy
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(cam):
    rclpy.init(); n = rclpy.create_node("cloud")
    got = {}
    n.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
    n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
    n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
    def tfcb(m):
        for t in m.transforms:
            if t.child_frame_id == f"{cam}_optical_frame" and t.header.frame_id == "world":
                got["tf"] = t.transform
    qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(TFMessage, "/tf_static", tfcb, qos)
    n.create_subscription(TFMessage, "/tf", tfcb, 100)
    end = time.time()+30
    while len(got) < 4 and time.time() < end: rclpy.spin_once(n, timeout_sec=0.2)
    assert len(got) == 4, got.keys()
    br = CvBridge()
    color = br.imgmsg_to_cv2(got["c"], "rgb8")
    depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float32)
    k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u-cx)*depth/fx; Y = (v-cy)*depth/fy; Z = depth
    P = np.stack([X,Y,Z], -1).reshape(-1,3)
    tf = got["tf"]; R = quat_R(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
    t = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
    Pw = (P @ R.T + t).reshape(H, W, 3)
    rclpy.shutdown()
    np.savez(f"{cam}_cloud.npz", pw=Pw, color=color, depth=depth)
    return Pw, color, depth

if __name__ == "__main__":
    Pw, color, depth = grab(sys.argv[1])
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    print("center px world", Pw[240,320])
