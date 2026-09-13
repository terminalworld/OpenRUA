import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
qos = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def cb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["t"] = t.transform
node.create_subscription(TFMessage, "/tf_static", cb, qos); node.create_subscription(TFMessage, "/tf", cb, 50)
import time
t0=time.time()
while not all(k in got for k in "dit"):
    rclpy.spin_once(node, timeout_sec=0.2)
    if time.time()-t0>20: raise SystemExit("missing: "+str([k for k in "dit" if k not in got]))
d = np.frombuffer(got["d"].data, dtype=np.float32).reshape(got["d"].height, got["d"].width)
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = got["t"]; q = t.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T = np.array([t.translation.x,t.translation.y,t.translation.z])
v,u = np.mgrid[0:d.shape[0],0:d.shape[1]]
P = np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1) @ R.T + T
np.save(f"{cam}_world.npy", P)
print("saved", P.shape)
