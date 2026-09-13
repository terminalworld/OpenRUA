import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
rclpy.init(); n = rclpy.create_node("tfdump")
qos = QoSProfile(depth=10); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
got = {}
def cb(m):
    for t in m.transforms:
        got[(t.header.frame_id, t.child_frame_id)] = t.transform
n.create_subscription(TFMessage, "/tf_static", cb, qos)
n.create_subscription(TFMessage, "/tf", cb, 10)
import time
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.2)
for k, t in got.items():
    print(k, f"t=({t.translation.x:.3f},{t.translation.y:.3f},{t.translation.z:.3f}) q=({t.rotation.x:.3f},{t.rotation.y:.3f},{t.rotation.z:.3f},{t.rotation.w:.3f})")
import numpy as np
from lib import quat_R
chain = [("world","panda_link0"),("panda_link0","panda_link1"),("panda_link1","panda_link2"),("panda_link2","panda_link3"),("panda_link3","panda_link4"),("panda_link4","panda_link5"),("panda_link5","panda_link6"),("panda_link6","panda_link7"),("panda_link7","panda_link8"),("panda_link8","panda_hand")]
T = np.eye(4)
for k in chain:
    t = got[k]; A = np.eye(4); A[:3,:3] = quat_R([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]); A[:3,3] = [t.translation.x,t.translation.y,t.translation.z]
    T = T @ A
    print(k[1], np.round(T[:3,3],4))
