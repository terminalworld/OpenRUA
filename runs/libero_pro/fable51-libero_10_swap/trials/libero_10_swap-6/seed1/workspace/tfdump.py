import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
def cb(m):
    for t in m.transforms:
        got[(t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
for _ in range(30): rclpy.spin_once(node, timeout_sec=0.2)
for k, t in sorted(got.items()):
    print(k, f"t=({t.translation.x:.3f},{t.translation.y:.3f},{t.translation.z:.3f}) q=({t.rotation.x:.3f},{t.rotation.y:.3f},{t.rotation.z:.3f},{t.rotation.w:.3f})")
