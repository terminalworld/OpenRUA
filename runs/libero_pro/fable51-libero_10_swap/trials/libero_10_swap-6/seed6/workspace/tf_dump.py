import rclpy
from tf2_msgs.msg import TFMessage
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
def cb(m):
    for t in m.transforms:
        got[(t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf", cb, 10)
from rclpy.qos import QoSProfile, DurabilityPolicy
qos = QoSProfile(depth=10); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
node.create_subscription(TFMessage, "/tf_static", cb, qos)
import time
for _ in range(20): rclpy.spin_once(node, timeout_sec=0.2)
for k, t in sorted(got.items()):
    tr, r = t.translation, t.rotation
    print(f"{k[0]:>28} -> {k[1]:<28} t=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f}) q=({r.x:.3f},{r.y:.3f},{r.z:.3f},{r.w:.3f})")
