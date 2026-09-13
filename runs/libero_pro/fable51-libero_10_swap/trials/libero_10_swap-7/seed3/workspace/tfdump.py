import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
def cb(m, k):
    for t in m.transforms:
        got[(k, t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
node.create_subscription(TFMessage, "/tf_static", lambda m: cb(m, "static"), qos)
node.create_subscription(TFMessage, "/tf", lambda m: cb(m, "dyn"), 100)
import time
end = time.time() + 5
while time.time() < end: rclpy.spin_once(node, timeout_sec=0.2)
for (k, p, c), t in sorted(got.items()):
    tr, r = t.translation, t.rotation
    print(f"{k:6s} {p:28s} -> {c:28s} t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({r.x:.4f},{r.y:.4f},{r.z:.4f},{r.w:.4f})")
