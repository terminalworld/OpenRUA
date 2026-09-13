import rclpy, sys
from tf2_msgs.msg import TFMessage
rclpy.init(); node = rclpy.create_node("tfd")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf", cb, 50)
from rclpy.qos import QoSProfile, DurabilityPolicy
qos = QoSProfile(depth=50); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
node.create_subscription(TFMessage, "/tf_static", cb, qos)
import time
t0=time.time()
while time.time()-t0 < 5: rclpy.spin_once(node, timeout_sec=0.2)
for k,v in sorted(seen.items()):
    tr, q = v.translation, v.rotation
    print(f"{k[0]:>22} -> {k[1]:<28} t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
