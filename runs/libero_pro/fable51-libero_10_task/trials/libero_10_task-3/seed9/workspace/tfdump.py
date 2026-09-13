import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
seen = {}
def cb(msg):
    for t in msg.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
t0=time.time()
while time.time()-t0 < 4: rclpy.spin_once(node, timeout_sec=0.2)
for (p,c),t in sorted(seen.items()):
    tr=t.transform.translation; q=t.transform.rotation
    print(f"{p:28s} -> {c:32s} t=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f}) q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})")
