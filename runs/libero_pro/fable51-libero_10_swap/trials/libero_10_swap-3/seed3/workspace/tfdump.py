import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def cb(m, k):
    for t in m.transforms:
        got[(k, t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf_static", lambda m: cb(m, "static"), qos)
node.create_subscription(TFMessage, "/tf", lambda m: cb(m, "dyn"), 10)
import time
t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(node, timeout_sec=0.2)
for k, t in sorted(got.items()):
    print(k, f"t=({t.translation.x:.3f},{t.translation.y:.3f},{t.translation.z:.3f}) q=({t.rotation.x:.3f},{t.rotation.y:.3f},{t.rotation.z:.3f},{t.rotation.w:.3f})")
