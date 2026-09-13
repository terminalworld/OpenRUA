import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def cb(m, which):
    for t in m.transforms:
        got[(which, t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf_static", lambda m: cb(m, "static"), qos)
node.create_subscription(TFMessage, "/tf", lambda m: cb(m, "dyn"), 100)
import time
end = time.time() + 8
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
for k, t in sorted(got.items()):
    print(k, f"t=({t.translation.x:.4f},{t.translation.y:.4f},{t.translation.z:.4f}) q=({t.rotation.x:.4f},{t.rotation.y:.4f},{t.rotation.z:.4f},{t.rotation.w:.4f})")
