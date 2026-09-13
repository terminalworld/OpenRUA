import rclpy, yaml
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
for _ in range(30): rclpy.spin_once(node, timeout_sec=0.2)
for k, t in got.items():
    print(k, "t=(%.4f %.4f %.4f)" % (t.translation.x, t.translation.y, t.translation.z), "q=(%.4f %.4f %.4f %.4f)" % (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
