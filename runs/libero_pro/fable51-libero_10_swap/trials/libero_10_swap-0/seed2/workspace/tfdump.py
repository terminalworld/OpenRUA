import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
def cb(m, tag):
    for t in m.transforms:
        got[(tag, t.header.frame_id, t.child_frame_id)] = (t.transform.translation, t.transform.rotation)
node.create_subscription(TFMessage, "/tf_static", lambda m: cb(m,"static"), qos)
node.create_subscription(TFMessage, "/tf", lambda m: cb(m,"dyn"), 10)
import time
for _ in range(40): rclpy.spin_once(node, timeout_sec=0.1)
for k,(tr,q) in sorted(got.items()):
    print(k, f"t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
