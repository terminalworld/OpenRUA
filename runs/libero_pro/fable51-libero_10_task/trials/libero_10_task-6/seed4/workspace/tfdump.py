import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(TFMessage, "/tf_static", cb, qos)
n.create_subscription(TFMessage, "/tf", cb, 100)
end = time.time()+5
while time.time()<end: rclpy.spin_once(n, timeout_sec=0.2)
for k,v in sorted(seen.items()):
    tr, q = v.translation, v.rotation
    print(f"{k[0]:>28} -> {k[1]:<28} t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
