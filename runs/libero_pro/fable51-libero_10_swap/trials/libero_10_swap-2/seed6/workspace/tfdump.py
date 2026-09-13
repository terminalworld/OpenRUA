import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
n.create_subscription(TFMessage, "/tf", cb, 50)
n.create_subscription(TFMessage, "/tf_static", cb, QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))
end = time.time()+5
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.2)
for (p,c),t in sorted(seen.items()):
    print(f"{p:>28} -> {c:<28} t=({t.translation.x:.4f},{t.translation.y:.4f},{t.translation.z:.4f}) q=({t.rotation.x:.4f},{t.rotation.y:.4f},{t.rotation.z:.4f},{t.rotation.w:.4f})")
