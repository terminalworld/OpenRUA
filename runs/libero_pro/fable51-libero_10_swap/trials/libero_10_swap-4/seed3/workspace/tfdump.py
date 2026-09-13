import rclpy, time
from tf2_msgs.msg import TFMessage
rclpy.init(); n = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
n.create_subscription(TFMessage, "/tf", cb, 50)
n.create_subscription(TFMessage, "/tf_static", cb, 50)
t0 = time.time()
while time.time() - t0 < 12: rclpy.spin_once(n, timeout_sec=0.1)
for (p, c), t in sorted(seen.items()):
    tr, q = t.translation, t.rotation
    print(f"{p:>14} -> {c:<32} t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
