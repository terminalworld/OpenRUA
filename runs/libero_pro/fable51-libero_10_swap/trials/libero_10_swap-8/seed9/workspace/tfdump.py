import rclpy
from tf2_msgs.msg import TFMessage
rclpy.init(); n = rclpy.create_node("tfdump")
got = []
n.create_subscription(TFMessage, "/tf", got.append, 10)
import time
end = time.time()+5
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
seen = {}
for m in got:
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
for k, tr in seen.items():
    p, q = tr.translation, tr.rotation
    print(k, f"t=({p.x:.4f},{p.y:.4f},{p.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
