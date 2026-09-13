import rclpy
from tf2_msgs.msg import TFMessage
rclpy.init(); node = rclpy.create_node("tfdump")
got = []
node.create_subscription(TFMessage, "/tf", got.append, 10)
import time
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.3)
seen = {}
for m in got:
    for t in m.transforms:
        tr, q = t.transform.translation, t.transform.rotation
        seen[(t.header.frame_id, t.child_frame_id)] = (round(tr.x,4), round(tr.y,4), round(tr.z,4), round(q.x,4), round(q.y,4), round(q.z,4), round(q.w,4))
for k, v in sorted(seen.items()):
    print(k, v)
